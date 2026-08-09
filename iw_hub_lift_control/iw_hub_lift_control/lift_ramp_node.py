#!/usr/bin/env python3
"""
Publishes acceleration-limited JointState commands to smoothly ramp the
iw_hub lift joint to a target position.

Uses a trapezoidal velocity profile - the commanded velocity itself ramps up
and back down at a bounded acceleration, rather than stepping instantly from
0 to max_velocity and snapping back to 0 at the goal. A velocity step is an
infinite-acceleration jolt at both ends of the move; on a lift carrying an
unsecured load (e.g. a dolly resting on the platform via friction only),
that jolt is enough to shift the load even though the *position* trajectory
looks smooth.

Send a target with:
  ros2 topic pub --once /lift_position_cmd std_msgs/msg/Float64 "data: 0.09"

Or call the action-free simple trigger via parameter at launch:
  ros2 run iw_hub_lift_control lift_ramp_node --ros-args -p target_position:=0.09
"""

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LiftRampNode(Node):

    def __init__(self):
        super().__init__('lift_ramp_node')

        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('lift_cmd_topic', '/lift_position_cmd')
        self.declare_parameter('max_velocity', 0.01)       # m/s, tune to your lift stroke/limits
        # m/s^2 - how quickly the commanded velocity itself ramps up/down.
        # Lower = gentler start/stop, less likely to jolt an unsecured load,
        # but the lift takes longer to reach max_velocity.
        self.declare_parameter('max_acceleration', 0.01)
        self.declare_parameter('publish_rate', 50.0)        # Hz
        self.declare_parameter('effort', 10.0)
        self.declare_parameter('goal_tolerance', 0.0005)    # m
        self.declare_parameter('lower_limit', 0.0)
        self.declare_parameter('upper_limit', 0.1)         # matches lift_joint upperLimit in the USD
        self.declare_parameter('target_position', float('nan'))  # optional: ramp immediately on startup

        self.joint_name = self.get_parameter('joint_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.effort = self.get_parameter('effort').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lower_limit = self.get_parameter('lower_limit').value
        self.upper_limit = self.get_parameter('upper_limit').value

        cmd_topic = self.get_parameter('command_topic').value
        js_topic = self.get_parameter('joint_states_topic').value
        lift_cmd_topic = self.get_parameter('lift_cmd_topic').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(JointState, cmd_topic, qos)
        self.create_subscription(JointState, js_topic, self._joint_states_cb, qos)
        self.create_subscription(Float64, lift_cmd_topic, self._target_cb, 10)

        self.current_position = 0.0
        self.current_velocity = 0.0
        self.have_feedback = False
        self.target_position = None
        self.timer = None

        self.get_logger().info(
            f"lift_ramp_node ready: joint='{self.joint_name}' max_vel={self.max_velocity} m/s "
            f"max_accel={self.max_acceleration} m/s^2 rate={self.publish_rate} Hz, "
            f"limits=[{self.lower_limit}, {self.upper_limit}]")

        startup_target = self.get_parameter('target_position').value
        if startup_target == startup_target:  # not NaN
            self._start_ramp(startup_target)

    def _joint_states_cb(self, msg: JointState):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            if idx < len(msg.position):
                self.current_position = msg.position[idx]
                self.have_feedback = True

    def _target_cb(self, msg: Float64):
        self._start_ramp(msg.data)

    def _start_ramp(self, target: float):
        target = max(self.lower_limit, min(self.upper_limit, target))
        self.target_position = target
        self.get_logger().info(
            f"Ramping '{self.joint_name}' from {self.current_position:.4f} to "
            f"{self.target_position:.4f} (max_vel={self.max_velocity} m/s, "
            f"max_accel={self.max_acceleration} m/s^2)"
            + ("" if self.have_feedback else " (no /joint_states feedback yet, using internal estimate)"))

        if self.timer is None:
            period = 1.0 / self.publish_rate
            self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        if self.target_position is None:
            return

        dt = 1.0 / self.publish_rate
        delta = self.target_position - self.current_position
        direction = 1.0 if delta > 0 else -1.0
        distance_remaining = abs(delta)

        if distance_remaining <= self.goal_tolerance and abs(self.current_velocity) < 1e-4:
            self.current_position = self.target_position
            self.current_velocity = 0.0
            self._publish(velocity=0.0)
            self.get_logger().info(f"Reached target {self.target_position:.4f}")
            self.target_position = None
            self.destroy_timer(self.timer)
            self.timer = None
            return

        # Trapezoidal profile: figure out the fastest speed we could be going
        # right now and still decelerate to a stop exactly at the target
        # (v^2 = 2 * a * d, solved for v), then clamp to max_velocity. This
        # gives the ramp-down automatically without a separate deceleration
        # phase to track.
        max_speed_to_stop_in_time = (2.0 * self.max_acceleration * distance_remaining) ** 0.5
        desired_speed = min(self.max_velocity, max_speed_to_stop_in_time)
        desired_velocity = direction * desired_speed

        # Ramp actual commanded velocity toward desired_velocity at a bounded
        # acceleration - this is what actually eliminates the jolt, rather
        # than jumping straight to desired_velocity each tick.
        max_delta_v = self.max_acceleration * dt
        velocity_error = desired_velocity - self.current_velocity
        velocity_step = max(-max_delta_v, min(max_delta_v, velocity_error))
        self.current_velocity += velocity_step

        # Don't overshoot the target in one tick.
        step = self.current_velocity * dt
        if abs(step) > distance_remaining:
            step = direction * distance_remaining
            self.current_velocity = 0.0

        self.current_position += step
        self._publish(velocity=self.current_velocity)

    def _publish(self, velocity: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.current_position]
        msg.velocity = [velocity]
        msg.effort = [self.effort]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiftRampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()