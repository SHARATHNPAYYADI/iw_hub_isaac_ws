#!/usr/bin/env python3
"""
Publishes velocity-limited JointState commands to smoothly ramp the iw_hub
lift joint to a target position, instead of jumping there in one message.

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
        self.declare_parameter('publish_rate', 50.0)        # Hz
        self.declare_parameter('effort', 10.0)
        self.declare_parameter('goal_tolerance', 0.0005)    # m
        self.declare_parameter('lower_limit', 0.0)
        self.declare_parameter('upper_limit', 0.1)         # matches lift_joint upperLimit in the USD
        self.declare_parameter('target_position', float('nan'))  # optional: ramp immediately on startup

        self.joint_name = self.get_parameter('joint_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
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
        self.have_feedback = False
        self.target_position = None
        self.timer = None

        step = self.max_velocity / self.publish_rate
        self.get_logger().info(
            f"lift_ramp_node ready: joint='{self.joint_name}' max_vel={self.max_velocity} m/s "
            f"rate={self.publish_rate} Hz (step={step:.5f} m/tick), "
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
            f"{self.target_position:.4f} at {self.max_velocity} m/s"
            + ("" if self.have_feedback else " (no /joint_states feedback yet, using internal estimate)"))

        if self.timer is None:
            period = 1.0 / self.publish_rate
            self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        if self.target_position is None:
            return

        delta = self.target_position - self.current_position
        max_step = self.max_velocity / self.publish_rate

        if abs(delta) <= self.goal_tolerance:
            self.current_position = self.target_position
            self._publish(velocity=0.0)
            self.get_logger().info(f"Reached target {self.target_position:.4f}")
            self.target_position = None
            self.destroy_timer(self.timer)
            self.timer = None
            return

        step = max(-max_step, min(max_step, delta))
        self.current_position += step
        commanded_velocity = self.max_velocity if step > 0 else -self.max_velocity
        self._publish(velocity=commanded_velocity)

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
