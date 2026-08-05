#!/usr/bin/env python3
"""
Exposes a /mission service (iw_hub_pick_place_msgs/srv/RunMission).

  ros2 service call /mission iw_hub_pick_place_msgs/srv/RunMission \
    "{pick_up: goal_a, destination: goal_b}"

On each call: navigates to `pick_up`, raises the lift, navigates to
`destination`, lowers the lift. Runs once per call - no auto-looping.
Calls are serialized: a second call while one is in progress is rejected.

Requires:
  - Nav2 stack up and localized (see iw_hub_navigation)
  - lift_ramp_node running (see iw_hub_lift_control), listening on /lift_position_cmd
"""

import math
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from iw_hub_pick_place_msgs.srv import RunMission


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MissionExecutorNode(Node):

    def __init__(self):
        super().__init__('mission_executor_node')

        self.declare_parameter('goals_file', '')
        self.declare_parameter('lift_cmd_topic', '/lift_position_cmd')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('lift_joint_name', 'lift_joint')
        self.declare_parameter('lift_tolerance', 0.002)
        self.declare_parameter('lift_timeout_sec', 30.0)
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('nav_timeout_sec', 300.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('retreat_distance', 1.)
        self.declare_parameter('retreat_speed', 0.15)
        self.declare_parameter('retreat_timeout_sec', 20.0)

        goals_file = self.get_parameter('goals_file').value
        if not goals_file:
            raise RuntimeError(
                "Parameter 'goals_file' is required - pass the full path to "
                "pick_place_goals.yaml")

        with open(goals_file, 'r') as f:
            self.mission_config = yaml.safe_load(f)

        self.frame_id = self.mission_config.get('frame_id', 'map')
        self.waypoints = self.mission_config['waypoints']
        self.lift_levels = self.mission_config['lift']

        self.lift_topic = self.get_parameter('lift_cmd_topic').value
        self.lift_joint_name = self.get_parameter('lift_joint_name').value
        self.lift_tolerance = self.get_parameter('lift_tolerance').value
        self.lift_timeout_sec = self.get_parameter('lift_timeout_sec').value
        self.nav_timeout_sec = self.get_parameter('nav_timeout_sec').value
        self.retreat_distance = self.get_parameter('retreat_distance').value
        self.retreat_speed = self.get_parameter('retreat_speed').value
        self.retreat_timeout_sec = self.get_parameter('retreat_timeout_sec').value

        cb_group = ReentrantCallbackGroup()

        nav_action_name = self.get_parameter('nav_action_name').value
        self.nav_client = ActionClient(
            self, NavigateToPose, nav_action_name, callback_group=cb_group)

        self.lift_pub = self.create_publisher(Float64, self.lift_topic, 10)
        self.current_lift_position = None
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._joint_states_cb, 10, callback_group=cb_group)

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.current_odom_xy = None
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, 10, callback_group=cb_group)

        # Only one mission runs at a time; a lock serializes concurrent calls.
        self._mission_lock = threading.Lock()

        self.srv = self.create_service(
            RunMission, 'mission', self._handle_mission, callback_group=cb_group)

        self.get_logger().info(
            f"mission_executor_node ready. Waypoints available: {list(self.waypoints.keys())}. "
            "Call the /mission service to start a pick-and-place run.")

    def _joint_states_cb(self, msg: JointState):
        if self.lift_joint_name in msg.name:
            idx = msg.name.index(self.lift_joint_name)
            if idx < len(msg.position):
                self.current_lift_position = msg.position[idx]

    def _odom_cb(self, msg: Odometry):
        self.current_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # -- Service callback ---------------------------------------------------

    def _handle_mission(self, request, response):
        pick_up = request.pick_up
        destination = request.destination

        if pick_up not in self.waypoints:
            response.success = False
            response.message = f"Unknown pick_up waypoint '{pick_up}'"
            self.get_logger().error(response.message)
            return response

        if destination not in self.waypoints:
            response.success = False
            response.message = f"Unknown destination waypoint '{destination}'"
            self.get_logger().error(response.message)
            return response

        if not self._mission_lock.acquire(blocking=False):
            response.success = False
            response.message = "A mission is already running - try again once it finishes"
            self.get_logger().warn(response.message)
            return response

        try:
            self.get_logger().info(f"Mission started: pick_up='{pick_up}' destination='{destination}'")

            if not self.send_nav_goal(pick_up):
                response.success = False
                response.message = f"Failed to reach pick_up waypoint '{pick_up}'"
                return response

            if not self.send_lift_command('raised'):
                response.success = False
                response.message = (
                    f"Lift did not confirm reaching 'raised' before timeout - "
                    f"aborting before driving to '{destination}' to avoid moving "
                    f"with the dolly only partially lifted")
                return response

            if not self.send_nav_goal(destination):
                response.success = False
                response.message = f"Failed to reach destination waypoint '{destination}'"
                return response

            if not self.send_lift_command('lowered'):
                response.success = False
                response.message = (
                    "Reached destination but lift did not confirm reaching "
                    "'lowered' before timeout - check the lift manually")
                return response

            if not self.drive_straight(self.retreat_distance, self.retreat_speed):
                self.get_logger().warn(
                    "Retreat move did not complete fully - the dolly may still "
                    "be close to the robot, drive carefully on the next goal")

            response.success = True
            response.message = f"Mission complete: {pick_up} -> {destination}"
            self.get_logger().info(response.message)
            return response
        finally:
            self._mission_lock.release()

    # -- Straight-line retreat (clears the dropped dolly, which Nav2 can't see) --

    def drive_straight(self, distance: float, speed: float) -> bool:
        """Drives forward in a dead-straight line for `distance` meters,
        bypassing Nav2/the planner entirely so no turning can clip whatever
        was just dropped off (the dolly isn't sensed as a costmap obstacle)."""
        if self.current_odom_xy is None:
            self.get_logger().warn(
                "No /odom data yet - skipping retreat move, drive manually if needed")
            return False

        start_x, start_y = self.current_odom_xy
        self.get_logger().info(f"Retreating straight {distance} m at {speed} m/s")

        twist = Twist()
        twist.linear.x = speed

        start_time = time.time()
        traveled = 0.0
        rate_hz = 20.0

        while traveled < distance:
            if time.time() - start_time > self.retreat_timeout_sec:
                self.get_logger().warn(
                    f"Retreat move timed out after {self.retreat_timeout_sec}s "
                    f"(traveled {traveled:.2f}/{distance} m) - stopping")
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate_hz)
            if self.current_odom_xy is not None:
                x, y = self.current_odom_xy
                traveled = math.hypot(x - start_x, y - start_y)

        # Publish zero twist a couple of times to make sure it actually stops.
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        time.sleep(0.05)
        self.cmd_vel_pub.publish(stop)

        self.get_logger().info(f"Retreat complete, traveled {traveled:.2f} m")
        return traveled >= distance * 0.9  # allow a little slack

    # -- Nav2 -----------------------------------------------------------

    def send_nav_goal(self, waypoint_name: str) -> bool:
        wp = self.waypoints[waypoint_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(wp['yaw']))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Navigating to '{waypoint_name}' -> x={wp['x']}, y={wp['y']}, yaw={wp['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available")
            return False

        done_event = threading.Event()
        result_holder = {}

        def goal_response_cb(future):
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                result_holder['accepted'] = False
                done_event.set()
                return
            result_holder['accepted'] = True
            result_holder['goal_handle'] = goal_handle

            def result_cb(future2):
                result_holder['status'] = future2.result().status
                done_event.set()

            goal_handle.get_result_async().add_done_callback(result_cb)

        self.nav_client.send_goal_async(goal_msg).add_done_callback(goal_response_cb)

        if not done_event.wait(timeout=self.nav_timeout_sec):
            self.get_logger().error(
                f"Timed out waiting for navigation to '{waypoint_name}' after "
                f"{self.nav_timeout_sec}s - cancelling the goal")
            goal_handle = result_holder.get('goal_handle')
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            return False

        if not result_holder.get('accepted', False):
            self.get_logger().error(f"Goal to '{waypoint_name}' was rejected")
            return False

        status = result_holder.get('status')
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f"Reached '{waypoint_name}'")
            return True

        self.get_logger().error(f"Navigation to '{waypoint_name}' failed, status={status}")
        return False

    # -- Lift -------------------------------------------------------------

    def send_lift_command(self, lift_action: str) -> bool:
        target = float(self.lift_levels[lift_action])
        self.get_logger().info(f"Lift -> '{lift_action}' ({target} m)")
        msg = Float64()
        msg.data = target
        self.lift_pub.publish(msg)

        start = time.time()
        while time.time() - start < self.lift_timeout_sec:
            if self.current_lift_position is not None and \
                    abs(self.current_lift_position - target) <= self.lift_tolerance:
                self.get_logger().info(f"Lift reached '{lift_action}'")
                return True
            time.sleep(0.1)

        self.get_logger().warn(
            f"Lift did not confirm reaching '{lift_action}' within "
            f"{self.lift_timeout_sec}s - continuing anyway")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()