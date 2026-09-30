"""iw_hub MCP server - the tools Claude can call to control the robot.

Claude Code starts this over stdio (see MCP_CONFIG in server.py). Each tool is a
thin wrapper over a ROS 2 service or TF lookup, so ROS 2 and this workspace must
be sourced in the shell that starts the chat server:

  source /opt/ros/humble/setup.bash && source install/setup.bash
"""

import atexit
import math
import threading
from typing import Literal

from mcp.server.mcpserver import MCPServer

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.time import Time
    import tf2_ros

    from iw_hub_pick_place_msgs.srv import DriveStraight, ListWaypoints, Rotate, RunMission
    ROS_IMPORT_ERROR = None
except ImportError as e:  # reported on first tool call instead of crashing the server
    ROS_IMPORT_ERROR = e

MAP_FRAME = "map"
BASE_FRAME = "base_link"
MAX_DISTANCE_M = 10.0  # must not exceed manual_max_distance on mission_executor_node
MOVE_TIMEOUT_SEC = 90.0
MISSION_TIMEOUT_SEC = 900.0

mcp = MCPServer("robot")

_node = None
_tf_buffer = None
_spin_thread = None
_node_lock = threading.Lock()


def ros_node():
    """Start one rclpy node on first use, spun in a background thread."""
    global _node, _tf_buffer, _spin_thread
    if ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            f"ROS 2 is not available ({ROS_IMPORT_ERROR}). Start the chat server from a shell "
            "where /opt/ros/humble/setup.bash and the workspace install/setup.bash are sourced.")
    with _node_lock:
        if _node is None:
            rclpy.init()
            _node = rclpy.create_node("iw_hub_chat_mcp")
            _tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(_tf_buffer, _node)
            executor = SingleThreadedExecutor()
            executor.add_node(_node)
            _spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
            _spin_thread.start()
    return _node


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


@atexit.register
def _shutdown_ros():
    # Stop the spin thread cleanly; killing it mid-spin at exit aborts the process.
    if _node is not None:
        rclpy.try_shutdown()
        _spin_thread.join(timeout=2.0)


def call_service(srv_type, name: str, request, timeout_sec: float):
    node = ros_node()
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f"Service {name} is not available - is mission_executor_node running "
                "(ros2 launch iw_hub_navigation iw_hub_bringup.launch.py)?")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            raise RuntimeError(f"Service {name} did not answer within {timeout_sec:.0f}s")
        return future.result()
    finally:
        node.destroy_client(client)


@mcp.tool()
def move_robot(direction: Literal["forward", "backward", "left", "right"], amount: float) -> dict:
    """Move the iw_hub robot relative to where it is now.

    'forward'/'backward' drive in a straight line by `amount` meters (0-10).
    'left'/'right' turn in place by `amount` degrees (0-360).
    Straight moves bypass Nav2, so there is no obstacle avoidance.
    Blocks until the move finishes.
    """
    if direction in ("forward", "backward"):
        if not 0 <= amount <= MAX_DISTANCE_M:
            raise ValueError(f"amount must be between 0 and {MAX_DISTANCE_M} m for {direction}")
        req = DriveStraight.Request()
        req.distance = float(amount if direction == "forward" else -amount)
        res = call_service(DriveStraight, "/drive_straight", req, MOVE_TIMEOUT_SEC)
        result = {"success": res.success, "message": res.message, "traveled_m": round(res.traveled, 3)}
    else:
        if not 0 <= amount <= 360:
            raise ValueError(f"amount must be between 0 and 360 degrees for {direction}")
        req = Rotate.Request()
        req.angle = math.radians(amount if direction == "left" else -amount)
        res = call_service(Rotate, "/rotate", req, MOVE_TIMEOUT_SEC)
        result = {"success": res.success, "message": res.message,
                  "rotated_deg": round(math.degrees(res.rotated), 1)}
    if not res.success:
        raise RuntimeError(res.message)
    return result


@mcp.tool()
def get_robot_state() -> dict:
    """Return the robot's current pose in the map frame: x, y in meters and heading in degrees
    (0 = map +x axis, counter-clockwise positive)."""
    node = ros_node()
    deadline = node.get_clock().now() + Duration(seconds=3.0)
    while not _tf_buffer.can_transform(MAP_FRAME, BASE_FRAME, Time()):
        if node.get_clock().now() > deadline:
            raise RuntimeError(
                f"No {MAP_FRAME} -> {BASE_FRAME} transform - is Nav2 running and localized?")
        threading.Event().wait(0.1)
    t = _tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time()).transform
    q = t.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return {"x_m": round(t.translation.x, 3), "y_m": round(t.translation.y, 3),
            "heading_deg": round(math.degrees(yaw), 1)}


@mcp.tool()
def list_waypoints() -> list[dict]:
    """List the named waypoints (map frame, yaw in degrees) usable with run_mission."""
    res = call_service(ListWaypoints, "/list_waypoints", ListWaypoints.Request(), 10.0)
    return [
        {"name": n, "x_m": x, "y_m": y, "yaw_deg": round(math.degrees(yaw), 1), "description": d}
        for n, x, y, yaw, d in zip(res.names, res.x, res.y, res.yaw, res.description)
    ]


@mcp.tool()
def run_mission(pick_up: str, destination: str) -> dict:
    """Run a full pick-and-place: drive to the `pick_up` waypoint, raise the lift, drive to
    `destination`, lower the lift, then drive straight clear. Takes several minutes; blocks until done."""
    req = RunMission.Request()
    req.pick_up = pick_up
    req.destination = destination
    res = call_service(RunMission, "/mission", req, MISSION_TIMEOUT_SEC)
    if not res.success:
        raise RuntimeError(res.message)
    return {"success": True, "message": res.message}


if __name__ == "__main__":
    mcp.run()
