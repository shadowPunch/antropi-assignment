"""Small helpers around nav2_simple_commander's BasicNavigator, shared by this package's scripts."""
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

SPAWN = (-2.0, -0.5, 0.0)  # default spawn pose in tb3_simulation_launch.py (x, y, yaw)

# All goals in this package are written in Gazebo world coordinates.
# - AMCL mode: the saved map's origin is shifted so its frame equals the world frame.
# - SLAM mode: slam_toolbox starts the map frame at the spawn pose, so shift goals by SPAWN.
#   (Spawn yaw is 0, so a translation is enough.)


def world_to_slam_map(x: float, y: float) -> tuple[float, float]:
    return x - SPAWN[0], y - SPAWN[1]


def make_navigator() -> BasicNavigator:
    nav = BasicNavigator()
    # Stamp messages with Gazebo time, the clock AMCL and the costmaps use.
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    return nav


def make_pose(nav: BasicNavigator, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def wait_until_active(nav: BasicNavigator, node_name: str) -> None:
    """Poll a lifecycle node's state, retrying on dropped replies.

    BasicNavigator.waitUntilNav2Active() waits on get_state with no timeout; if the very
    first reply is lost during discovery it hangs forever (seen on Lyrical + Fast DDS).
    """
    client = nav.create_client(GetState, f'{node_name}/get_state')
    while not client.wait_for_service(timeout_sec=1.0):
        nav.info(f'waiting for {node_name}/get_state ...')
    while True:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
        if future.done() and future.result().current_state.label == 'active':
            nav.info(f'{node_name} is active')
            break
        if not future.done():
            client.remove_pending_request(future)
    nav.destroy_client(client)


def set_initial_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> None:
    """Publish the initial pose until AMCL answers with /amcl_pose (like RViz '2D Pose Estimate')."""
    while True:
        nav.setInitialPose(make_pose(nav, x, y, yaw))  # also clears nav.initial_pose_received
        rclpy.spin_once(nav, timeout_sec=1.0)
        if nav.initial_pose_received:
            nav.info('AMCL accepted the initial pose')
            return


def wait_for_point_in_costmap(nav: BasicNavigator, x: float, y: float, timeout_s: float = 15.0) -> bool:
    """Wait until map-frame (x, y) lies inside the global costmap.

    While mapping, the costmap only covers what the lidar has seen so far, and the planner
    rejects goals outside it ("Goal Coordinates ... was outside bounds").
    """
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        meta = nav.getGlobalCostmap().metadata
        ox, oy = meta.origin.position.x, meta.origin.position.y
        if ox <= x < ox + meta.size_x * meta.resolution and oy <= y < oy + meta.size_y * meta.resolution:
            return True
        rclpy.spin_once(nav, timeout_sec=1.0)
    nav.warn(f'({x:.2f}, {y:.2f}) still outside the global costmap after {timeout_s:.0f} s')
    return False


@dataclass
class GoalOutcome:
    succeeded: bool
    result: str         # TaskResult name
    duration_s: float   # sim time
    recoveries: int     # recovery behaviors (spin, back up, ...) the BT triggered
    error: str = ''


def go_to(nav: BasicNavigator, x: float, y: float, yaw: float = 0.0, slam: bool = False,
          timeout_s: float = 180.0) -> GoalOutcome:
    """Drive to world (x, y, yaw), like RViz 'Nav2 Goal'."""
    mx, my = world_to_slam_map(x, y) if slam else (x, y)
    if slam:
        wait_for_point_in_costmap(nav, mx, my)
    nav.goToPose(make_pose(nav, mx, my, yaw))
    start = nav.get_clock().now()
    last_print = -5.0
    recoveries = 0
    elapsed = 0.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
        if fb:
            recoveries = max(recoveries, fb.number_of_recoveries)
        if fb and elapsed - last_print >= 5.0:
            last_print = elapsed
            print(f'  [{elapsed:5.1f}s] distance remaining: {fb.distance_remaining:.2f} m', flush=True)
        if elapsed > timeout_s:
            nav.cancelTask()
            break

    result = nav.getResult()
    code, msg = nav.getTaskError()
    error = f'error {code}: {msg}' if code else ''
    print(f'  goal world ({x:.2f}, {y:.2f}) -> {result.name} in {elapsed:.1f}s'
          + (f' ({error})' if error else ''), flush=True)
    return GoalOutcome(result == TaskResult.SUCCEEDED, result.name, elapsed, recoveries, error)
