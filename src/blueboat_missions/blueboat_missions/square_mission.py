import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sura_msgs.msg import Navigator


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class SquareMissionNode(Node):
    def __init__(self) -> None:
        super().__init__('blueboat_square_mission')

        self.declare_parameter('navigator_topic', '/navigator_msg')
        self.declare_parameter('setpoint_topic', '/body_position/setpoint')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('arrival_radius', 0.45)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('square_side', 2.0)
        self.declare_parameter('square_heading', 0.0)
        self.declare_parameter('start_from_current_pose', True)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('clockwise', False)
        self.declare_parameter('hold_final_goal', True)
        self.declare_parameter('pool_corners', False)
        self.declare_parameter('pool_length_x', 12.0)
        self.declare_parameter('pool_width_y', 6.0)
        self.declare_parameter('pool_margin', 1.0)
        self.declare_parameter('use_pool_bounds', False)
        self.declare_parameter('pool_min_x', -5.0)
        self.declare_parameter('pool_max_x', 5.0)
        self.declare_parameter('pool_min_y', -2.0)
        self.declare_parameter('pool_max_y', 2.0)

        self.navigator_topic = self.get_parameter('navigator_topic').value
        self.setpoint_topic = self.get_parameter('setpoint_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.square_side = float(self.get_parameter('square_side').value)
        self.square_heading = float(self.get_parameter('square_heading').value)
        self.start_from_current_pose = bool(self.get_parameter('start_from_current_pose').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.clockwise = bool(self.get_parameter('clockwise').value)
        self.hold_final_goal = bool(self.get_parameter('hold_final_goal').value)
        self.pool_corners = bool(self.get_parameter('pool_corners').value)
        self.pool_length_x = float(self.get_parameter('pool_length_x').value)
        self.pool_width_y = float(self.get_parameter('pool_width_y').value)
        self.pool_margin = float(self.get_parameter('pool_margin').value)
        self.use_pool_bounds = bool(self.get_parameter('use_pool_bounds').value)
        self.pool_min_x = float(self.get_parameter('pool_min_x').value)
        self.pool_max_x = float(self.get_parameter('pool_max_x').value)
        self.pool_min_y = float(self.get_parameter('pool_min_y').value)
        self.pool_max_y = float(self.get_parameter('pool_max_y').value)

        self.navigator_sub = self.create_subscription(
            Navigator,
            self.navigator_topic,
            self.navigator_callback,
            10,
        )
        self.setpoint_pub = self.create_publisher(PoseStamped, self.setpoint_topic, 10)

        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.latest_navigator: Optional[Navigator] = None
        self.origin_latched = False
        self.mission_started = False
        self.mission_complete = False
        self.current_waypoint_index = 0
        self.hold_started_at = None
        self.waypoints: List[PoseStamped] = []

        self.get_logger().info('Square mission node ready')

    def navigator_callback(self, msg: Navigator) -> None:
        self.latest_navigator = msg

    def build_mission(self) -> None:
        if self.pool_corners:
            self.build_pool_corners_mission()
            return

        if self.start_from_current_pose and self.latest_navigator is not None:
            self.origin_x = self.latest_navigator.position.position.x
            self.origin_y = self.latest_navigator.position.position.y

        heading = self.square_heading
        c = math.cos(heading)
        s = math.sin(heading)
        side = self.square_side
        offsets = [
            (side, 0.0),
            (side, side if not self.clockwise else -side),
            (0.0, side if not self.clockwise else -side),
            (0.0, 0.0),
        ]

        self.waypoints = []
        for dx, dy in offsets:
            wp = PoseStamped()
            wp.header.frame_id = self.frame_id
            wp.pose.position.x = self.origin_x + c * dx - s * dy
            wp.pose.position.y = self.origin_y + s * dx + c * dy
            wp.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
            wp.pose.orientation.x = qx
            wp.pose.orientation.y = qy
            wp.pose.orientation.z = qz
            wp.pose.orientation.w = qw
            self.waypoints.append(wp)

        self.origin_latched = True
        self.mission_started = True
        self.mission_complete = False
        self.current_waypoint_index = 0
        self.hold_started_at = None

        self.get_logger().info(
            f'Square mission built at origin=({self.origin_x:.2f}, {self.origin_y:.2f}), '
            f'side={self.square_side:.2f}, heading={self.square_heading:.2f}'
        )

    def build_pool_corners_mission(self) -> None:
        if self.use_pool_bounds:
            min_x = self.pool_min_x
            max_x = self.pool_max_x
            min_y = self.pool_min_y
            max_y = self.pool_max_y
        else:
            min_x = self.pool_margin
            max_x = self.pool_length_x - self.pool_margin
            min_y = self.pool_margin
            max_y = self.pool_width_y - self.pool_margin

        if min_x >= max_x or min_y >= max_y:
            self.get_logger().error('Invalid pool-corners bounds')
            return

        corners = [
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
            (max_x, min_y),
        ]
        if self.clockwise:
            corners = [
                (min_x, min_y),
                (min_x, max_y),
                (max_x, max_y),
                (max_x, min_y),
                (min_x, min_y),
            ]

        self.waypoints = []
        for index, (x, y) in enumerate(corners):
            wp = PoseStamped()
            wp.header.frame_id = self.frame_id
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = 0.0
            if index < len(corners) - 1:
                next_x, next_y = corners[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = self.target_yaw
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            wp.pose.orientation.x = qx
            wp.pose.orientation.y = qy
            wp.pose.orientation.z = qz
            wp.pose.orientation.w = qw
            self.waypoints.append(wp)

        self.origin_latched = True
        self.mission_started = True
        self.mission_complete = False
        self.current_waypoint_index = 0
        self.hold_started_at = None

        self.get_logger().info(
            f'Pool-corners mission built with x=[{min_x:.2f}, {max_x:.2f}], '
            f'y=[{min_y:.2f}, {max_y:.2f}], margin={self.pool_margin:.2f}'
        )

    def current_waypoint(self) -> Optional[PoseStamped]:
        if not self.waypoints:
            return None
        if self.current_waypoint_index >= len(self.waypoints):
            return self.waypoints[-1]
        return self.waypoints[self.current_waypoint_index]

    def distance_to_waypoint(self, waypoint: PoseStamped) -> Optional[float]:
        if self.latest_navigator is None:
            return None
        dx = waypoint.pose.position.x - self.latest_navigator.position.position.x
        dy = waypoint.pose.position.y - self.latest_navigator.position.position.y
        return math.hypot(dx, dy)

    def publish_waypoint(self, waypoint: PoseStamped) -> None:
        waypoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(waypoint)

    def timer_callback(self) -> None:
        if self.mission_complete and self.hold_final_goal:
            waypoint = self.current_waypoint()
            if waypoint is not None:
                self.publish_waypoint(waypoint)
            return

        if not self.mission_started:
            if self.start_from_current_pose and self.latest_navigator is None:
                self.get_logger().info('Waiting for /navigator_msg to start square mission...')
                return
            self.build_mission()

        waypoint = self.current_waypoint()
        if waypoint is None:
            return

        self.publish_waypoint(waypoint)

        distance = self.distance_to_waypoint(waypoint)
        if distance is None:
            return

        now = self.get_clock().now()
        if distance <= self.arrival_radius:
            if self.hold_started_at is None:
                self.hold_started_at = now
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_index + 1} reached at '
                    f'dist={distance:.3f}. Holding for {self.hold_time:.2f} s'
                )
            elif (now - self.hold_started_at).nanoseconds >= int(self.hold_time * 1e9):
                self.current_waypoint_index += 1
                self.hold_started_at = None
                if self.current_waypoint_index >= len(self.waypoints):
                    self.mission_complete = True
                    self.get_logger().info('Square mission complete')
                else:
                    self.get_logger().info(
                        f'Advancing to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}'
                    )
        else:
            self.hold_started_at = None


def main() -> None:
    rclpy.init()
    node = SquareMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
