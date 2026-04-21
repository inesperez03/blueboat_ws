from typing import List, Optional

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy


class BlueBoatTeleop(Node):
    def __init__(self) -> None:
        super().__init__('blueboat_teleop')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_z', 3)

        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_angular_z', 1.0)
        self.declare_parameter('turbo_scale_linear_x', 1.5)
        self.declare_parameter('turbo_scale_angular_z', 1.5)

        self.declare_parameter('enable_button', 5)
        self.declare_parameter('turbo_button', -1)
        self.declare_parameter('require_enable_button', False)

        self.declare_parameter('mode_modifier_button', 5)
        self.declare_parameter('alternate_mode_modifier_button', -1)
        self.declare_parameter('body_force_button', 4)
        self.declare_parameter('body_velocity_button', 2)
        self.declare_parameter('stop_controllers_button', -1)

        self.declare_parameter('controller_manager_name', '/controller_manager')
        self.declare_parameter('body_force_controller_name', 'body_force_controller')
        self.declare_parameter('body_velocity_controller_name', 'body_velocity_controller')
        self.declare_parameter('thruster_test_controller_name', 'thruster_test_controller')

        self.declare_parameter('axis_deadzone', 0.1)
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_angular_z', False)

        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.axis_linear_x = int(self.get_parameter('axis_linear_x').value)
        self.axis_angular_z = int(self.get_parameter('axis_angular_z').value)

        self.scale_linear_x = float(self.get_parameter('scale_linear_x').value)
        self.scale_angular_z = float(self.get_parameter('scale_angular_z').value)
        self.turbo_scale_linear_x = float(
            self.get_parameter('turbo_scale_linear_x').value)
        self.turbo_scale_angular_z = float(
            self.get_parameter('turbo_scale_angular_z').value)

        self.enable_button = int(self.get_parameter('enable_button').value)
        self.turbo_button = int(self.get_parameter('turbo_button').value)
        self.require_enable_button = bool(
            self.get_parameter('require_enable_button').value)

        self.mode_modifier_button = int(self.get_parameter('mode_modifier_button').value)
        self.alternate_mode_modifier_button = int(
            self.get_parameter('alternate_mode_modifier_button').value)
        self.body_force_button = int(self.get_parameter('body_force_button').value)
        self.body_velocity_button = int(self.get_parameter('body_velocity_button').value)
        self.stop_controllers_button = int(
            self.get_parameter('stop_controllers_button').value)

        self.controller_manager_name = str(
            self.get_parameter('controller_manager_name').value)
        self.body_force_controller_name = str(
            self.get_parameter('body_force_controller_name').value)
        self.body_velocity_controller_name = str(
            self.get_parameter('body_velocity_controller_name').value)
        self.thruster_test_controller_name = str(
            self.get_parameter('thruster_test_controller_name').value)

        self.axis_deadzone = float(self.get_parameter('axis_deadzone').value)
        self.invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self.invert_angular_z = bool(self.get_parameter('invert_angular_z').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

        switch_service = f'{self.controller_manager_name}/switch_controller'
        self.switch_client = self.create_client(SwitchController, switch_service)

        self.last_joy_msg: Optional[Joy] = None
        self.last_joy_time = None
        self.last_published_twist = Twist()
        self.zero_sent = False
        self.pending_switch = False
        self.active_mode = 'none'
        self.prev_force_combo_pressed = False
        self.prev_velocity_combo_pressed = False
        self.warned_waiting_for_joy = False
        self.pending_switch_steps = []

        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.get_logger().info('BlueBoat teleop ready')
        self.get_logger().info(f'Listening on joy topic: {self.joy_topic}')
        self.get_logger().info(f'Publishing cmd_vel on: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Mode switching on {switch_service}: RB+LB toggles {self.body_force_controller_name}, '
            f'RB+X toggles {self.body_velocity_controller_name} through '
            f'{self.body_force_controller_name}')

    def joy_callback(self, msg: Joy) -> None:
        self.last_joy_msg = msg
        self.last_joy_time = self.get_clock().now()
        self.zero_sent = False
        self.handle_mode_switch_shortcuts(msg.buttons)

    def handle_mode_switch_shortcuts(self, buttons: List[int]) -> None:
        modifier_pressed = self.mode_modifier_pressed(buttons)
        force_combo_pressed = (
            modifier_pressed and
            self.button_pressed(buttons, self.body_force_button)
        )
        velocity_combo_pressed = (
            modifier_pressed and
            self.button_pressed(buttons, self.body_velocity_button)
        )
        if force_combo_pressed and not self.prev_force_combo_pressed:
            if self.active_mode == 'force':
                self.switch_mode(
                    start=[],
                    stop=[
                        self.body_force_controller_name,
                        self.thruster_test_controller_name,
                    ],
                    label='body force disabled',
                    target_mode='none',
                )
            elif self.active_mode == 'velocity':
                self.switch_mode(
                    start=[],
                    stop=[
                        self.body_velocity_controller_name,
                        self.body_force_controller_name,
                        self.thruster_test_controller_name,
                    ],
                    label='body force and velocity disabled',
                    target_mode='none',
                )
            else:
                self.switch_mode(
                    start=[self.body_force_controller_name],
                    stop=[
                        self.body_velocity_controller_name,
                        self.thruster_test_controller_name,
                    ],
                    label='body force',
                    target_mode='force',
                )

        elif velocity_combo_pressed and not self.prev_velocity_combo_pressed:
            if self.active_mode == 'velocity':
                self.switch_mode(
                    start=[],
                    stop=[self.body_velocity_controller_name],
                    label='body velocity disabled',
                    target_mode='force',
                )
            else:
                # Do this in two controller-manager calls. Activating both chainable
                # controllers in one request is more fragile across ros2_control versions.
                self.switch_mode_sequence([
                    {
                        'start': [self.body_force_controller_name],
                        'stop': [self.thruster_test_controller_name],
                        'label': 'body force prerequisite',
                        'target_mode': None,
                    },
                    {
                        'start': [self.body_velocity_controller_name],
                        'stop': [],
                        'label': 'body velocity',
                        'target_mode': 'velocity',
                    },
                ])

        self.prev_force_combo_pressed = force_combo_pressed
        self.prev_velocity_combo_pressed = velocity_combo_pressed

    def mode_modifier_pressed(self, buttons: List[int]) -> bool:
        return (
            self.button_pressed(buttons, self.mode_modifier_button) or
            self.button_pressed(buttons, self.alternate_mode_modifier_button)
        )

    def switch_mode_sequence(self, steps: List[dict]) -> None:
        if self.pending_switch:
            return

        self.pending_switch_steps = steps[1:]
        first_step = steps[0]
        self.switch_mode(
            start=first_step['start'],
            stop=first_step['stop'],
            label=first_step['label'],
            target_mode=first_step.get('target_mode'),
        )

    def switch_mode(
        self,
        start: List[str],
        stop: List[str],
        label: str,
        target_mode: Optional[str] = None,
    ) -> None:
        if self.pending_switch:
            return

        if not self.switch_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warning('switch_controller service not available')
            return

        self.pending_switch = True
        self.publish_if_changed(Twist())

        request = SwitchController.Request()
        request.activate_controllers = start
        request.deactivate_controllers = stop
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True

        future = self.switch_client.call_async(request)
        future.add_done_callback(
            lambda fut, switch_label=label, switch_target_mode=target_mode:
            self._handle_switch_result(fut, switch_label, switch_target_mode))

    def _handle_switch_result(self, future, label: str, target_mode: Optional[str]) -> None:
        self.pending_switch = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS future failure path
            self.get_logger().error(f'Failed to switch to {label} mode: {exc}')
            return

        if response is not None and response.ok:
            if target_mode is not None:
                self.active_mode = target_mode
            self.get_logger().info(f'Controller switch complete: {label}')
            if self.pending_switch_steps:
                next_step = self.pending_switch_steps.pop(0)
                self.switch_mode(
                    start=next_step['start'],
                    stop=next_step['stop'],
                    label=next_step['label'],
                    target_mode=next_step.get('target_mode'),
                )
        else:
            self.pending_switch_steps = []
            self.get_logger().warning(f'Controller switch to {label} mode was rejected')

    def publish_command(self) -> None:
        twist = Twist()

        if self.last_joy_msg is None or self.last_joy_time is None:
            if not self.warned_waiting_for_joy:
                self.get_logger().warning(
                    f'No Joy messages received yet on {self.joy_topic}. '
                    'Check that joy_node is running and the gamepad is visible in /dev/input.')
                self.warned_waiting_for_joy = True
            self.publish_if_changed(twist)
            return

        age = self.get_clock().now() - self.last_joy_time
        if age > Duration(seconds=self.command_timeout):
            self.publish_if_changed(twist)
            return

        if self.pending_switch:
            self.publish_if_changed(twist)
            return

        if not self.teleop_enabled(self.last_joy_msg.buttons):
            self.publish_if_changed(twist)
            return

        linear_scale = self.scale_linear_x
        angular_scale = self.scale_angular_z

        if self.button_pressed(self.last_joy_msg.buttons, self.turbo_button):
            linear_scale = self.turbo_scale_linear_x
            angular_scale = self.turbo_scale_angular_z

        linear_x = self.axis_value(self.last_joy_msg.axes, self.axis_linear_x)
        angular_z = self.axis_value(self.last_joy_msg.axes, self.axis_angular_z)

        if self.invert_linear_x:
            linear_x *= -1.0
        if self.invert_angular_z:
            angular_z *= -1.0

        twist.linear.x = linear_x * linear_scale
        twist.angular.z = angular_z * angular_scale

        self.publish_if_changed(twist)

    def teleop_enabled(self, buttons: List[int]) -> bool:
        if not self.require_enable_button:
            return True
        return self.button_pressed(buttons, self.enable_button)

    def axis_value(self, axes: List[float], index: int) -> float:
        if index < 0 or index >= len(axes):
            return 0.0

        value = float(axes[index])
        if abs(value) < self.axis_deadzone:
            return 0.0
        return value

    @staticmethod
    def button_pressed(buttons: List[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        return bool(buttons[index])

    def publish_if_changed(self, twist: Twist) -> None:
        is_zero = self.is_zero_twist(twist)
        if self.twist_equal(self.last_published_twist, twist) and is_zero and self.zero_sent:
            return

        self.cmd_pub.publish(twist)
        self.last_published_twist = twist
        self.zero_sent = is_zero

    @staticmethod
    def twist_equal(lhs: Twist, rhs: Twist, eps: float = 1e-9) -> bool:
        return (
            abs(lhs.linear.x - rhs.linear.x) < eps and
            abs(lhs.linear.y - rhs.linear.y) < eps and
            abs(lhs.linear.z - rhs.linear.z) < eps and
            abs(lhs.angular.x - rhs.angular.x) < eps and
            abs(lhs.angular.y - rhs.angular.y) < eps and
            abs(lhs.angular.z - rhs.angular.z) < eps
        )

    @staticmethod
    def is_zero_twist(twist: Twist, eps: float = 1e-9) -> bool:
        return (
            abs(twist.linear.x) < eps and
            abs(twist.linear.y) < eps and
            abs(twist.linear.z) < eps and
            abs(twist.angular.x) < eps and
            abs(twist.angular.y) < eps and
            abs(twist.angular.z) < eps
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueBoatTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
