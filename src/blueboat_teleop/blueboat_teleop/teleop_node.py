from typing import List, Optional

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped, Twist, Wrench
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sura_msgs.msg import Navigator


class BlueBoatTeleop(Node):
    def __init__(self) -> None:
        super().__init__('blueboat_teleop')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('body_force_topic', '/body_force/command')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_z', 3)

        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_angular_z', 1.0)
        self.declare_parameter('turbo_scale_linear_x', 1.5)
        self.declare_parameter('turbo_scale_angular_z', 1.5)
        self.declare_parameter('scale_force_x', 12.0)
        self.declare_parameter('scale_torque_z', 3.0)
        self.declare_parameter('turbo_scale_force_x', 18.0)
        self.declare_parameter('turbo_scale_torque_z', 4.5)
        self.declare_parameter('force_filter_alpha', 0.25)
        self.declare_parameter('max_force_rate_x', 24.0)
        self.declare_parameter('max_torque_rate_z', 6.0)

        self.declare_parameter('enable_button', 5)
        self.declare_parameter('turbo_button', -1)
        self.declare_parameter('require_enable_button', False)

        self.declare_parameter('mode_modifier_button', 5)
        self.declare_parameter('alternate_mode_modifier_button', -1)
        self.declare_parameter('body_force_button', 4)
        self.declare_parameter('body_velocity_button', 2)
        self.declare_parameter('body_position_button', 0)
        self.declare_parameter('stop_controllers_button', -1)

        self.declare_parameter('controller_manager_name', '/controller_manager')
        self.declare_parameter('body_force_controller_name', 'body_force_controller')
        self.declare_parameter('body_velocity_controller_name', 'body_velocity_controller')
        self.declare_parameter('body_position_controller_name', 'body_position_controller')
        self.declare_parameter('thruster_test_controller_name', 'thruster_test_controller')
        self.declare_parameter('navigator_topic', '/navigator_msg')
        self.declare_parameter('position_setpoint_topic', '/body_position/setpoint')

        self.declare_parameter('axis_deadzone', 0.1)
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_angular_z', False)

        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.body_force_topic = self.get_parameter('body_force_topic').value
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
        self.scale_force_x = float(self.get_parameter('scale_force_x').value)
        self.scale_torque_z = float(self.get_parameter('scale_torque_z').value)
        self.turbo_scale_force_x = float(
            self.get_parameter('turbo_scale_force_x').value)
        self.turbo_scale_torque_z = float(
            self.get_parameter('turbo_scale_torque_z').value)
        self.force_filter_alpha = self.clamp(
            float(self.get_parameter('force_filter_alpha').value), 0.0, 1.0)
        self.max_force_rate_x = max(
            0.0, float(self.get_parameter('max_force_rate_x').value))
        self.max_torque_rate_z = max(
            0.0, float(self.get_parameter('max_torque_rate_z').value))

        self.enable_button = int(self.get_parameter('enable_button').value)
        self.turbo_button = int(self.get_parameter('turbo_button').value)
        self.require_enable_button = bool(
            self.get_parameter('require_enable_button').value)

        self.mode_modifier_button = int(self.get_parameter('mode_modifier_button').value)
        self.alternate_mode_modifier_button = int(
            self.get_parameter('alternate_mode_modifier_button').value)
        self.body_force_button = int(self.get_parameter('body_force_button').value)
        self.body_velocity_button = int(self.get_parameter('body_velocity_button').value)
        self.body_position_button = int(self.get_parameter('body_position_button').value)
        self.stop_controllers_button = int(
            self.get_parameter('stop_controllers_button').value)

        self.controller_manager_name = str(
            self.get_parameter('controller_manager_name').value)
        self.body_force_controller_name = str(
            self.get_parameter('body_force_controller_name').value)
        self.body_velocity_controller_name = str(
            self.get_parameter('body_velocity_controller_name').value)
        self.body_position_controller_name = str(
            self.get_parameter('body_position_controller_name').value)
        self.thruster_test_controller_name = str(
            self.get_parameter('thruster_test_controller_name').value)
        self.navigator_topic = str(self.get_parameter('navigator_topic').value)
        self.position_setpoint_topic = str(
            self.get_parameter('position_setpoint_topic').value)

        self.axis_deadzone = float(self.get_parameter('axis_deadzone').value)
        self.invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self.invert_angular_z = bool(self.get_parameter('invert_angular_z').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.force_pub = self.create_publisher(Wrench, self.body_force_topic, 10)
        self.position_setpoint_pub = self.create_publisher(
            PoseStamped, self.position_setpoint_topic, 10)
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )
        self.navigator_sub = self.create_subscription(
            Navigator,
            self.navigator_topic,
            self.navigator_callback,
            10,
        )

        switch_service = f'{self.controller_manager_name}/switch_controller'
        self.switch_client = self.create_client(SwitchController, switch_service)
        list_service = f'{self.controller_manager_name}/list_controllers'
        self.list_controllers_client = self.create_client(ListControllers, list_service)

        self.last_joy_msg: Optional[Joy] = None
        self.last_joy_time = None
        self.last_published_twist = Twist()
        self.last_published_wrench = Wrench()
        self.filtered_force_x = 0.0
        self.filtered_torque_z = 0.0
        self.last_publish_time = self.get_clock().now()
        self.zero_twist_sent = False
        self.zero_wrench_sent = False
        self.pending_switch = False
        self.active_mode = 'none'
        self.prev_force_combo_pressed = False
        self.prev_velocity_combo_pressed = False
        self.prev_position_combo_pressed = False
        self.warned_waiting_for_joy = False
        self.pending_switch_steps = []
        self.last_navigator_msg: Optional[Navigator] = None

        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(timer_period, self.publish_command)
        self.controller_state_timer = self.create_timer(1.0, self.refresh_controller_state)

        self.get_logger().info('BlueBoat teleop ready')
        self.get_logger().info(f'Listening on joy topic: {self.joy_topic}')
        self.get_logger().info(f'Publishing cmd_vel on: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing body force on: {self.body_force_topic}')
        self.get_logger().info(
            f'Mode switching on {switch_service}: RB+LB toggles {self.body_force_controller_name}, '
            f'RB+X toggles {self.body_velocity_controller_name} through '
            f'{self.body_force_controller_name}, RB+A toggles '
            f'{self.body_position_controller_name} through '
            f'{self.body_velocity_controller_name}')

    def navigator_callback(self, msg: Navigator) -> None:
        self.last_navigator_msg = msg

    def joy_callback(self, msg: Joy) -> None:
        self.last_joy_msg = msg
        self.last_joy_time = self.get_clock().now()
        self.zero_twist_sent = False
        self.zero_wrench_sent = False
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
        position_combo_pressed = (
            modifier_pressed and
            self.button_pressed(buttons, self.body_position_button)
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
            elif self.active_mode == 'position':
                self.switch_mode(
                    start=[],
                    stop=[
                        self.body_position_controller_name,
                        self.body_velocity_controller_name,
                        self.body_force_controller_name,
                        self.thruster_test_controller_name,
                    ],
                    label='body position, velocity and force disabled',
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
                        'stop': [
                            self.body_position_controller_name,
                            self.thruster_test_controller_name,
                        ],
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
        elif position_combo_pressed and not self.prev_position_combo_pressed:
            if self.active_mode == 'position':
                self.switch_mode(
                    start=[],
                    stop=[self.body_position_controller_name],
                    label='body position disabled',
                    target_mode='velocity',
                )
            else:
                if not self.publish_hold_setpoint():
                    self.get_logger().warning(
                        'Cannot enable body position mode without navigator feedback yet')
                else:
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
                            'label': 'body velocity prerequisite',
                            'target_mode': None,
                        },
                        {
                            'start': [self.body_position_controller_name],
                            'stop': [],
                            'label': 'body position',
                            'target_mode': 'position',
                        },
                    ])

        self.prev_force_combo_pressed = force_combo_pressed
        self.prev_velocity_combo_pressed = velocity_combo_pressed
        self.prev_position_combo_pressed = position_combo_pressed

    def mode_modifier_pressed(self, buttons: List[int]) -> bool:
        return (
            self.button_pressed(buttons, self.mode_modifier_button) or
            self.button_pressed(buttons, self.alternate_mode_modifier_button)
        )

    def publish_hold_setpoint(self) -> bool:
        if self.last_navigator_msg is None:
            return False

        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        setpoint.pose = self.last_navigator_msg.position
        self.position_setpoint_pub.publish(setpoint)
        return True

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
        self.publish_stop_commands(immediate=True)

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

    def refresh_controller_state(self) -> None:
        if self.pending_switch:
            return
        if not self.list_controllers_client.wait_for_service(timeout_sec=0.0):
            return

        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._handle_controller_state)

    def _handle_controller_state(self, future) -> None:
        if self.pending_switch:
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS future failure path
            self.get_logger().debug(f'Failed to read controller state: {exc}')
            return

        controllers = {controller.name: controller for controller in response.controller}
        active = {
            name
            for name, controller in controllers.items()
            if controller.state == 'active'
        }
        if self.body_position_controller_name in active:
            detected_mode = 'position'
        elif self.body_velocity_controller_name in active:
            detected_mode = 'velocity'
        elif (
            self.body_force_controller_name in active and
            not controllers[self.body_force_controller_name].is_chained
        ):
            detected_mode = 'force'
        else:
            detected_mode = 'none'

        if detected_mode != self.active_mode:
            self.active_mode = detected_mode
            self.reset_filtered_force()
            self.get_logger().info(f'Detected controller mode: {self.active_mode}')

    def publish_command(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_publish_time).nanoseconds * 1.0e-9)
        self.last_publish_time = now

        if self.last_joy_msg is None or self.last_joy_time is None:
            if not self.warned_waiting_for_joy:
                self.get_logger().warning(
                    f'No Joy messages received yet on {self.joy_topic}. '
                    'Check that joy_node is running and the gamepad is visible in /dev/input.')
                self.warned_waiting_for_joy = True
            self.publish_stop_commands(dt)
            return

        age = self.get_clock().now() - self.last_joy_time
        if age > Duration(seconds=self.command_timeout):
            self.publish_stop_commands(dt)
            return

        if self.pending_switch:
            self.publish_stop_commands(dt)
            return

        if not self.teleop_enabled(self.last_joy_msg.buttons):
            self.publish_stop_commands(dt)
            return

        linear_x = self.axis_value(self.last_joy_msg.axes, self.axis_linear_x)
        angular_z = self.axis_value(self.last_joy_msg.axes, self.axis_angular_z)

        if self.invert_linear_x:
            linear_x *= -1.0
        if self.invert_angular_z:
            angular_z *= -1.0

        if self.active_mode == 'force':
            self.publish_force_command(linear_x, angular_z, dt)
        elif self.active_mode == 'velocity':
            self.publish_velocity_command(linear_x, angular_z)
            self.publish_wrench_if_changed(Wrench())
        else:
            self.publish_stop_commands(dt)

    def publish_velocity_command(self, linear_x: float, angular_z: float) -> None:
        linear_scale = self.scale_linear_x
        angular_scale = self.scale_angular_z

        if self.button_pressed(self.last_joy_msg.buttons, self.turbo_button):
            linear_scale = self.turbo_scale_linear_x
            angular_scale = self.turbo_scale_angular_z

        twist = Twist()
        twist.linear.x = linear_x * linear_scale
        twist.angular.z = angular_z * angular_scale

        self.publish_twist_if_changed(twist)

    def publish_force_command(self, linear_x: float, angular_z: float, dt: float) -> None:
        force_scale = self.scale_force_x
        torque_scale = self.scale_torque_z

        if self.button_pressed(self.last_joy_msg.buttons, self.turbo_button):
            force_scale = self.turbo_scale_force_x
            torque_scale = self.turbo_scale_torque_z

        target_force_x = linear_x * force_scale
        target_torque_z = angular_z * torque_scale

        self.filtered_force_x = self.filtered_value(
            current=self.filtered_force_x,
            target=target_force_x,
            alpha=self.force_filter_alpha,
            max_rate=self.max_force_rate_x,
            dt=dt,
        )
        self.filtered_torque_z = self.filtered_value(
            current=self.filtered_torque_z,
            target=target_torque_z,
            alpha=self.force_filter_alpha,
            max_rate=self.max_torque_rate_z,
            dt=dt,
        )

        wrench = Wrench()
        wrench.force.x = self.filtered_force_x
        wrench.torque.z = self.filtered_torque_z

        self.publish_twist_if_changed(Twist())
        self.publish_wrench_if_changed(wrench)

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

    def publish_stop_commands(self, dt: float = 0.0, immediate: bool = False) -> None:
        if immediate:
            self.filtered_force_x = 0.0
            self.filtered_torque_z = 0.0
        else:
            self.filtered_force_x = self.filtered_value(
                current=self.filtered_force_x,
                target=0.0,
                alpha=self.force_filter_alpha,
                max_rate=self.max_force_rate_x,
                dt=dt,
            )
            self.filtered_torque_z = self.filtered_value(
                current=self.filtered_torque_z,
                target=0.0,
                alpha=self.force_filter_alpha,
                max_rate=self.max_torque_rate_z,
                dt=dt,
            )

        wrench = Wrench()
        wrench.force.x = self.filtered_force_x
        wrench.torque.z = self.filtered_torque_z

        self.publish_twist_if_changed(Twist())
        self.publish_wrench_if_changed(wrench)

    def reset_filtered_force(self) -> None:
        self.filtered_force_x = 0.0
        self.filtered_torque_z = 0.0
        self.last_published_wrench = Wrench()
        self.zero_wrench_sent = False

    def publish_twist_if_changed(self, twist: Twist) -> None:
        is_zero = self.is_zero_twist(twist)
        if (
            self.twist_equal(self.last_published_twist, twist) and
            is_zero and
            self.zero_twist_sent
        ):
            return

        self.cmd_pub.publish(twist)
        self.last_published_twist = twist
        self.zero_twist_sent = is_zero

    def publish_wrench_if_changed(self, wrench: Wrench) -> None:
        is_zero = self.is_zero_wrench(wrench)
        if (
            self.wrench_equal(self.last_published_wrench, wrench) and
            is_zero and
            self.zero_wrench_sent
        ):
            return

        self.force_pub.publish(wrench)
        self.last_published_wrench = wrench
        self.zero_wrench_sent = is_zero

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

    @staticmethod
    def wrench_equal(lhs: Wrench, rhs: Wrench, eps: float = 1e-9) -> bool:
        return (
            abs(lhs.force.x - rhs.force.x) < eps and
            abs(lhs.force.y - rhs.force.y) < eps and
            abs(lhs.force.z - rhs.force.z) < eps and
            abs(lhs.torque.x - rhs.torque.x) < eps and
            abs(lhs.torque.y - rhs.torque.y) < eps and
            abs(lhs.torque.z - rhs.torque.z) < eps
        )

    @staticmethod
    def is_zero_wrench(wrench: Wrench, eps: float = 1e-9) -> bool:
        return (
            abs(wrench.force.x) < eps and
            abs(wrench.force.y) < eps and
            abs(wrench.force.z) < eps and
            abs(wrench.torque.x) < eps and
            abs(wrench.torque.y) < eps and
            abs(wrench.torque.z) < eps
        )

    @staticmethod
    def filtered_value(
        current: float,
        target: float,
        alpha: float,
        max_rate: float,
        dt: float,
    ) -> float:
        filtered = current + alpha * (target - current)
        if max_rate > 0.0 and dt > 0.0:
            max_delta = max_rate * dt
            filtered = current + BlueBoatTeleop.clamp(
                filtered - current,
                -max_delta,
                max_delta,
            )
        return filtered

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


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
