import math
from typing import Dict, Iterable

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StatusLightFeedback(Node):
    def __init__(self) -> None:
        super().__init__("status_light_feedback")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("commands_topic", "/status_light_controller/commands")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("startup_duration", 3.0)
        self.declare_parameter("status_timeout", 3.0)
        self.declare_parameter(
            "relevant_prefixes",
            ["/Sensors", "/Hardware", "/Navigation", "/Controllers"],
        )
        self.declare_parameter("waiting_blink_period", 0.5)
        self.declare_parameter("warn_blink_period", 1.0)
        self.declare_parameter("error_blink_period", 0.25)

        diagnostics_topic = self.get_parameter("diagnostics_topic").value
        commands_topic = self.get_parameter("commands_topic").value
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._startup_duration = Duration(
            seconds=float(self.get_parameter("startup_duration").value)
        )
        self._status_timeout = Duration(
            seconds=float(self.get_parameter("status_timeout").value)
        )
        self._relevant_prefixes = tuple(self.get_parameter("relevant_prefixes").value)
        self._waiting_blink_period = float(
            self.get_parameter("waiting_blink_period").value
        )
        self._warn_blink_period = float(self.get_parameter("warn_blink_period").value)
        self._error_blink_period = float(self.get_parameter("error_blink_period").value)

        self._status_levels: Dict[str, int] = {}
        self._status_times: Dict[str, rclpy.time.Time] = {}
        self._state_label = "BOOT"
        self._last_output = None
        self._boot_started = self.get_clock().now()

        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        self._commands_pub = self.create_publisher(Float64MultiArray, commands_topic, 10)
        self._timer = self.create_timer(
            1.0 / max(self._publish_rate_hz, 1e-3),
            self._publish_light_command,
        )

        self.get_logger().info(
            "Status light feedback started. "
            f"diagnostics_topic={diagnostics_topic} commands_topic={commands_topic}"
        )

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        now = self.get_clock().now()
        for status in msg.status:
            if self._is_relevant(status.name):
                self._status_levels[status.name] = int(status.level)
                self._status_times[status.name] = now

    def _is_relevant(self, name: str) -> bool:
        if not self._relevant_prefixes:
            return True
        return any(name.startswith(prefix) for prefix in self._relevant_prefixes)

    def _fresh_levels(self) -> Iterable[int]:
        now = self.get_clock().now()
        for name, level in self._status_levels.items():
            last_seen = self._status_times.get(name)
            if last_seen is None:
                continue
            if now - last_seen <= self._status_timeout:
                yield level

    def _compute_state(self) -> str:
        now = self.get_clock().now()
        fresh_levels = list(self._fresh_levels())

        if now - self._boot_started < self._startup_duration and not fresh_levels:
            return "BOOT"
        if not fresh_levels:
            return "WAITING"

        worst_level = max(fresh_levels)
        if worst_level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE):
            return "ERROR"
        if worst_level == DiagnosticStatus.WARN:
            return "WARN"
        return "OK"

    def _blink_output(self, period: float) -> bool:
        if period <= 0.0:
            return True
        phase = self.get_clock().now().nanoseconds / 1e9
        return math.fmod(phase, period) < (period / 2.0)

    def _publish_light_command(self) -> None:
        state = self._compute_state()
        if state != self._state_label:
            self.get_logger().info("Status light state -> %s", state)
            self._state_label = state

        if state == "OK":
            enabled = True
        elif state == "WARN":
            enabled = self._blink_output(self._warn_blink_period)
        elif state == "ERROR":
            enabled = self._blink_output(self._error_blink_period)
        else:
            enabled = self._blink_output(self._waiting_blink_period)

        if self._last_output is enabled:
            return

        msg = Float64MultiArray()
        msg.data = [1.0 if enabled else 0.0]
        self._commands_pub.publish(msg)
        self._last_output = enabled


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusLightFeedback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
