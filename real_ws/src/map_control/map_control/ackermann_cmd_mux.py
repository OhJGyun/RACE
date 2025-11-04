#!/usr/bin/env python3
"""
Ackermann command multiplexer with 3-level priority system.

Priority levels (highest to lowest):
1. Joy teleop - Manual control, always takes priority (safety override)
2. Recovery - Collision recovery commands
3. Primary - Autonomous map controller

Joy teleop is considered active if commands were received within joy_timeout.
Recovery commands take priority over primary but not over joy teleop.
"""

import copy
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannCommandMux(Node):
    """Multiplex Ackermann commands with 3-level priority: joy > recovery > primary."""

    def __init__(self) -> None:
        super().__init__('ackermann_cmd_mux')

        # Parameters controlling topics and timeouts
        self.declare_parameter('joy_topic', '/joy_teleop/ackermann_cmd')
        self.declare_parameter('recovery_topic', '/recovery/ackermann_cmd')
        self.declare_parameter('primary_topic', '/map_control/ackermann_cmd')
        self.declare_parameter('output_topic', '/drive')
        self.declare_parameter('joy_timeout', 0.5)              # Joy considered active if cmd within 0.5s
        self.declare_parameter('recovery_hold_time', 0.3)       # Recovery hold duration

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.recovery_topic = str(self.get_parameter('recovery_topic').value)
        self.primary_topic = str(self.get_parameter('primary_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.joy_timeout_duration = float(self.get_parameter('joy_timeout').value)
        self.recovery_hold_time = float(self.get_parameter('recovery_hold_time').value)

        self.joy_timeout = Duration(seconds=self.joy_timeout_duration)
        self.recovery_timeout = Duration(seconds=self.recovery_hold_time)

        # State tracking
        self.last_joy_time: Optional[Time] = None
        self.last_joy_cmd: Optional[AckermannDriveStamped] = None

        self.recovery_active = False
        self.recovery_start_time: Optional[Time] = None
        self.recovery_cmd_template: Optional[AckermannDriveStamped] = None

        # Publisher for the final command
        self.output_pub = self.create_publisher(
            AckermannDriveStamped,
            self.output_topic,
            10,
        )

        # Subscriptions for all three priority levels
        self.joy_sub = self.create_subscription(
            AckermannDriveStamped,
            self.joy_topic,
            self._joy_callback,
            10,
        )

        self.recovery_sub = self.create_subscription(
            AckermannDriveStamped,
            self.recovery_topic,
            self._recovery_callback,
            10,
        )

        self.primary_sub = self.create_subscription(
            AckermannDriveStamped,
            self.primary_topic,
            self._primary_callback,
            10,
        )

        # Timer for recovery timeout management
        self.recovery_timer = self.create_timer(0.05, self._check_recovery_timeout)

        self.get_logger().info(
            f'[MUX] 3-level priority active:\n'
            f'  1. Joy:     {self.joy_topic} (timeout: {self.joy_timeout_duration}s)\n'
            f'  2. Recovery: {self.recovery_topic}\n'
            f'  3. Primary:  {self.primary_topic}\n'
            f'  Output:      {self.output_topic}'
        )

    def _is_joy_active(self) -> bool:
        """Check if joy teleop is currently active (received command within timeout)."""
        if self.last_joy_time is None:
            return False

        now = self.get_clock().now()
        elapsed = now - self.last_joy_time
        return elapsed < self.joy_timeout

    def _joy_callback(self, msg: AckermannDriveStamped) -> None:
        """Joy teleop has highest priority - always forward immediately."""
        self.last_joy_time = self.get_clock().now()
        self.last_joy_cmd = copy.deepcopy(msg)

        # Joy overrides everything
        self.output_pub.publish(msg)

        # Log when joy takes control
        if self.recovery_active:
            self.get_logger().info('[MUX] JOY OVERRIDE - Joy teleop active, ignoring recovery')

    def _recovery_callback(self, msg: AckermannDriveStamped) -> None:
        """Recovery has 2nd priority - active only if joy is inactive."""
        # Joy teleop overrides recovery
        if self._is_joy_active():
            self.get_logger().debug('[MUX] Recovery command ignored; joy teleop active.')
            return

        # Don't accept new recovery commands while one is already active
        if self.recovery_active:
            self.get_logger().debug('[MUX] Recovery command ignored; hold timer active.')
            return

        now = self.get_clock().now()
        self.recovery_active = True
        self.recovery_start_time = now
        self.recovery_cmd_template = copy.deepcopy(msg)

        self.get_logger().warn('[MUX] RECOVERY ACTIVE - Overriding primary control')
        self._publish_recovery_command(now)

    def _primary_callback(self, msg: AckermannDriveStamped) -> None:
        """Primary (map control) has lowest priority - only active if joy and recovery inactive."""
        # Joy teleop overrides primary
        if self._is_joy_active():
            return

        # Recovery overrides primary
        if self.recovery_active:
            return

        # Primary control active
        self.output_pub.publish(msg)

    def _check_recovery_timeout(self) -> None:
        """Manage recovery timeout and continue publishing recovery commands."""
        if not self.recovery_active or self.recovery_start_time is None:
            return

        now = self.get_clock().now()

        # Joy teleop can interrupt recovery
        if self._is_joy_active():
            self.recovery_active = False
            self.recovery_start_time = None
            self.recovery_cmd_template = None
            self.get_logger().info('[MUX] Recovery interrupted by joy teleop')
            return

        elapsed = now - self.recovery_start_time
        if elapsed < self.recovery_timeout:
            # Continue publishing recovery command
            self._publish_recovery_command(now)
            return

        # Recovery timeout expired
        self.recovery_active = False
        self.recovery_start_time = None
        self.recovery_cmd_template = None
        self.get_logger().info('[MUX] Recovery completed. Returning to primary control.')

    def _publish_recovery_command(self, stamp: Time) -> None:
        """Publish the stored recovery command with updated timestamp."""
        if self.recovery_cmd_template is None:
            return

        cmd = copy.deepcopy(self.recovery_cmd_template)
        cmd.header.stamp = stamp.to_msg()
        self.output_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AckermannCommandMux()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down ackermann_cmd_mux...')
    except Exception as e:
        if node:
            node.get_logger().error(f'Exception in ackermann_cmd_mux: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
