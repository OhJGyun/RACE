#!/usr/bin/env python3
"""
Ackermann command multiplexer.

This node arbitrates between the main map controller commands and recovery
commands. Recovery commands always take priority while they are active, and
normal control resumes automatically once recovery commands stop arriving.

Once a recovery command is accepted, new recovery messages are ignored
until the recovery hold timer expires so the vehicle keeps executing the
initial reverse command for a deterministic duration.
"""

import copy
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannCommandMux(Node):
    """Multiplex multiple Ackermann command sources with recovery priority."""

    def __init__(self) -> None:
        super().__init__('ackermann_cmd_mux')

        # Parameters controlling topics and recovery timeout
        self.declare_parameter('primary_topic', '/map_control/ackermann_cmd')
        self.declare_parameter('recovery_topic', '/recovery/ackermann_cmd')
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('recovery_hold_time', 0.3)

        self.primary_topic = str(self.get_parameter('primary_topic').value)
        self.recovery_topic = str(self.get_parameter('recovery_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.recovery_hold_time = float(self.get_parameter('recovery_hold_time').value)

        self.recovery_timeout = Duration(seconds=self.recovery_hold_time)
        self.recovery_active = False
        self.recovery_start_time: Optional[Time] = None
        self.recovery_cmd_template: Optional[AckermannDriveStamped] = None

        # Publisher for the final command
        self.output_pub = self.create_publisher(
            AckermannDriveStamped,
            self.output_topic,
            10,
        )

        # Subscriptions for primary and recovery commands
        self.primary_sub = self.create_subscription(
            AckermannDriveStamped,
            self.primary_topic,
            self._primary_callback,
            10,
        )

        self.recovery_sub = self.create_subscription(
            AckermannDriveStamped,
            self.recovery_topic,
            self._recovery_callback,
            10,
        )

        # Timer used to release recovery priority once commands stop arriving
        self.recovery_timer = self.create_timer(0.05, self._check_recovery_timeout)

        self.get_logger().info(
            f'Ackermann command mux active. primary={self.primary_topic}, '
            f'recovery={self.recovery_topic}, output={self.output_topic}'
        )

    def _primary_callback(self, msg: AckermannDriveStamped) -> None:
        """Forward primary commands only when recovery is inactive."""
        if self.recovery_active:
            return
        self.output_pub.publish(msg)

    def _recovery_callback(self, msg: AckermannDriveStamped) -> None:
        """Accept the first recovery command and ignore subsequent ones until release."""
        if self.recovery_active:
            self.get_logger().debug('[MUX] Recovery command ignored; hold timer active.')
            return

        now = self.get_clock().now()
        self.recovery_active = True
        self.recovery_start_time = now
        self.recovery_cmd_template = copy.deepcopy(msg)

        self.get_logger().warn('[MUX] Recovery command active. Overriding primary control.')
        self._publish_recovery_command(now)

    def _check_recovery_timeout(self) -> None:
        """Release recovery priority once commands stop arriving."""
        if not self.recovery_active or self.recovery_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = now - self.recovery_start_time
        if elapsed < self.recovery_timeout:
            self._publish_recovery_command(now)
            return

        self.recovery_active = False
        self.recovery_start_time = None
        self.recovery_cmd_template = None
        self.get_logger().info('[MUX] Recovery command released. Returning to primary control.')

    def _publish_recovery_command(self, stamp: Time) -> None:
        """Publish the stored recovery command with the provided stamp."""
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
