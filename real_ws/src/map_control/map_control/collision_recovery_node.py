#!/usr/bin/env python3
"""
Collision recovery node for map_control package (IMU jerk-based).

This node monitors IMU linear_x acceleration jerk (rate of change) to detect collisions.
When the vehicle experiences a sudden negative acceleration spike (large jerk),
it assumes a collision and publishes reverse Ackermann commands to recover.

Key: Uses acceleration change rate (jerk) for fast collision detection.
"""

from collections import deque
import time
from typing import Deque, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu


class CollisionRecoveryNode(Node):
    """ROS 2 node that detects collisions via IMU jerk (acceleration change rate) and issues reverse commands."""

    def __init__(self) -> None:
        super().__init__('collision_recovery_node')

        # Declare configurable parameters
        self.declare_parameter('enable_recovery', True)           # Enable/disable collision recovery
        self.declare_parameter('jerk_threshold', 8.0)             # Jerk threshold for collision detection [m/s³]
        self.declare_parameter('accel_threshold', -3.0)           # Minimum negative accel to consider [m/s²]
        self.declare_parameter('collision_confirm_time', 0.1)     # Time to confirm collision [s]
        self.declare_parameter('commanded_speed_threshold', 0.3)  # Min commanded speed to monitor [m/s]
        self.declare_parameter('reverse_speed', -1.5)             # Reverse speed [m/s]
        self.declare_parameter('reverse_duration', 1.0)           # Reverse duration [s]
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('command_topic', '/recovery/ackermann_cmd')
        self.declare_parameter('drive_cmd_topic', '/map_control/ackermann_cmd')  # Monitor commanded speed
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('log_jerk_threshold', 5.0)         # Log jerks above this value [m/s³]

        # Load parameter values
        self.enable_recovery = bool(self.get_parameter('enable_recovery').value)
        self.jerk_threshold = float(
            self.get_parameter('jerk_threshold').value
        )
        self.accel_threshold = float(
            self.get_parameter('accel_threshold').value
        )
        self.collision_confirm_time = float(
            self.get_parameter('collision_confirm_time').value
        )
        self.commanded_speed_threshold = float(
            self.get_parameter('commanded_speed_threshold').value
        )
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_duration = float(self.get_parameter('reverse_duration').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.drive_cmd_topic = str(self.get_parameter('drive_cmd_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.log_jerk_threshold = float(self.get_parameter('log_jerk_threshold').value)

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        # Drive command subscriber (to know if we're trying to move)
        self.drive_cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            self.drive_cmd_topic,
            self.drive_cmd_callback,
            10
        )

        # Publisher for recovery Ackermann commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.command_topic,
            10,
        )

        # IMU acceleration history for jerk calculation
        self.prev_accel: Optional[float] = None
        self.prev_accel_time: Optional[Time] = None

        # Commanded speed tracking
        self.commanded_speed: float = 0.0
        self.last_cmd_time: Optional[Time] = None

        # Recovery state
        self.is_recovering = False
        self.recovery_start_time: Optional[Time] = None
        self.collision_detected_time: Optional[Time] = None

        # Timer running at 20 Hz for recovery management
        self.timer = self.create_timer(0.05, self._timer_callback)

        if self.enable_recovery:
            self.get_logger().info(
                f'collision_recovery_node initialized: monitoring {self.imu_topic} '
                f'for collision detection (jerk threshold: {self.jerk_threshold} m/s³, '
                f'accel threshold: {self.accel_threshold} m/s²)'
            )
        else:
            self.get_logger().warn(
                'collision_recovery_node initialized but DISABLED (enable_recovery=False)'
            )

    def imu_callback(self, msg: Imu) -> None:
        """IMU callback - calculate jerk and detect collision impulse."""
        now = self.get_clock().now()
        accel_x = msg.linear_acceleration.x

        # Calculate jerk if we have previous acceleration data
        if self.prev_accel is not None and self.prev_accel_time is not None:
            dt = (now.nanoseconds - self.prev_accel_time.nanoseconds) / 1e9

            if dt > 0 and dt < 0.5:  # Valid time difference
                jerk = abs(accel_x - self.prev_accel) / dt
                accel_change = accel_x - self.prev_accel

                # Log significant jerks for parameter tuning
                if jerk > self.log_jerk_threshold:
                    self.get_logger().info(
                        f'[JERK] jerk={jerk:.2f} m/s³, '
                        f'accel_x={accel_x:.2f} m/s², '
                        f'prev_accel={self.prev_accel:.2f} m/s², '
                        f'Δaccel={accel_change:.2f} m/s², '
                        f'dt={dt:.4f}s, '
                        f'cmd_speed={self.commanded_speed:.2f} m/s'
                    )

                # Skip collision detection if recovery is disabled or already recovering
                if not self.enable_recovery or self.is_recovering:
                    self.prev_accel = accel_x
                    self.prev_accel_time = now
                    return

                # Check for collision: large negative jerk (sudden deceleration)
                if (jerk > self.jerk_threshold and
                    accel_change < 0 and
                    accel_x < self.accel_threshold):

                    self._handle_collision_impulse(now, accel_x, jerk, accel_change)

        # Update previous acceleration
        self.prev_accel = accel_x
        self.prev_accel_time = now

    def drive_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        """Track commanded driving speed."""
        self.commanded_speed = msg.drive.speed
        self.last_cmd_time = self.get_clock().now()

    def _handle_collision_impulse(self, now: Time, accel_x: float, jerk: float, accel_change: float) -> None:
        """Handle detected collision impulse."""
        # Check if we're being commanded to move
        if self.last_cmd_time is None:
            return

        cmd_age = (now.nanoseconds - self.last_cmd_time.nanoseconds) / 1e9
        if cmd_age > 0.5:  # Command too old
            return

        if abs(self.commanded_speed) < self.commanded_speed_threshold:
            return

        # Collision impulse detected while commanded to move
        if self.collision_detected_time is None:
            self.collision_detected_time = now
            self.get_logger().warn(
                f'[COLLISION] Impulse detected! '
                f'accel_x={accel_x:.2f} m/s², jerk={jerk:.2f} m/s³, '
                f'Δaccel={accel_change:.2f} m/s², commanded={self.commanded_speed:.2f} m/s. '
                f'Confirming for {self.collision_confirm_time}s...'
            )

    def _timer_callback(self) -> None:
        """Timer callback executed at 20 Hz for recovery management."""
        # Skip if recovery is disabled
        if not self.enable_recovery:
            return

        now = self.get_clock().now()

        if self.is_recovering:
            self._handle_recovery(now)
            return

        # Check if collision was detected and confirmation time has passed
        if self.collision_detected_time is not None:
            elapsed = (now.nanoseconds - self.collision_detected_time.nanoseconds) / 1e9
            if elapsed >= self.collision_confirm_time:
                self._start_recovery(now)

    def _start_recovery(self, now: Time) -> None:
        """Begin the recovery maneuver by publishing reverse commands."""
        self.is_recovering = True
        self.recovery_start_time = now
        self.collision_detected_time = None
        # Reset acceleration tracking during recovery
        self.prev_accel = None
        self.prev_accel_time = None
        self.get_logger().warn(
            f'[RECOVERY] Collision confirmed! Initiating reverse at {self.reverse_speed} m/s '
            f'for {self.reverse_duration}s'
        )
        self._publish_drive_command(self.reverse_speed)

    def _handle_recovery(self, now: Time) -> None:
        """Continue or finish the recovery maneuver depending on elapsed time."""
        if self.recovery_start_time is None:
            self.get_logger().error('Recovery state inconsistency detected; resetting state.')
            self._finish_recovery()
            return

        elapsed = (now.nanoseconds - self.recovery_start_time.nanoseconds) / 1e9
        if elapsed < self.reverse_duration:
            # Keep publishing reverse command
            self._publish_drive_command(self.reverse_speed)
        else:
            self._finish_recovery()

    def _finish_recovery(self) -> None:
        """Stop the vehicle and exit recovery mode."""
        self._publish_drive_command(0.0)
        self.is_recovering = False
        self.recovery_start_time = None
        self.get_logger().info('[RECOVERY] Reverse maneuver completed. Resuming normal control.')

    def _publish_drive_command(self, speed: float) -> None:
        """Publish an Ackermann drive command at the given speed and zero steering."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.drive.speed = speed
        cmd.drive.steering_angle = 0.0
        self.drive_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CollisionRecoveryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down collision_recovery_node...')
    except Exception as e:
        if node:
            node.get_logger().error(f'Exception in collision_recovery_node: {e}')
        else:
            print(f'Exception during initialization: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
