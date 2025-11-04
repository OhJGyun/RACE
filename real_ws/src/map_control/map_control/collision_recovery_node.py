#!/usr/bin/env python3
"""
Collision recovery node for map_control package (IMU linear_x acceleration-based).

This node monitors IMU linear_x acceleration to detect collision/stuck states.
When the vehicle's commanded speed exists but IMU linear_x acceleration stays
within a specific range (e.g., [-1.0, 0.0]) for a specified duration,
it assumes a collision and publishes reverse Ackermann commands to recover.

Key: Uses raw IMU linear_acceleration.x, not integrated velocity.
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
    """ROS 2 node that detects collisions via IMU linear_x acceleration and issues reverse commands."""

    def __init__(self) -> None:
        super().__init__('collision_recovery_node')

        # Declare configurable parameters
        self.declare_parameter('enable_recovery', True)           # Enable/disable collision recovery
        self.declare_parameter('imu_accel_min', -1.0)             # Min linear_x accel for collision [m/s²]
        self.declare_parameter('imu_accel_max', 0.0)              # Max linear_x accel for collision [m/s²]
        self.declare_parameter('collision_time_threshold', 0.5)   # Time to confirm collision [s]
        self.declare_parameter('commanded_speed_threshold', 0.3)  # Min commanded speed to monitor [m/s]
        self.declare_parameter('reverse_speed', -1.5)             # Reverse speed [m/s]
        self.declare_parameter('reverse_duration', 1.0)           # Reverse duration [s]
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('command_topic', '/recovery/ackermann_cmd')
        self.declare_parameter('drive_cmd_topic', '/map_control/ackermann_cmd')  # Monitor commanded speed
        self.declare_parameter('base_frame', 'base_link')

        # Load parameter values
        self.enable_recovery = bool(self.get_parameter('enable_recovery').value)
        self.imu_accel_min = float(
            self.get_parameter('imu_accel_min').value
        )
        self.imu_accel_max = float(
            self.get_parameter('imu_accel_max').value
        )
        self.collision_time_threshold = float(
            self.get_parameter('collision_time_threshold').value
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

        # IMU acceleration history buffer (time, linear_x_accel)
        self.accel_history: Deque[Tuple[Time, float]] = deque()

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
                f'for collision detection (accel in range [{self.imu_accel_min}, {self.imu_accel_max}] m/s²)'
            )
        else:
            self.get_logger().warn(
                'collision_recovery_node initialized but DISABLED (enable_recovery=False)'
            )

    def imu_callback(self, msg: Imu) -> None:
        """IMU callback - store linear_x acceleration history."""
        now = self.get_clock().now()
        accel_x = msg.linear_acceleration.x

        # Store acceleration in history
        self.accel_history.append((now, accel_x))

        # Keep only recent history (within collision_time_threshold + 0.5s buffer)
        max_age_ns = int((self.collision_time_threshold + 0.5) * 1e9)
        while self.accel_history:
            oldest_time = self.accel_history[0][0]
            if (now.nanoseconds - oldest_time.nanoseconds) <= max_age_ns:
                break
            self.accel_history.popleft()

    def drive_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        """Track commanded driving speed."""
        self.commanded_speed = msg.drive.speed
        self.last_cmd_time = self.get_clock().now()

    def _timer_callback(self) -> None:
        """Timer callback executed at 20 Hz for recovery management."""
        # Skip if recovery is disabled
        if not self.enable_recovery:
            return

        now = self.get_clock().now()

        if self.is_recovering:
            self._handle_recovery(now)
            return

        # Check for collision condition
        if self._is_collision_detected(now):
            if self.collision_detected_time is None:
                # First detection
                self.collision_detected_time = now
                recent_accel = self.accel_history[-1][1] if self.accel_history else 0.0
                self.get_logger().warn(
                    f'[COLLISION] Possible collision detected (commanded: {self.commanded_speed:.2f} m/s, '
                    f'IMU accel_x: {recent_accel:.2f} m/s²). '
                    f'Waiting {self.collision_time_threshold}s to confirm...'
                )
            else:
                # Check if condition persisted long enough
                elapsed = (now.nanoseconds - self.collision_detected_time.nanoseconds) / 1e9
                if elapsed >= self.collision_time_threshold:
                    self._start_recovery(now)
        else:
            # Reset detection if condition no longer met
            if self.collision_detected_time is not None:
                self.get_logger().info('[COLLISION] Condition cleared before threshold. Resetting.')
                self.collision_detected_time = None

    def _is_collision_detected(self, now: Time) -> bool:
        """Check if vehicle is commanded to move but IMU accel_x shows collision state."""
        # Need recent command data
        if self.last_cmd_time is None:
            return False

        cmd_age = (now.nanoseconds - self.last_cmd_time.nanoseconds) / 1e9
        if cmd_age > 0.5:  # Command too old
            return False

        # Check if we're being commanded to move forward
        if abs(self.commanded_speed) < self.commanded_speed_threshold:
            return False

        # Check recent IMU acceleration readings
        if not self.accel_history:
            return False

        threshold_ns = int(self.collision_time_threshold * 1e9)
        recent_accels = [
            accel for timestamp, accel in self.accel_history
            if (now.nanoseconds - timestamp.nanoseconds) <= threshold_ns
        ]

        if not recent_accels:
            return False

        # All recent accelerations must be within collision range
        return all(
            self.imu_accel_min <= accel <= self.imu_accel_max
            for accel in recent_accels
        )

    def _start_recovery(self, now: Time) -> None:
        """Begin the recovery maneuver by publishing reverse commands."""
        self.is_recovering = True
        self.recovery_start_time = now
        self.collision_detected_time = None
        self.accel_history.clear()
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
