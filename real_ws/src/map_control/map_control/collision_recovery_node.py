#!/usr/bin/env python3
"""
Collision recovery node for map_control package.

This node monitors the transform between the map and base_link frames to
detect when the vehicle is stuck (position change below a threshold over a
time window). When a stuck condition is detected, it publishes a reverse
Ackermann command for a configurable duration to help the vehicle break free.
"""

from collections import deque
import math
import time
from typing import Deque, Tuple, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
    TransformException,
)


class CollisionRecoveryNode(Node):
    """ROS 2 node that detects collisions/stuck states and issues reverse commands."""

    def __init__(self) -> None:
        super().__init__('collision_recovery_node')

        # Declare configurable parameters
        self.declare_parameter('stuck_distance_threshold', 0.5)
        self.declare_parameter('stuck_time_threshold', 2.0)
        self.declare_parameter('reverse_speed', -0.5)
        self.declare_parameter('reverse_duration', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('command_topic', '/recovery/ackermann_cmd')

        # Load parameter values
        self.stuck_distance_threshold = float(
            self.get_parameter('stuck_distance_threshold').value
        )
        self.stuck_time_threshold = float(
            self.get_parameter('stuck_time_threshold').value
        )
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_duration = float(self.get_parameter('reverse_duration').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.command_topic = str(self.get_parameter('command_topic').value)

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Publisher for Ackermann commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.command_topic,
            10,
        )

        # Position history buffer (time, x, y)
        self.position_history: Deque[Tuple[Time, float, float]] = deque()

        # Recovery state
        self.is_recovering = False
        self.recovery_start_time: Optional[Time] = None

        # Wait for TF to become available
        self.tf_ready = False
        self._wait_for_tf()

        # Timer running at 10 Hz
        self.timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            'collision_recovery_node initialized: monitoring TF for stuck detection.'
        )

    def _wait_for_tf(self) -> None:
        """Wait for TF to become available before starting."""
        self.get_logger().info('Waiting for TF (map -> base_link) to become available...')
        retry_count = 0
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
                if transform is not None:
                    self.tf_ready = True
                    self.get_logger().info('TF is now available. Starting collision recovery monitoring.')
                    return
            except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
                retry_count += 1
                if retry_count % 10 == 0:  # Log every 10 seconds
                    self.get_logger().info(f'Still waiting for TF... ({retry_count}s elapsed)')
                time.sleep(1.0)

    def _timer_callback(self) -> None:
        """Timer callback executed at 10 Hz."""
        if not self.tf_ready:
            return

        now = self.get_clock().now()

        transform = self._lookup_transform()
        if transform is None:
            return

        current_position = self._extract_xy(transform)
        self._update_position_history(now, current_position)

        if self.is_recovering:
            self._handle_recovery(now)
            return

        if self._is_stuck(now):
            self._start_recovery(now)

    def _lookup_transform(self) -> Optional[TransformStamped]:
        """Attempt to retrieve the latest transform, handling TF exceptions."""
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
        except TransformException as exc:
            self.get_logger().warn(f'Unexpected TF exception: {exc}')
        return None

    @staticmethod
    def _extract_xy(transform: TransformStamped) -> Tuple[float, float]:
        """Extract the x, y position from a TransformStamped."""
        translation = transform.transform.translation
        return translation.x, translation.y

    def _update_position_history(self, now: Time, position: Tuple[float, float]) -> None:
        """Maintain a sliding window of positions within the stuck time threshold."""
        self.position_history.append((now, position[0], position[1]))

        threshold_duration = Duration(seconds=self.stuck_time_threshold)
        while self.position_history:
            oldest_time = self.position_history[0][0]
            if now - oldest_time <= threshold_duration:
                break
            self.position_history.popleft()

    def _is_stuck(self, now: Time) -> bool:
        """Determine if the vehicle has moved less than the threshold within the time window."""
        if not self.position_history:
            return False

        oldest_time, oldest_x, oldest_y = self.position_history[0]

        time_window = (now - oldest_time).nanoseconds / 1e9
        if time_window < self.stuck_time_threshold:
            return False

        _, current_x, current_y = self.position_history[-1]
        distance = math.hypot(current_x - oldest_x, current_y - oldest_y)

        return distance <= self.stuck_distance_threshold

    def _start_recovery(self, now: Time) -> None:
        """Begin the recovery maneuver by publishing reverse commands."""
        self.is_recovering = True
        self.recovery_start_time = now
        self.position_history.clear()
        self.get_logger().warn('[RECOVERY] Vehicle stuck detected. Initiating reverse maneuver.')
        self._publish_drive_command(self.reverse_speed)

    def _handle_recovery(self, now: Time) -> None:
        """Continue or finish the recovery maneuver depending on elapsed time."""
        if self.recovery_start_time is None:
            self.get_logger().error('Recovery state inconsistency detected; resetting state.')
            self._finish_recovery()
            return

        elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
        if elapsed < self.reverse_duration:
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
