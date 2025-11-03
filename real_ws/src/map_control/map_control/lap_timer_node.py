#!/usr/bin/env python3
"""
Lap Timer Node for F1TENTH Racing

결승선을 두 점(point1, point2)으로 정의하고, 차량이 선분을 통과할 때마다
랩타임을 측정하여 퍼블리시합니다.
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclTime

from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException


class LapTimerNode(Node):
    def __init__(self) -> None:
        super().__init__("lap_timer_node")

        # ====================================================================
        # 🏁 결승선 포인트 설정 (사용자가 수정하는 부분)
        # ====================================================================
        # Map 좌표계 기준으로 결승선을 정의하는 두 점
        # 예시: (x1, y1), (x2, y2)
        self.finish_line_p1 = np.array([-3.5, 4.6], dtype=np.float64)
        self.finish_line_p2 = np.array([-3.4, 2.2], dtype=np.float64)

        # 결승선 근처 허용 범위 (미터)
        self.finish_line_tolerance = 0.5

        # 최소 랩타임 (초) - 너무 짧은 랩은 오검출로 간주
        self.min_lap_time = 5.0

        # ====================================================================
        # Parameters
        # ====================================================================
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("update_rate_hz", 50.0)

        # 파라미터 로딩
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.marker_frame = str(self.get_parameter("marker_frame_id").value)
        self.update_rate = float(self.get_parameter("update_rate_hz").value)

        # ====================================================================
        # State Variables
        # ====================================================================
        self.pose: Optional[Tuple[float, float, float]] = None
        self.last_signed_distance: Optional[float] = None
        self.lap_counter: int = 0  # -1: warmup, 0+: actual laps
        self.last_cross_time: Optional[float] = None
        self.first_cross_done: bool = False  # warmup lap flag

        # 결승선 법선 벡터 계산 (normalized)
        line_vec = self.finish_line_p2 - self.finish_line_p1
        # 법선 벡터: n = [dy, -dx]
        self.finish_line_normal = np.array([line_vec[1], -line_vec[0]], dtype=np.float64)
        norm = np.linalg.norm(self.finish_line_normal)
        if norm > 1e-6:
            self.finish_line_normal /= norm

        # ====================================================================
        # TF2
        # ====================================================================
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ====================================================================
        # ROS Publishers
        # ====================================================================
        self.pub_lap_time = self.create_publisher(Float32, "/lap_time", 10)
        self.pub_lap_counter = self.create_publisher(Int32, "/lap_counter", 10)
        self.pub_finish_line_marker = self.create_publisher(Marker, "/lap_timer/finish_line", 10)

        # ====================================================================
        # Timers
        # ====================================================================
        if self.update_rate > 0.0:
            self.timer = self.create_timer(1.0 / self.update_rate, self._check_lap)

        self.viz_timer = self.create_timer(1.0, self._publish_finish_line_marker)

        self.get_logger().info(
            f"Lap Timer Node ready. Finish line: "
            f"P1=({self.finish_line_p1[0]:.2f}, {self.finish_line_p1[1]:.2f}), "
            f"P2=({self.finish_line_p2[0]:.2f}, {self.finish_line_p2[1]:.2f})"
        )

    # ========================================================================
    # TF를 통한 Pose 획득 (map -> base_link)
    # ========================================================================
    def _get_current_pose_from_tf(self) -> Optional[Tuple[float, float, float]]:
        """Get current pose from TF (map->base_link)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                RclTime(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            # Convert quaternion to yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except (TransformException, Exception) as e:
            self.get_logger().warn(
                f"TF lookup failed: {e}",
                throttle_duration_sec=1.0
            )
            return None

    # ========================================================================
    # 🏁 랩 체크 메인 로직
    # ========================================================================
    def _check_lap(self) -> None:
        """매 주기마다 호출되어 결승선 통과 여부를 확인"""
        # 현재 차량 위치 획득
        self.pose = self._get_current_pose_from_tf()
        if self.pose is None:
            return

        px, py, yaw = self.pose
        vehicle_pos = np.array([px, py], dtype=np.float64)

        # ====================================================================
        # 🔍 통과 판정 (부호 반전, yaw 체크)
        # ====================================================================

        # 1. 차량 위치에서 결승선까지의 부호 있는 거리 계산
        # d = n · (p - p1)
        diff = vehicle_pos - self.finish_line_p1
        signed_distance = np.dot(self.finish_line_normal, diff)

        # 2. 결승선 근처에 있는지 확인 (허용 범위 체크)
        if abs(signed_distance) > self.finish_line_tolerance:
            self.last_signed_distance = signed_distance
            return

        # 3. 부호 반전 확인 (음수 -> 양수 통과)
        if self.last_signed_distance is None:
            self.last_signed_distance = signed_distance
            return

        crossed = False
        if self.last_signed_distance < 0.0 and signed_distance >= 0.0:
            # 음수에서 양수로 전환 = 결승선 통과 (한 방향)
            crossed = True
        elif self.last_signed_distance > 0.0 and signed_distance <= 0.0:
            # 양수에서 음수로 전환 = 결승선 통과 (반대 방향)
            # 역주행 방지를 위해 yaw 체크
            crossed = False  # 역방향은 무시 (필요시 활성화)

        self.last_signed_distance = signed_distance

        if not crossed:
            return

        # 4. 차량 진행 방향 체크 (선분 방향과의 내적)
        # 결승선 벡터 방향
        line_direction = self.finish_line_p2 - self.finish_line_p1
        line_direction_norm = np.linalg.norm(line_direction)
        if line_direction_norm > 1e-6:
            line_direction /= line_direction_norm

        # 차량 진행 방향 벡터
        vehicle_direction = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)

        # 내적: 양수면 같은 방향, 음수면 반대 방향
        # 법선 벡터와의 내적으로 정방향 통과 확인
        forward_cross = np.dot(vehicle_direction, self.finish_line_normal) > 0.0

        if not forward_cross:
            # 역방향 통과는 무시
            self.get_logger().warn("Reverse direction crossing detected, ignoring")
            return

        # ====================================================================
        # ⏱️ 랩타임 및 랩 카운트 퍼블리시 위치
        # ====================================================================

        now = self.get_clock().now().nanoseconds * 1e-9

        # 첫 번째 통과 = warmup (무시)
        if not self.first_cross_done:
            self.first_cross_done = True
            self.last_cross_time = now
            self.lap_counter = 0
            self.get_logger().info("🏁 Warmup lap completed (not counted)")
            return

        # 랩타임 계산
        if self.last_cross_time is not None:
            lap_time = now - self.last_cross_time

            # 최소 랩타임 체크 (오검출 방지)
            if lap_time < self.min_lap_time:
                self.get_logger().warn(
                    f"Lap time too short ({lap_time:.2f}s), ignoring (min: {self.min_lap_time}s)"
                )
                return

            # 랩 카운터 증가
            self.lap_counter += 1

            # 랩타임 퍼블리시
            lap_time_msg = Float32()
            lap_time_msg.data = float(lap_time)
            self.pub_lap_time.publish(lap_time_msg)

            # 랩 카운터 퍼블리시
            lap_counter_msg = Int32()
            lap_counter_msg.data = self.lap_counter
            self.pub_lap_counter.publish(lap_counter_msg)

            # 로그 출력
            self.get_logger().info(
                f"🏁 Lap {self.lap_counter} completed! "
                f"Time: {lap_time:.3f}s at position ({px:.2f}, {py:.2f})"
            )

            # 다음 랩을 위해 시간 업데이트
            self.last_cross_time = now
        else:
            # 첫 랩 시작
            self.last_cross_time = now

    # ========================================================================
    # 시각화: 결승선 마커
    # ========================================================================
    def _publish_finish_line_marker(self) -> None:
        """결승선을 RViz에 시각화"""
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "finish_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 선 두께
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = Duration(seconds=0).to_msg()

        # 결승선 두 점
        p1 = Point()
        p1.x = float(self.finish_line_p1[0])
        p1.y = float(self.finish_line_p1[1])
        p1.z = 0.0

        p2 = Point()
        p2.x = float(self.finish_line_p2[0])
        p2.y = float(self.finish_line_p2[1])
        p2.z = 0.0

        marker.points = [p1, p2]

        self.pub_finish_line_marker.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LapTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
