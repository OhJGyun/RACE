#!/usr/bin/env python3

"""
고주파 로컬라이제이션 노드
AMCL(저주파) + 오도메트리 인터폴레이션(고주파) 융합

AMCL: 15Hz, 정확한 글로벌 위치 보정
Odometry: 100Hz, 부드러운 로컬 추정
Output: 100Hz, 정밀한 위치 정보
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math
from collections import deque
import time

class HighFrequencyLocalizer(Node):
    def __init__(self):
        super().__init__('high_frequency_localizer_node')

        # 구독자
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # 발행자
        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            '/fused_pose',
            10
        )

        self.fused_pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/fused_pose_cov',
            10
        )

        self.high_freq_odom_pub = self.create_publisher(
            Odometry,
            '/high_freq_odom',
            10
        )

        # 상태 변수
        self.last_amcl_pose = None
        self.last_amcl_time = None
        self.last_odom_pose = None
        self.last_odom_time = None
        self.last_imu_data = None

        # 드리프트 보정
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0

        # 고주파 타이머 (100Hz)
        self.create_timer(0.01, self.publish_high_freq_pose)

        # 성능 모니터링
        self.pose_timestamps = deque(maxlen=1000)
        self.create_timer(5.0, self.print_performance)

        self.get_logger().info("고주파 로컬라이제이션 노드 시작 (100Hz)")

    def amcl_callback(self, msg):
        """AMCL 포즈 업데이트 - 드리프트 보정"""
        current_time = self.get_clock().now()

        # 현재 오도메트리와 AMCL 포즈 차이 계산 (드리프트)
        if self.last_odom_pose is not None:
            amcl_x = msg.pose.pose.position.x
            amcl_y = msg.pose.pose.position.y

            # 쿼터니언을 오일러각으로 변환
            amcl_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

            odom_x = self.last_odom_pose.position.x
            odom_y = self.last_odom_pose.position.y
            odom_yaw = self.quaternion_to_yaw(self.last_odom_pose.orientation)

            # 드리프트 계산
            self.drift_x = amcl_x - odom_x
            self.drift_y = amcl_y - odom_y
            self.drift_yaw = self.normalize_angle(amcl_yaw - odom_yaw)

            self.get_logger().info(
                f"드리프트 보정: dx={self.drift_x:.3f}, dy={self.drift_y:.3f}, "
                f"dyaw={math.degrees(self.drift_yaw):.1f}°"
            )

        self.last_amcl_pose = msg.pose.pose
        self.last_amcl_time = current_time

    def odom_callback(self, msg):
        """오도메트리 업데이트"""
        self.last_odom_pose = msg.pose.pose
        self.last_odom_time = self.get_clock().now()

    def imu_callback(self, msg):
        """IMU 데이터 저장"""
        self.last_imu_data = msg

    def publish_high_freq_pose(self):
        """100Hz로 융합된 고정밀 포즈 발행"""
        if self.last_odom_pose is None:
            return

        current_time = self.get_clock().now()

        # 드리프트 보정된 오도메트리 계산
        corrected_x = self.last_odom_pose.position.x + self.drift_x
        corrected_y = self.last_odom_pose.position.y + self.drift_y

        odom_yaw = self.quaternion_to_yaw(self.last_odom_pose.orientation)
        corrected_yaw = self.normalize_angle(odom_yaw + self.drift_yaw)

        # 고주파 포즈 메시지 생성
        fused_pose = PoseStamped()
        fused_pose.header.stamp = current_time.to_msg()
        fused_pose.header.frame_id = "map"

        fused_pose.pose.position.x = corrected_x
        fused_pose.pose.position.y = corrected_y
        fused_pose.pose.position.z = 0.0

        # 오일러각을 쿼터니언으로 변환
        fused_pose.pose.orientation = self.yaw_to_quaternion(corrected_yaw)

        # 발행
        self.fused_pose_pub.publish(fused_pose)

        # PoseWithCovarianceStamped 버전도 발행 (Pure Pursuit 호환성)
        fused_pose_cov = PoseWithCovarianceStamped()
        fused_pose_cov.header = fused_pose.header
        fused_pose_cov.pose.pose = fused_pose.pose

        # 간단한 covariance 설정 (실제로는 더 정교하게 계산 가능)
        fused_pose_cov.pose.covariance[0] = 0.1   # x variance
        fused_pose_cov.pose.covariance[7] = 0.1   # y variance
        fused_pose_cov.pose.covariance[35] = 0.05 # yaw variance

        self.fused_pose_cov_pub.publish(fused_pose_cov)

        # 성능 모니터링
        self.pose_timestamps.append(time.time())

        # 고주파 오도메트리도 발행
        high_freq_odom = Odometry()
        high_freq_odom.header = fused_pose.header
        high_freq_odom.child_frame_id = "base_link"
        high_freq_odom.pose.pose = fused_pose.pose

        # 속도 정보 복사 (오도메트리에서)
        if hasattr(self, 'last_odom_twist'):
            high_freq_odom.twist = self.last_odom_twist

        self.high_freq_odom_pub.publish(high_freq_odom)

    def quaternion_to_yaw(self, quaternion):
        """쿼터니언을 yaw 각도로 변환"""
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z

        # yaw 계산
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def yaw_to_quaternion(self, yaw):
        """yaw 각도를 쿼터니언으로 변환"""
        from geometry_msgs.msg import Quaternion

        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def print_performance(self):
        """성능 통계 출력"""
        if len(self.pose_timestamps) < 10:
            return

        # 최근 1초간의 주파수 계산
        current_time = time.time()
        recent_timestamps = [t for t in self.pose_timestamps if current_time - t <= 1.0]

        if len(recent_timestamps) > 1:
            frequency = len(recent_timestamps)
            self.get_logger().info(f"고주파 로컬라이제이션 주파수: {frequency} Hz")

def main():
    rclpy.init()

    print("\n" + "="*60)
    print("고주파 로컬라이제이션 시스템")
    print("="*60)
    print("AMCL (저주파) + 오도메트리 인터폴레이션 (고주파)")
    print("출력: /fused_pose (100Hz)")
    print("="*60)

    node = HighFrequencyLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n고주파 로컬라이제이션 종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()