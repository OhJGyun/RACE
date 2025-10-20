#!/usr/bin/env python3

"""
로컬라이제이션 성능 평가 테스트 코드

업데이트 속도, 위치 분산, 추정 정확도 등을 실시간으로 모니터링하고 분석

사용법:
ros2 run slam_nav localization_performance_test.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import numpy as np
import time
import math
import json
import csv
import os
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

class LocalizationPerformanceTest(Node):
    def __init__(self):
        super().__init__('localization_performance_test')

        # 구독자 설정
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # TF 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 성능 데이터 저장
        self.pose_timestamps = deque(maxlen=1000)
        self.poses = deque(maxlen=1000)
        self.covariances = deque(maxlen=1000)

        # 통계 변수
        self.last_pose_time = None
        self.update_intervals = deque(maxlen=100)

        # 이동 거리 추적
        self.last_position = None
        self.total_distance = 0.0
        self.position_history = deque(maxlen=500)

        # Map alignment 관련 변수
        self.current_map = None
        self.current_scan = None
        self.current_pose = None
        self.alignment_scores = deque(maxlen=100)
        self.obstacle_distances = deque(maxlen=100)
        self.scan_timestamps = deque(maxlen=100)

        # 데이터 저장 관련 변수
        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        self.data_dir = "/home/f1/f1tenth_ws/localization_logs"
        self.ensure_data_directory()

        # 누적 데이터 저장용 (제한 없음)
        self.all_performance_data = []
        self.last_save_time = time.time()
        self.save_interval = 10.0  # 10초마다 저장

        # CSV 파일 경로 설정
        self.csv_filename = os.path.join(self.data_dir, f"localization_states_{self.session_id}.csv")
        self.state_data = []  # 위치 상태 벡터 데이터 저장용

        # 타이머 설정
        self.create_timer(5.0, self.print_statistics)
        self.create_timer(self.save_interval, self.save_data_periodic)

        self.get_logger().info("로컬라이제이션 성능 테스트 시작...")
        self.get_logger().info("AMCL pose 데이터를 수집 중...")

    def amcl_pose_callback(self, msg):
        current_time = time.time()

        # 업데이트 주기 계산
        if self.last_pose_time is not None:
            interval = current_time - self.last_pose_time
            self.update_intervals.append(interval)

        self.last_pose_time = current_time

        # 포즈 데이터 저장
        pose = msg.pose.pose
        self.pose_timestamps.append(current_time)
        self.poses.append(pose)

        # 공분산 데이터 저장
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        self.covariances.append(cov)

        # 위치 변화 추적
        current_pos = [pose.position.x, pose.position.y]
        self.position_history.append(current_pos)

        if self.last_position is not None:
            distance = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 +
                (current_pos[1] - self.last_position[1])**2
            )
            self.total_distance += distance

        self.last_position = current_pos
        self.current_pose = pose

        # 상태 벡터 데이터 저장
        self.save_state_vector(current_time, pose, cov)

    def scan_callback(self, msg):
        """LiDAR 스캔 데이터 콜백"""
        self.current_scan = msg
        self.scan_timestamps.append(time.time())

        # Map alignment 계산 (맵과 포즈가 모두 있을 때만)
        if self.current_map is not None and self.current_pose is not None:
            alignment_score = self.calculate_scan_map_alignment()
            if alignment_score is not None:
                self.alignment_scores.append(alignment_score)

            obstacle_distance = self.calculate_point_obstacle_distance()
            if obstacle_distance is not None:
                self.obstacle_distances.append(obstacle_distance)

    def map_callback(self, msg):
        """맵 데이터 콜백"""
        self.current_map = msg
        self.get_logger().info("맵 데이터 수신 완료")

    def calculate_position_variance(self, window_size=50):
        """최근 N개 위치의 분산 계산"""
        if len(self.position_history) < window_size:
            return None, None

        recent_positions = list(self.position_history)[-window_size:]
        positions = np.array(recent_positions)

        var_x = np.var(positions[:, 0])
        var_y = np.var(positions[:, 1])

        return var_x, var_y

    def calculate_update_rate(self):
        """업데이트 주기 통계 계산"""
        if len(self.update_intervals) == 0:
            return None, None, None

        intervals = np.array(self.update_intervals)
        mean_interval = np.mean(intervals)
        std_interval = np.std(intervals)
        frequency = 1.0 / mean_interval if mean_interval > 0 else 0

        return frequency, mean_interval, std_interval

    def calculate_covariance_stats(self):
        """공분산 매트릭스 통계"""
        if len(self.covariances) == 0:
            return None, None, None, None

        recent_cov = self.covariances[-1]

        # 위치 불확실성 (x, y)
        pos_uncertainty_x = math.sqrt(recent_cov[0, 0])
        pos_uncertainty_y = math.sqrt(recent_cov[1, 1])

        # 방향 불확실성
        orientation_uncertainty = math.sqrt(recent_cov[5, 5])

        # 전체 위치 불확실성
        total_pos_uncertainty = math.sqrt(pos_uncertainty_x**2 + pos_uncertainty_y**2)

        return pos_uncertainty_x, pos_uncertainty_y, orientation_uncertainty, total_pos_uncertainty

    def calculate_movement_stats(self):
        """이동 통계 계산"""
        if len(self.position_history) < 2:
            return 0.0, 0.0

        # 현재 속도 추정 (최근 5개 포인트 기반)
        if len(self.position_history) >= 5 and len(self.pose_timestamps) >= 5:
            recent_positions = list(self.position_history)[-5:]
            recent_times = list(self.pose_timestamps)[-5:]

            time_diff = recent_times[-1] - recent_times[0]
            if time_diff > 0:
                distance_moved = math.sqrt(
                    (recent_positions[-1][0] - recent_positions[0][0])**2 +
                    (recent_positions[-1][1] - recent_positions[0][1])**2
                )
                current_velocity = distance_moved / time_diff
            else:
                current_velocity = 0.0
        else:
            current_velocity = 0.0

        return self.total_distance, current_velocity

    def calculate_scan_map_alignment(self):
        """스캔-맵 정렬 점수 계산"""
        if self.current_scan is None or self.current_map is None or self.current_pose is None:
            return None

        try:
            # LiDAR 포인트들을 맵 좌표계로 변환
            scan_points = self.scan_to_cartesian(self.current_scan, self.current_pose)
            if len(scan_points) == 0:
                return None

            # 맵에서 장애물과의 매칭 확인
            matched_points = 0
            total_points = len(scan_points)

            for point in scan_points:
                if self.is_point_near_obstacle(point, self.current_map):
                    matched_points += 1

            alignment_score = matched_points / total_points if total_points > 0 else 0.0
            return alignment_score

        except Exception as e:
            self.get_logger().warn(f"Alignment 계산 오류: {e}")
            return None

    def calculate_point_obstacle_distance(self):
        """각 LiDAR 포인트와 장애물 간의 평균 거리"""
        if self.current_scan is None or self.current_map is None or self.current_pose is None:
            return None

        try:
            scan_points = self.scan_to_cartesian(self.current_scan, self.current_pose)
            if len(scan_points) == 0:
                return None

            distances = []
            for point in scan_points:
                min_dist = self.get_min_distance_to_obstacle(point, self.current_map)
                if min_dist is not None:
                    distances.append(min_dist)

            return np.mean(distances) if distances else None

        except Exception as e:
            self.get_logger().warn(f"Distance 계산 오류: {e}")
            return None

    def scan_to_cartesian(self, scan, pose):
        """LiDAR 스캔을 맵 좌표계의 카테시안 좌표로 변환"""
        points = []

        angle = scan.angle_min
        for range_val in scan.ranges:
            if scan.range_min <= range_val <= scan.range_max:
                # 로봇 좌표계에서의 포인트
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)

                # 맵 좌표계로 변환
                cos_theta = math.cos(pose.orientation.z)  # 간단한 2D 변환
                sin_theta = math.sin(pose.orientation.z)

                global_x = pose.position.x + (local_x * cos_theta - local_y * sin_theta)
                global_y = pose.position.y + (local_x * sin_theta + local_y * cos_theta)

                points.append([global_x, global_y])

            angle += scan.angle_increment

        return np.array(points)

    def is_point_near_obstacle(self, point, map_msg, threshold=0.15):
        """포인트가 맵의 장애물 근처에 있는지 확인"""
        # 맵 좌표를 그리드 인덱스로 변환
        grid_x = int((point[0] - map_msg.info.origin.position.x) / map_msg.info.resolution)
        grid_y = int((point[1] - map_msg.info.origin.position.y) / map_msg.info.resolution)

        # 맵 경계 확인
        if (grid_x < 0 or grid_x >= map_msg.info.width or
            grid_y < 0 or grid_y >= map_msg.info.height):
            return False

        # 주변 셀들 확인 (threshold 내의 영역)
        search_radius = int(threshold / map_msg.info.resolution)

        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy

                if (0 <= check_x < map_msg.info.width and
                    0 <= check_y < map_msg.info.height):

                    index = check_y * map_msg.info.width + check_x
                    if index < len(map_msg.data) and map_msg.data[index] > 50:  # 장애물 임계값
                        return True

        return False

    def get_min_distance_to_obstacle(self, point, map_msg, max_search_dist=1.0):
        """포인트에서 가장 가까운 장애물까지의 거리"""
        min_distance = float('inf')

        # 맵 좌표를 그리드 인덱스로 변환
        grid_x = int((point[0] - map_msg.info.origin.position.x) / map_msg.info.resolution)
        grid_y = int((point[1] - map_msg.info.origin.position.y) / map_msg.info.resolution)

        # 검색 반경 (그리드 셀 단위)
        search_radius = int(max_search_dist / map_msg.info.resolution)

        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy

                if (0 <= check_x < map_msg.info.width and
                    0 <= check_y < map_msg.info.height):

                    index = check_y * map_msg.info.width + check_x
                    if index < len(map_msg.data) and map_msg.data[index] > 50:
                        # 장애물 발견 - 거리 계산
                        obstacle_x = map_msg.info.origin.position.x + check_x * map_msg.info.resolution
                        obstacle_y = map_msg.info.origin.position.y + check_y * map_msg.info.resolution

                        distance = math.sqrt((point[0] - obstacle_x)**2 + (point[1] - obstacle_y)**2)
                        min_distance = min(min_distance, distance)

        return min_distance if min_distance != float('inf') else None

    def calculate_map_alignment_stats(self):
        """Map alignment 통계 계산"""
        if len(self.alignment_scores) == 0:
            return None, None, None, None

        # Alignment score 통계
        recent_scores = list(self.alignment_scores)[-20:]  # 최근 20개
        mean_alignment = np.mean(recent_scores)
        std_alignment = np.std(recent_scores)

        # Obstacle distance 통계
        obstacle_stats = None, None
        if len(self.obstacle_distances) > 0:
            recent_distances = list(self.obstacle_distances)[-20:]
            mean_distance = np.mean(recent_distances)
            std_distance = np.std(recent_distances)
            obstacle_stats = mean_distance, std_distance

        return mean_alignment, std_alignment, obstacle_stats[0], obstacle_stats[1]

    def ensure_data_directory(self):
        """데이터 저장 디렉터리 생성"""
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            self.get_logger().info(f"데이터 저장 디렉터리 생성: {self.data_dir}")

    def save_state_vector(self, timestamp, pose, covariance):
        """상태 벡터 데이터를 저장"""
        # 쿼터니언에서 오일러 각 추출 (단순 2D 근사)
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # 2D yaw 각도 계산
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        # 공분산 추출
        cov_flat = covariance.flatten()

        state_record = {
            'timestamp': timestamp,
            'datetime': datetime.fromtimestamp(timestamp).isoformat(),
            'session_id': self.session_id,

            # 위치 상태
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'yaw': yaw,

            # 공분산 매트릭스 (6x6 = 36개 요소)
            'cov_xx': cov_flat[0],   # x-x
            'cov_xy': cov_flat[1],   # x-y
            'cov_xz': cov_flat[2],   # x-z
            'cov_x_roll': cov_flat[3],
            'cov_x_pitch': cov_flat[4],
            'cov_x_yaw': cov_flat[5],

            'cov_yy': cov_flat[7],   # y-y
            'cov_yz': cov_flat[8],   # y-z
            'cov_y_roll': cov_flat[9],
            'cov_y_pitch': cov_flat[10],
            'cov_y_yaw': cov_flat[11],

            'cov_zz': cov_flat[14],  # z-z
            'cov_z_roll': cov_flat[15],
            'cov_z_pitch': cov_flat[16],
            'cov_z_yaw': cov_flat[17],

            'cov_roll_roll': cov_flat[21],
            'cov_roll_pitch': cov_flat[22],
            'cov_roll_yaw': cov_flat[23],

            'cov_pitch_pitch': cov_flat[28],
            'cov_pitch_yaw': cov_flat[29],

            'cov_yaw_yaw': cov_flat[35],  # yaw-yaw

            # 불확실성 지표
            'uncertainty_x': math.sqrt(abs(cov_flat[0])),
            'uncertainty_y': math.sqrt(abs(cov_flat[7])),
            'uncertainty_yaw': math.sqrt(abs(cov_flat[35])),
            'uncertainty_total': math.sqrt(cov_flat[0] + cov_flat[7])
        }

        self.state_data.append(state_record)

        # 실시간으로 CSV에 저장 (헤더 관리)
        self.write_state_to_csv(state_record)

    def write_state_to_csv(self, state_record):
        """상태 데이터를 CSV 파일에 실시간 저장"""
        file_exists = os.path.exists(self.csv_filename)

        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=state_record.keys())

            if not file_exists:
                writer.writeheader()
                self.get_logger().info(f"상태 벡터 CSV 파일 생성: {self.csv_filename}")

            writer.writerow(state_record)

    def collect_current_performance_data(self):
        """현재 성능 데이터를 수집하여 딕셔너리로 반환"""
        current_time = time.time()

        # 기본 성능 지표
        freq, mean_interval, std_interval = self.calculate_update_rate()
        var_x, var_y = self.calculate_position_variance()
        unc_x, unc_y, unc_theta, total_unc = self.calculate_covariance_stats()
        total_dist, velocity = self.calculate_movement_stats()
        mean_align, std_align, mean_dist, std_dist = self.calculate_map_alignment_stats()

        data_point = {
            'timestamp': current_time,
            'datetime': datetime.fromtimestamp(current_time).isoformat(),
            'session_id': self.session_id,

            # 업데이트 성능
            'update_frequency': freq,
            'update_interval_mean': mean_interval,
            'update_interval_std': std_interval,

            # 위치 안정성
            'position_variance_x': var_x,
            'position_variance_y': var_y,
            'position_variance_total': (var_x + var_y) if var_x is not None and var_y is not None else None,

            # 불확실성
            'uncertainty_x': unc_x,
            'uncertainty_y': unc_y,
            'uncertainty_theta': unc_theta,
            'uncertainty_total': total_unc,

            # 이동 통계
            'total_distance': total_dist,
            'current_velocity': velocity,

            # Map alignment
            'alignment_score_mean': mean_align,
            'alignment_score_std': std_align,
            'obstacle_distance_mean': mean_dist,
            'obstacle_distance_std': std_dist,

            # 현재 위치 (있는 경우)
            'current_x': self.current_pose.position.x if self.current_pose else None,
            'current_y': self.current_pose.position.y if self.current_pose else None,
            'current_theta': self.current_pose.orientation.z if self.current_pose else None,

            # 데이터 수집 상태
            'collected_poses': len(self.poses),
            'collected_scans': len(self.scan_timestamps),
            'alignment_analyses': len(self.alignment_scores)
        }

        return data_point

    def save_data_periodic(self):
        """주기적으로 데이터 저장"""
        if len(self.poses) < 5:  # 충분한 데이터가 없으면 저장하지 않음
            return

        current_data = self.collect_current_performance_data()
        if current_data:
            self.all_performance_data.append(current_data)

            # CSV 파일로 저장 (추가 모드)
            csv_file = os.path.join(self.data_dir, f"performance_{self.session_id}.csv")
            self.save_to_csv(current_data, csv_file)

    def save_to_csv(self, data_point, csv_file):
        """단일 데이터 포인트를 CSV에 저장"""
        file_exists = os.path.exists(csv_file)

        with open(csv_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=data_point.keys())

            if not file_exists:
                writer.writeheader()

            writer.writerow(data_point)

    def save_session_summary(self):
        """세션 종료 시 전체 요약 저장"""
        if not self.all_performance_data:
            return

        summary_file = os.path.join(self.data_dir, f"session_summary_{self.session_id}.json")

        # 세션 요약 통계 계산
        summary = {
            'session_info': {
                'session_id': self.session_id,
                'start_time': self.session_start_time.isoformat(),
                'end_time': datetime.now().isoformat(),
                'duration_seconds': time.time() - self.pose_timestamps[0] if self.pose_timestamps else 0,
                'total_data_points': len(self.all_performance_data)
            },
            'performance_summary': self.calculate_session_statistics(),
            'raw_data_file': f"performance_{self.session_id}.csv"
        }

        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(f"세션 요약 저장 완료: {summary_file}")

    def calculate_session_statistics(self):
        """전체 세션의 통계 계산"""
        if not self.all_performance_data:
            return {}

        # 유효한 데이터만 필터링
        valid_data = [d for d in self.all_performance_data if d.get('update_frequency') is not None]

        if not valid_data:
            return {}

        stats = {}

        # 각 지표별 통계 계산
        numeric_fields = [
            'update_frequency', 'position_variance_total', 'uncertainty_total',
            'current_velocity', 'alignment_score_mean', 'obstacle_distance_mean'
        ]

        for field in numeric_fields:
            values = [d[field] for d in valid_data if d.get(field) is not None]
            if values:
                stats[field] = {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': np.min(values),
                    'max': np.max(values),
                    'median': np.median(values)
                }

        return stats

    def print_statistics(self):
        """주기적으로 통계 출력"""
        if len(self.poses) < 5:
            self.get_logger().info("데이터 수집 중... (충분한 데이터 필요)")
            return

        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("로컬라이제이션 성능 통계")
        self.get_logger().info("="*80)

        # 1. 업데이트 성능
        freq, mean_interval, std_interval = self.calculate_update_rate()
        if freq is not None:
            self.get_logger().info(f"📊 업데이트 성능:")
            self.get_logger().info(f"   - 주파수: {freq:.2f} Hz")
            self.get_logger().info(f"   - 평균 간격: {mean_interval*1000:.1f} ms")
            self.get_logger().info(f"   - 간격 편차: ±{std_interval*1000:.1f} ms")

            if freq < 20:
                self.get_logger().warn("⚠️  업데이트 주파수가 낮습니다 (20Hz 이상 권장)")
            else:
                self.get_logger().info("✅ 업데이트 주파수 양호")

        # 2. 위치 분산 (안정성)
        var_x, var_y = self.calculate_position_variance()
        if var_x is not None:
            self.get_logger().info(f"\n📍 위치 안정성 (최근 50개 샘플):")
            self.get_logger().info(f"   - X축 분산: {var_x:.6f} m²")
            self.get_logger().info(f"   - Y축 분산: {var_y:.6f} m²")
            self.get_logger().info(f"   - 총 분산: {var_x + var_y:.6f} m²")

            if var_x + var_y > 0.001:  # 1mm²
                self.get_logger().warn("⚠️  위치 분산이 높습니다 (불안정)")
            else:
                self.get_logger().info("✅ 위치 추정 안정")

        # 3. 불확실성 (공분산)
        unc_x, unc_y, unc_theta, total_unc = self.calculate_covariance_stats()
        if unc_x is not None:
            self.get_logger().info(f"\n🎯 추정 불확실성:")
            self.get_logger().info(f"   - X축 불확실성: ±{unc_x:.3f} m")
            self.get_logger().info(f"   - Y축 불확실성: ±{unc_y:.3f} m")
            self.get_logger().info(f"   - 방향 불확실성: ±{math.degrees(unc_theta):.1f}°")
            self.get_logger().info(f"   - 총 위치 불확실성: ±{total_unc:.3f} m")

            if total_unc > 0.1:  # 10cm
                self.get_logger().warn("⚠️  위치 불확실성이 높습니다")
            else:
                self.get_logger().info("✅ 위치 불확실성 양호")

        # 4. 이동 통계
        total_dist, velocity = self.calculate_movement_stats()
        self.get_logger().info(f"\n🚗 이동 통계:")
        self.get_logger().info(f"   - 총 이동거리: {total_dist:.2f} m")
        self.get_logger().info(f"   - 현재 속도: {velocity:.2f} m/s ({velocity*3.6:.1f} km/h)")

        # 5. Map Alignment 성능
        mean_align, std_align, mean_dist, std_dist = self.calculate_map_alignment_stats()
        if mean_align is not None:
            self.get_logger().info(f"\n🗺️  Map Alignment 성능:")
            self.get_logger().info(f"   - 스캔-맵 매칭률: {mean_align:.3f} ({mean_align*100:.1f}%)")
            self.get_logger().info(f"   - 매칭률 편차: ±{std_align:.3f}")

            if mean_align > 0.85:
                self.get_logger().info("✅ 스캔-맵 정렬 상태 양호")
            elif mean_align > 0.70:
                self.get_logger().warn("⚠️  스캔-맵 정렬 상태 보통")
            else:
                self.get_logger().warn("❌ 스캔-맵 정렬 상태 불량")

            if mean_dist is not None:
                self.get_logger().info(f"   - 평균 장애물 거리: {mean_dist:.3f} m")
                self.get_logger().info(f"   - 거리 편차: ±{std_dist:.3f} m")

                if mean_dist < 0.15:
                    self.get_logger().info("✅ 장애물 정렬 정확도 양호")
                elif mean_dist < 0.25:
                    self.get_logger().warn("⚠️  장애물 정렬 정확도 보통")
                else:
                    self.get_logger().warn("❌ 장애물 정렬 정확도 불량")

        # 6. 데이터 수집 정보
        self.get_logger().info(f"\n📈 데이터 수집:")
        self.get_logger().info(f"   - 수집된 포즈: {len(self.poses)}개")
        self.get_logger().info(f"   - 수집된 스캔: {len(self.scan_timestamps)}개")
        self.get_logger().info(f"   - 정렬 분석 횟수: {len(self.alignment_scores)}개")
        self.get_logger().info(f"   - 수집 시간: {(self.pose_timestamps[-1] - self.pose_timestamps[0]):.1f}초")
        self.get_logger().info(f"   - 데이터 저장 위치: {self.data_dir}")
        self.get_logger().info(f"   - 상태 벡터 저장: {len(self.state_data)}개 (CSV: {self.csv_filename})")

    def cleanup_and_save(self):
        """종료 시 정리 및 최종 저장"""
        self.save_session_summary()
        self.get_logger().info("데이터 저장 완료. 시각화를 위해 analysis 스크립트를 사용하세요.")
        self.get_logger().info(f"상태 벡터 CSV 파일: {self.csv_filename}")
        self.get_logger().info(f"총 {len(self.state_data)}개의 상태 벡터가 저장되었습니다.")

def create_visualization_script():
    """데이터 분석 및 시각화를 위한 별도 스크립트 생성"""
    analysis_script = '''#!/usr/bin/env python3

"""
로컬라이제이션 성능 데이터 분석 및 시각화 도구

사용법:
python3 localization_analysis.py --session SESSION_ID
python3 localization_analysis.py --compare SESSION1 SESSION2
python3 localization_analysis.py --list  # 세션 목록 보기
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import json
import os
from datetime import datetime
import numpy as np

class LocalizationAnalyzer:
    def __init__(self, data_dir="/home/f1/f1tenth_ws/localization_logs"):
        self.data_dir = data_dir

    def list_sessions(self):
        """사용 가능한 세션 목록 출력"""
        if not os.path.exists(self.data_dir):
            print(f"데이터 디렉터리가 없습니다: {self.data_dir}")
            return

        sessions = []
        for file in os.listdir(self.data_dir):
            if file.startswith("session_summary_") and file.endswith(".json"):
                session_id = file.replace("session_summary_", "").replace(".json", "")
                sessions.append(session_id)

        print(f"\\n📊 사용 가능한 세션들 ({len(sessions)}개):")
        print("=" * 50)

        for session_id in sorted(sessions):
            summary_file = os.path.join(self.data_dir, f"session_summary_{session_id}.json")
            try:
                with open(summary_file, 'r') as f:
                    summary = json.load(f)

                start_time = summary['session_info']['start_time']
                duration = summary['session_info']['duration_seconds']
                data_points = summary['session_info']['total_data_points']

                print(f"🔹 {session_id}")
                print(f"   시작 시간: {start_time}")
                print(f"   지속 시간: {duration:.1f}초")
                print(f"   데이터 포인트: {data_points}개")
                print()

            except Exception as e:
                print(f"❌ {session_id}: 요약 파일 읽기 실패 - {e}")

    def load_session_data(self, session_id):
        """세션 데이터 로드"""
        csv_file = os.path.join(self.data_dir, f"performance_{session_id}.csv")
        summary_file = os.path.join(self.data_dir, f"session_summary_{session_id}.json")

        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"세션 데이터 파일이 없습니다: {csv_file}")

        # CSV 데이터 로드
        df = pd.read_csv(csv_file)
        df['datetime'] = pd.to_datetime(df['datetime'])

        # 요약 데이터 로드
        summary = None
        if os.path.exists(summary_file):
            with open(summary_file, 'r') as f:
                summary = json.load(f)

        return df, summary

    def visualize_session(self, session_id, save_plots=True):
        """단일 세션 시각화"""
        print(f"\\n📈 세션 {session_id} 분석 중...")

        try:
            df, summary = self.load_session_data(session_id)
        except FileNotFoundError as e:
            print(f"❌ {e}")
            return

        # 4개 서브플롯 생성
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'로컬라이제이션 성능 분석 - {session_id}', fontsize=16)

        # 1. 업데이트 주파수
        valid_freq = df['update_frequency'].dropna()
        if not valid_freq.empty:
            ax1.plot(df.loc[valid_freq.index, 'datetime'], valid_freq, 'b-', alpha=0.7)
            ax1.axhline(y=20, color='r', linestyle='--', alpha=0.5, label='권장 최소값 (20Hz)')
            ax1.set_title('업데이트 주파수')
            ax1.set_ylabel('Hz')
            ax1.legend()
            ax1.grid(True, alpha=0.3)

        # 2. Map Alignment Score
        valid_align = df['alignment_score_mean'].dropna()
        if not valid_align.empty:
            ax2.plot(df.loc[valid_align.index, 'datetime'], valid_align, 'g-', alpha=0.7)
            ax2.axhline(y=0.85, color='g', linestyle='--', alpha=0.5, label='양호 (85%)')
            ax2.axhline(y=0.70, color='orange', linestyle='--', alpha=0.5, label='보통 (70%)')
            ax2.set_title('Map Alignment Score')
            ax2.set_ylabel('매칭률')
            ax2.legend()
            ax2.grid(True, alpha=0.3)

        # 3. 위치 불확실성
        valid_unc = df['uncertainty_total'].dropna()
        if not valid_unc.empty:
            ax3.plot(df.loc[valid_unc.index, 'datetime'], valid_unc, 'r-', alpha=0.7)
            ax3.axhline(y=0.1, color='r', linestyle='--', alpha=0.5, label='권장 최대값 (10cm)')
            ax3.set_title('위치 불확실성')
            ax3.set_ylabel('미터 (m)')
            ax3.legend()
            ax3.grid(True, alpha=0.3)

        # 4. 장애물 거리
        valid_dist = df['obstacle_distance_mean'].dropna()
        if not valid_dist.empty:
            ax4.plot(df.loc[valid_dist.index, 'datetime'], valid_dist, 'purple', alpha=0.7)
            ax4.axhline(y=0.15, color='g', linestyle='--', alpha=0.5, label='양호 (<15cm)')
            ax4.axhline(y=0.25, color='orange', linestyle='--', alpha=0.5, label='보통 (<25cm)')
            ax4.set_title('평균 장애물 거리')
            ax4.set_ylabel('미터 (m)')
            ax4.legend()
            ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_plots:
            plot_file = os.path.join(self.data_dir, f"analysis_{session_id}.png")
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"📊 시각화 저장 완료: {plot_file}")

        plt.show()

        # 통계 요약 출력
        if summary and 'performance_summary' in summary:
            self.print_session_summary(session_id, summary)

    def compare_sessions(self, session_ids, save_plots=True):
        """여러 세션 비교"""
        print(f"\\n🔄 세션 비교: {', '.join(session_ids)}")

        sessions_data = {}
        for session_id in session_ids:
            try:
                df, summary = self.load_session_data(session_id)
                sessions_data[session_id] = {'data': df, 'summary': summary}
            except FileNotFoundError:
                print(f"❌ 세션 {session_id} 데이터를 찾을 수 없습니다.")
                continue

        if len(sessions_data) < 2:
            print("비교할 수 있는 세션이 부족합니다.")
            return

        # 비교 시각화
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'세션 비교 분석', fontsize=16)

        colors = ['blue', 'red', 'green', 'orange', 'purple']

        for i, (session_id, data) in enumerate(sessions_data.items()):
            df = data['data']
            color = colors[i % len(colors)]

            # 상대 시간으로 변환 (세션 시작부터의 경과 시간)
            df['relative_time'] = (df['datetime'] - df['datetime'].iloc[0]).dt.total_seconds() / 60  # 분 단위

            # 각 지표 플롯
            metrics = [
                ('update_frequency', ax1, '업데이트 주파수 (Hz)'),
                ('alignment_score_mean', ax2, 'Map Alignment Score'),
                ('uncertainty_total', ax3, '위치 불확실성 (m)'),
                ('obstacle_distance_mean', ax4, '평균 장애물 거리 (m)')
            ]

            for metric, ax, ylabel in metrics:
                valid_data = df[metric].dropna()
                if not valid_data.empty:
                    ax.plot(df.loc[valid_data.index, 'relative_time'], valid_data,
                           color=color, alpha=0.7, label=session_id)
                    ax.set_ylabel(ylabel)
                    ax.set_xlabel('시간 (분)')
                    ax.legend()
                    ax.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_plots:
            plot_file = os.path.join(self.data_dir, f"comparison_{'_vs_'.join(session_ids)}.png")
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"📊 비교 시각화 저장: {plot_file}")

        plt.show()

    def print_session_summary(self, session_id, summary):
        """세션 요약 통계 출력"""
        print(f"\\n📋 세션 {session_id} 요약 통계:")
        print("=" * 50)

        session_info = summary['session_info']
        print(f"⏱️  지속 시간: {session_info['duration_seconds']:.1f}초")
        print(f"📊 데이터 포인트: {session_info['total_data_points']}개")

        if 'performance_summary' in summary:
            perf = summary['performance_summary']

            metrics = [
                ('update_frequency', '업데이트 주파수', 'Hz'),
                ('alignment_score_mean', 'Map Alignment', ''),
                ('uncertainty_total', '위치 불확실성', 'm'),
                ('obstacle_distance_mean', '장애물 거리', 'm')
            ]

            for key, name, unit in metrics:
                if key in perf:
                    stats = perf[key]
                    print(f"\\n{name}:")
                    print(f"  평균: {stats['mean']:.3f} {unit}")
                    print(f"  표준편차: {stats['std']:.3f} {unit}")
                    print(f"  범위: {stats['min']:.3f} ~ {stats['max']:.3f} {unit}")

def main():
    parser = argparse.ArgumentParser(description='로컬라이제이션 성능 데이터 분석')
    parser.add_argument('--session', help='분석할 세션 ID')
    parser.add_argument('--compare', nargs='+', help='비교할 세션 ID들')
    parser.add_argument('--list', action='store_true', help='사용 가능한 세션 목록 보기')
    parser.add_argument('--data-dir', default='/home/f1/f1tenth_ws/localization_logs',
                       help='데이터 디렉터리 경로')

    args = parser.parse_args()

    analyzer = LocalizationAnalyzer(args.data_dir)

    if args.list:
        analyzer.list_sessions()
    elif args.session:
        analyzer.visualize_session(args.session)
    elif args.compare:
        analyzer.compare_sessions(args.compare)
    else:
        print("옵션을 선택해주세요. --help를 참조하세요.")

if __name__ == '__main__':
    main()
'''

    analysis_file = "/home/f1/f1tenth_ws/src/slam_nav/scripts/localization_analysis.py"
    with open(analysis_file, 'w') as f:
        f.write(analysis_script)

    # 실행 권한 부여
    os.chmod(analysis_file, 0o755)
    print(f"✅ 분석 스크립트 생성 완료: {analysis_file}")

def main():
    rclpy.init()

    print("\n" + "="*80)
    print("F1TENTH 로컬라이제이션 성능 테스트")
    print("="*80)
    print("이 도구는 AMCL 로컬라이제이션의 성능을 실시간으로 분석합니다.")
    print("\n측정 항목:")
    print("- 업데이트 주파수 및 안정성")
    print("- 위치 추정 분산 (안정성)")
    print("- 불확실성 (공분산 매트릭스)")
    print("- 이동 통계")
    print("- Map Alignment (스캔-맵 정렬)")
    print("- 장애물 거리 분석")
    print("\n로봇을 이동시키면서 결과를 관찰하세요.")
    print("Ctrl+C로 종료합니다.")
    print("="*80)

    # 분석 스크립트 생성 (처음 실행 시에만)
    create_visualization_script()

    node = LocalizationPerformanceTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n테스트가 종료되었습니다.")
        node.cleanup_and_save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()