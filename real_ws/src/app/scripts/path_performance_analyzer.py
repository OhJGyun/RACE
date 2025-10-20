#!/usr/bin/env python3
"""
Pure Pursuit Performance Analyzer
실제 주행 경로와 기준 경로를 비교하여 성능을 분석하는 도구

Usage:
1. 실제 주행 중 경로 저장: python3 path_performance_analyzer.py --record
2. 분석 실행: python3 path_performance_analyzer.py --analyze reference_path.csv actual_path.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime
import math

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # 구독자 설정
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 데이터 저장용
        self.recorded_path = []
        self.start_time = datetime.now()

        # 파라미터
        self.recording_interval = 0.1  # 0.1초마다 기록
        self.last_record_time = 0.0

        self.get_logger().info('Path recording started...')

    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 일정 간격으로만 기록
        if current_time - self.last_record_time > self.recording_interval:
            pose = msg.pose.pose
            twist = msg.twist.twist

            # 데이터 기록
            self.recorded_path.append({
                'timestamp': current_time,
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': self.quaternion_to_yaw(pose.orientation),
                'speed': math.sqrt(twist.linear.x**2 + twist.linear.y**2),
                'angular_velocity': twist.angular.z
            })

            self.last_record_time = current_time

    def quaternion_to_yaw(self, quaternion):
        """쿼터니언을 yaw 각도로 변환"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_path(self, filename=None):
        """기록된 경로를 CSV 파일로 저장"""
        if not filename:
            timestamp = self.start_time.strftime("%Y%m%d_%H%M%S")
            filename = f"recorded_path_{timestamp}.csv"

        df = pd.DataFrame(self.recorded_path)
        filepath = os.path.join('/home/f1/f1tenth_ws/src/pure_pursuit/data', filename)

        # 디렉토리 생성
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        df.to_csv(filepath, index=False)
        self.get_logger().info(f'Path saved to: {filepath}')
        return filepath

class PathAnalyzer:
    def __init__(self):
        self.reference_path = None
        self.actual_path = None

    def load_reference_path(self, filepath):
        """기준 경로 CSV 파일 로드"""
        try:
            df = pd.read_csv(filepath)
            # CSV 파일 형식에 따라 컬럼명 조정
            if 'x_m' in df.columns:  # raceline CSV 형식
                self.reference_path = df[['x_m', 'y_m']].rename(columns={'x_m': 'x', 'y_m': 'y'})
                if 'vx_mps' in df.columns:
                    self.reference_path['speed'] = df['vx_mps']
            else:  # 일반적인 x, y 형식
                self.reference_path = df[['x', 'y']]
                if 'speed' in df.columns:
                    self.reference_path['speed'] = df['speed']

            print(f"Reference path loaded: {len(self.reference_path)} points")
            return True
        except Exception as e:
            print(f"Error loading reference path: {e}")
            return False

    def load_actual_path(self, filepath):
        """실제 주행 경로 CSV 파일 로드"""
        try:
            self.actual_path = pd.read_csv(filepath)
            print(f"Actual path loaded: {len(self.actual_path)} points")
            return True
        except Exception as e:
            print(f"Error loading actual path: {e}")
            return False

    def calculate_cross_track_error(self):
        """Cross-track error 계산"""
        if self.reference_path is None or self.actual_path is None:
            return None

        errors = []
        ref_points = self.reference_path[['x', 'y']].values

        for _, actual_point in self.actual_path.iterrows():
            actual_pos = np.array([actual_point['x'], actual_point['y']])

            # 가장 가까운 기준점 찾기
            distances = np.linalg.norm(ref_points - actual_pos, axis=1)
            min_idx = np.argmin(distances)

            errors.append(distances[min_idx])

        return np.array(errors)

    def calculate_speed_comparison(self):
        """속도 비교 분석"""
        if 'speed' not in self.reference_path.columns or 'speed' not in self.actual_path.columns:
            return None, None

        ref_speeds = self.reference_path['speed'].values
        actual_speeds = self.actual_path['speed'].values

        # 길이 맞추기 (더 짧은 것에 맞춤)
        min_len = min(len(ref_speeds), len(actual_speeds))
        ref_speeds = ref_speeds[:min_len]
        actual_speeds = actual_speeds[:min_len]

        return ref_speeds, actual_speeds

    def generate_performance_report(self, output_dir='/home/f1/f1tenth_ws/src/pure_pursuit/analysis'):
        """성능 분석 리포트 생성"""
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Cross-track error 분석
        cte = self.calculate_cross_track_error()

        # 속도 비교 분석
        ref_speeds, actual_speeds = self.calculate_speed_comparison()

        # 결과 출력
        print("\n=== PERFORMANCE ANALYSIS REPORT ===")
        print(f"Analysis Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Reference Path Points: {len(self.reference_path)}")
        print(f"Actual Path Points: {len(self.actual_path)}")

        if cte is not None:
            print(f"\n--- Cross-Track Error ---")
            print(f"Mean CTE: {np.mean(cte):.4f} m")
            print(f"Max CTE: {np.max(cte):.4f} m")
            print(f"RMS CTE: {np.sqrt(np.mean(cte**2)):.4f} m")
            print(f"Std CTE: {np.std(cte):.4f} m")

        if ref_speeds is not None and actual_speeds is not None:
            speed_diff = actual_speeds - ref_speeds
            print(f"\n--- Speed Analysis ---")
            print(f"Mean Reference Speed: {np.mean(ref_speeds):.2f} m/s")
            print(f"Mean Actual Speed: {np.mean(actual_speeds):.2f} m/s")
            print(f"Mean Speed Difference: {np.mean(speed_diff):.2f} m/s")
            print(f"Speed Tracking RMS: {np.sqrt(np.mean(speed_diff**2)):.2f} m/s")

        # 시각화
        self.create_visualizations(cte, ref_speeds, actual_speeds, output_dir, timestamp)

        # 리포트 파일 저장
        report_file = os.path.join(output_dir, f"performance_report_{timestamp}.txt")
        with open(report_file, 'w') as f:
            f.write("=== PURE PURSUIT PERFORMANCE ANALYSIS ===\n")
            f.write(f"Analysis Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Reference Path Points: {len(self.reference_path)}\n")
            f.write(f"Actual Path Points: {len(self.actual_path)}\n")

            if cte is not None:
                f.write(f"\n--- Cross-Track Error ---\n")
                f.write(f"Mean CTE: {np.mean(cte):.4f} m\n")
                f.write(f"Max CTE: {np.max(cte):.4f} m\n")
                f.write(f"RMS CTE: {np.sqrt(np.mean(cte**2)):.4f} m\n")
                f.write(f"Std CTE: {np.std(cte):.4f} m\n")

            if ref_speeds is not None and actual_speeds is not None:
                speed_diff = actual_speeds - ref_speeds
                f.write(f"\n--- Speed Analysis ---\n")
                f.write(f"Mean Reference Speed: {np.mean(ref_speeds):.2f} m/s\n")
                f.write(f"Mean Actual Speed: {np.mean(actual_speeds):.2f} m/s\n")
                f.write(f"Mean Speed Difference: {np.mean(speed_diff):.2f} m/s\n")
                f.write(f"Speed Tracking RMS: {np.sqrt(np.mean(speed_diff**2)):.2f} m/s\n")

        print(f"\nReport saved to: {report_file}")
        return report_file

    def create_visualizations(self, cte, ref_speeds, actual_speeds, output_dir, timestamp):
        """시각화 생성"""

        # 1. 경로 비교 플롯
        plt.figure(figsize=(15, 10))

        # 서브플롯 1: 경로 비교
        plt.subplot(2, 2, 1)
        plt.plot(self.reference_path['x'], self.reference_path['y'], 'b-', label='Reference Path', linewidth=2)
        plt.plot(self.actual_path['x'], self.actual_path['y'], 'r-', label='Actual Path', linewidth=2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Path Comparison')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        # 서브플롯 2: Cross-track error
        if cte is not None:
            plt.subplot(2, 2, 2)
            plt.plot(cte, 'g-', linewidth=1)
            plt.xlabel('Waypoint Index')
            plt.ylabel('Cross-Track Error (m)')
            plt.title(f'Cross-Track Error (RMS: {np.sqrt(np.mean(cte**2)):.3f}m)')
            plt.grid(True)

        # 서브플롯 3: 속도 비교
        if ref_speeds is not None and actual_speeds is not None:
            plt.subplot(2, 2, 3)
            time_axis = np.arange(len(ref_speeds)) * 0.1  # 0.1초 간격 가정
            plt.plot(time_axis, ref_speeds, 'b-', label='Reference Speed', linewidth=2)
            plt.plot(time_axis, actual_speeds, 'r-', label='Actual Speed', linewidth=2)
            plt.xlabel('Time (s)')
            plt.ylabel('Speed (m/s)')
            plt.title('Speed Comparison')
            plt.legend()
            plt.grid(True)

        # 서브플롯 4: 속도 오차
        if ref_speeds is not None and actual_speeds is not None:
            plt.subplot(2, 2, 4)
            speed_error = actual_speeds - ref_speeds
            time_axis = np.arange(len(speed_error)) * 0.1
            plt.plot(time_axis, speed_error, 'purple', linewidth=1)
            plt.xlabel('Time (s)')
            plt.ylabel('Speed Error (m/s)')
            plt.title(f'Speed Tracking Error (RMS: {np.sqrt(np.mean(speed_error**2)):.3f}m/s)')
            plt.grid(True)

        plt.tight_layout()
        plot_file = os.path.join(output_dir, f"performance_analysis_{timestamp}.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Visualization saved to: {plot_file}")

def main():
    parser = argparse.ArgumentParser(description='Pure Pursuit Performance Analyzer')
    parser.add_argument('--record', action='store_true', help='Record actual path during driving')
    parser.add_argument('--analyze', nargs=2, metavar=('reference_path', 'actual_path'),
                       help='Analyze performance by comparing reference and actual paths')
    parser.add_argument('--time', type=int, default=60, help='Recording time in seconds (default: 60)')

    args = parser.parse_args()

    if args.record:
        # 경로 기록 모드
        rclpy.init()
        recorder = PathRecorder()

        print(f"Recording path for {args.time} seconds...")
        print("Press Ctrl+C to stop recording early")

        try:
            import time
            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time) < args.time:
                rclpy.spin_once(recorder, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\nRecording stopped by user")

        # 경로 저장
        saved_file = recorder.save_path()
        print(f"\nRecording completed. Path saved to: {saved_file}")

        recorder.destroy_node()
        rclpy.shutdown()

    elif args.analyze:
        # 분석 모드
        reference_path, actual_path = args.analyze

        analyzer = PathAnalyzer()

        if analyzer.load_reference_path(reference_path) and analyzer.load_actual_path(actual_path):
            analyzer.generate_performance_report()
        else:
            print("Failed to load path files")

    else:
        parser.print_help()

if __name__ == '__main__':
    main()