#!/usr/bin/env python3

"""
로컬라이제이션 상태 벡터 분석 도구

사용법:
python3 state_vector_analysis.py --file localization_states_20250920_151700.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime
import yaml
from PIL import Image
import tkinter as tk
from tkinter import filedialog

class StateVectorAnalyzer:
    def __init__(self, csv_file, map_yaml=None):
        self.csv_file = csv_file
        self.map_yaml = map_yaml
        self.df = None
        self.map_data = None
        self.map_info = None

    def load_data(self):
        """CSV 데이터 로드"""
        if not os.path.exists(self.csv_file):
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {self.csv_file}")

        self.df = pd.read_csv(self.csv_file)
        self.df['datetime'] = pd.to_datetime(self.df['datetime'])

        # 파일 타입 확인 및 컬럼 매핑
        if 'x' not in self.df.columns and 'current_x' in self.df.columns:
            print("⚠️  Performance 파일 감지됨. 컬럼 매핑을 적용합니다...")
            self.df['x'] = self.df['current_x']
            self.df['y'] = self.df['current_y']
            if 'current_theta' in self.df.columns:
                self.df['yaw'] = self.df['current_theta']

            # uncertainty 컬럼 매핑
            if 'uncertainty_theta' in self.df.columns and 'uncertainty_yaw' not in self.df.columns:
                self.df['uncertainty_yaw'] = self.df['uncertainty_theta']

            # 없는 컬럼들 기본값으로 채우기
            missing_cols = {
                'uncertainty_yaw': 0.0,
                'cov_xx': 0.0,
                'cov_yy': 0.0,
                'cov_yaw_yaw': 0.0,
                'cov_xy': 0.0
            }

            for col, default_val in missing_cols.items():
                if col not in self.df.columns:
                    self.df[col] = default_val

        elif 'x' not in self.df.columns:
            raise ValueError("❌ 올바른 localization CSV 파일이 아닙니다. 'x' 또는 'current_x' 컬럼이 필요합니다.")

        print(f"✅ 데이터 로드 완료: {len(self.df)}개 샘플")
        print(f"   시간 범위: {self.df['datetime'].iloc[0]} ~ {self.df['datetime'].iloc[-1]}")
        print(f"   지속 시간: {(self.df['datetime'].iloc[-1] - self.df['datetime'].iloc[0]).total_seconds():.1f}초")

        # 맵 데이터 로드
        if self.map_yaml:
            self.load_map_data()

    def load_map_data(self):
        """맵 데이터 로드"""
        try:
            # YAML 파일 읽기
            with open(self.map_yaml, 'r') as f:
                self.map_info = yaml.safe_load(f)

            # PGM 이미지 파일 경로
            if os.path.isabs(self.map_info['image']):
                pgm_path = self.map_info['image']
            else:
                pgm_path = os.path.join(os.path.dirname(self.map_yaml), self.map_info['image'])

            # PGM 이미지 로드
            map_image = Image.open(pgm_path)
            self.map_data = np.array(map_image)

            print(f"✅ 맵 데이터 로드 완료: {pgm_path}")
            print(f"   맵 크기: {self.map_data.shape}")
            print(f"   해상도: {self.map_info['resolution']} m/pixel")
            print(f"   원점: {self.map_info['origin']}")

        except Exception as e:
            print(f"❌ 맵 로드 실패: {e}")
            self.map_data = None
            self.map_info = None

    def world_to_map(self, x, y):
        """월드 좌표를 맵 픽셀 좌표로 변환"""
        if self.map_info is None:
            return None, None

        origin_x, origin_y = self.map_info['origin'][:2]
        resolution = self.map_info['resolution']

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        # 맵 이미지는 y축이 뒤집혀 있음
        map_y = self.map_data.shape[0] - map_y

        return map_x, map_y

    def analyze_trajectory(self):
        """궤적 분석"""
        print("\n📍 궤적 분석:")
        print("=" * 50)

        # 위치 범위
        x_min, x_max = self.df['x'].min(), self.df['x'].max()
        y_min, y_max = self.df['y'].min(), self.df['y'].max()

        print(f"X 범위: {x_min:.3f} ~ {x_max:.3f} m (범위: {x_max-x_min:.3f} m)")
        print(f"Y 범위: {y_min:.3f} ~ {y_max:.3f} m (범위: {y_max-y_min:.3f} m)")

        # 총 이동 거리 계산
        distances = np.sqrt(np.diff(self.df['x'])**2 + np.diff(self.df['y'])**2)
        total_distance = np.sum(distances)

        print(f"총 이동 거리: {total_distance:.3f} m")
        print(f"평균 이동 속도: {total_distance / (self.df['timestamp'].iloc[-1] - self.df['timestamp'].iloc[0]):.3f} m/s")

        # 각도 변화
        yaw_range = self.df['yaw'].max() - self.df['yaw'].min()
        print(f"Yaw 각도 범위: {np.degrees(yaw_range):.1f}°")

    def analyze_uncertainty(self):
        """불확실성 분석"""
        print("\n🎯 불확실성 분석:")
        print("=" * 50)

        # 불확실성 통계
        unc_stats = {
            'X 불확실성': self.df['uncertainty_x'],
            'Y 불확실성': self.df['uncertainty_y'],
            'Yaw 불확실성': self.df['uncertainty_yaw'],
            '총 위치 불확실성': self.df['uncertainty_total']
        }

        for name, data in unc_stats.items():
            print(f"{name}:")
            print(f"  평균: {data.mean():.4f} m")
            print(f"  표준편차: {data.std():.4f} m")
            print(f"  최대: {data.max():.4f} m")
            print(f"  최소: {data.min():.4f} m")
            print()

    def analyze_covariance(self):
        """공분산 매트릭스 분석"""
        print("\n📊 공분산 매트릭스 분석:")
        print("=" * 50)

        # 주요 공분산 요소들
        cov_elements = {
            'cov_xx': 'X-X 공분산',
            'cov_yy': 'Y-Y 공분산',
            'cov_yaw_yaw': 'Yaw-Yaw 공분산',
            'cov_xy': 'X-Y 교차 공분산'
        }

        for col, name in cov_elements.items():
            if col in self.df.columns:
                data = self.df[col]
                print(f"{name}:")
                print(f"  평균: {data.mean():.6f}")
                print(f"  표준편차: {data.std():.6f}")
                print(f"  범위: {data.min():.6f} ~ {data.max():.6f}")
                print()

    def create_visualizations(self, save_plots=True):
        """시각화 생성"""
        print("\n📈 시각화 생성 중...")

        # 1. 궤적 플롯
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # 궤적 with 맵 오버레이
        if self.map_data is not None and self.map_info is not None:
            # 맵 표시
            origin_x, origin_y = self.map_info['origin'][:2]
            resolution = self.map_info['resolution']

            # 맵의 월드 좌표 범위 계산
            map_width_world = self.map_data.shape[1] * resolution
            map_height_world = self.map_data.shape[0] * resolution

            extent = [origin_x, origin_x + map_width_world,
                     origin_y, origin_y + map_height_world]

            # 맵을 grayscale로 표시 (뒤집어서)
            ax1.imshow(np.flipud(self.map_data), extent=extent, cmap='gray', alpha=0.6, origin='lower')

        # 궤적 오버레이
        ax1.plot(self.df['x'], self.df['y'], 'b-', alpha=0.8, linewidth=2, label='Trajectory')
        ax1.scatter(self.df['x'].iloc[0], self.df['y'].iloc[0], color='green', s=150, label='Start', marker='o', edgecolors='white', linewidth=2)
        ax1.scatter(self.df['x'].iloc[-1], self.df['y'].iloc[-1], color='red', s=150, label='End', marker='x', linewidth=3)

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Robot Trajectory on Map')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 시간에 따른 위치
        relative_time = (self.df['datetime'] - self.df['datetime'].iloc[0]).dt.total_seconds()
        ax2.plot(relative_time, self.df['x'], label='X', alpha=0.8)
        ax2.plot(relative_time, self.df['y'], label='Y', alpha=0.8)
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Position (m)')
        ax2.set_title('Position vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # 불확실성 변화
        ax3.plot(relative_time, self.df['uncertainty_x'], label='X Uncertainty', alpha=0.8)
        ax3.plot(relative_time, self.df['uncertainty_y'], label='Y Uncertainty', alpha=0.8)
        ax3.plot(relative_time, self.df['uncertainty_total'], label='Total Uncertainty', alpha=0.8, linewidth=2)
        ax3.axhline(y=0.1, color='r', linestyle='--', alpha=0.5, label='Recommended Max (10cm)')
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Uncertainty (m)')
        ax3.set_title('Uncertainty vs Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Yaw 각도 변화
        ax4.plot(relative_time, np.degrees(self.df['yaw']), 'purple', alpha=0.8)
        ax4.set_xlabel('Time (seconds)')
        ax4.set_ylabel('Yaw Angle (degrees)')
        ax4.set_title('Orientation vs Time')
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_plots:
            # 파일명에서 확장자 제거하고 분석 결과 저장
            base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
            plot_file = os.path.join(os.path.dirname(self.csv_file), f"analysis_{base_name}.png")
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"📊 시각화 저장: {plot_file}")

        plt.show()

    def create_uncertainty_heatmap(self, save_plots=True):
        """불확실성 히트맵 생성"""
        print("\n🗺️  불확실성 히트맵 생성...")

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # 위치별 불확실성 스캐터 with 맵 오버레이
        if self.map_data is not None and self.map_info is not None:
            # 맵 표시
            origin_x, origin_y = self.map_info['origin'][:2]
            resolution = self.map_info['resolution']

            # 맵의 월드 좌표 범위 계산
            map_width_world = self.map_data.shape[1] * resolution
            map_height_world = self.map_data.shape[0] * resolution

            extent = [origin_x, origin_x + map_width_world,
                     origin_y, origin_y + map_height_world]

            # 맵을 grayscale로 표시
            ax1.imshow(np.flipud(self.map_data), extent=extent, cmap='gray', alpha=0.6, origin='lower')

        # 불확실성 스캐터 오버레이
        scatter1 = ax1.scatter(self.df['x'], self.df['y'], c=self.df['uncertainty_total'],
                             cmap='viridis', alpha=0.8, s=30, edgecolors='white', linewidth=0.5)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Total Uncertainty by Position on Map')
        plt.colorbar(scatter1, ax=ax1, label='Uncertainty (m)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 시간별 불확실성 히트맵 스타일
        relative_time = (self.df['datetime'] - self.df['datetime'].iloc[0]).dt.total_seconds()
        scatter2 = ax2.scatter(relative_time, self.df['uncertainty_total'],
                             c=self.df['uncertainty_total'], cmap='plasma', alpha=0.7, s=20)
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Total Uncertainty (m)')
        ax2.set_title('Uncertainty vs Time')
        plt.colorbar(scatter2, ax=ax2, label='Uncertainty (m)')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_plots:
            base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
            heatmap_file = os.path.join(os.path.dirname(self.csv_file), f"uncertainty_heatmap_{base_name}.png")
            plt.savefig(heatmap_file, dpi=300, bbox_inches='tight')
            print(f"🗺️  히트맵 저장: {heatmap_file}")

        plt.show()

    def analyze_stability(self):
        """안정성 분석"""
        print("\n⚖️  안정성 분석:")
        print("=" * 50)

        # 위치 변화의 표준편차 (안정성 지표)
        x_stability = self.df['x'].std()
        y_stability = self.df['y'].std()
        yaw_stability = self.df['yaw'].std()

        print(f"위치 안정성 (표준편차):")
        print(f"  X: {x_stability:.4f} m")
        print(f"  Y: {y_stability:.4f} m")
        print(f"  Yaw: {np.degrees(yaw_stability):.2f}°")

        # 불확실성의 변화율
        unc_change = np.abs(np.diff(self.df['uncertainty_total']))
        print(f"\n불확실성 변화율:")
        print(f"  평균 변화: {unc_change.mean():.6f} m/step")
        print(f"  최대 변화: {unc_change.max():.6f} m/step")

        # 급격한 변화 감지 (임계값 기반)
        position_jumps = np.sqrt(np.diff(self.df['x'])**2 + np.diff(self.df['y'])**2)
        large_jumps = position_jumps[position_jumps > 0.1]  # 10cm 이상 이동

        print(f"\n급격한 위치 변화 (>10cm):")
        print(f"  발생 횟수: {len(large_jumps)}회")
        if len(large_jumps) > 0:
            print(f"  최대 이동: {large_jumps.max():.3f} m")

    def generate_report(self):
        """전체 분석 리포트 생성"""
        print("\n" + "="*80)
        print("📋 로컬라이제이션 상태 벡터 분석 리포트")
        print("="*80)

        # 데이터 로드
        self.load_data()

        # 각종 분석 수행
        self.analyze_trajectory()
        self.analyze_uncertainty()
        self.analyze_covariance()
        self.analyze_stability()

        # 시각화
        self.create_visualizations()
        self.create_uncertainty_heatmap()

        print("\n✅ 분석 완료!")

def main():
    parser = argparse.ArgumentParser(description='로컬라이제이션 상태 벡터 분석')
    parser.add_argument('--file', help='분석할 CSV 파일 경로 (없으면 GUI로 선택)')
    parser.add_argument('--map', help='맵 YAML 파일 경로')
    parser.add_argument('--data-dir', default='/home/f1/f1tenth_ws/localization_logs',
                       help='데이터 디렉터리 경로')

    args = parser.parse_args()

    # CSV 파일 선택
    csv_file = args.file
    if not csv_file:
        try:
            print("📊 CSV 파일을 선택하세요...")

            # GUI 초기화 (숨김)
            root = tk.Tk()
            root.withdraw()

            # CSV 파일 선택 다이얼로그
            csv_file = filedialog.askopenfilename(
                title="분석할 CSV 파일 선택",
                initialdir=args.data_dir,
                filetypes=[
                    ("CSV files", "*.csv"),
                    ("All files", "*.*")
                ]
            )

            root.destroy()

            if not csv_file:
                print("❌ CSV 파일이 선택되지 않았습니다. 종료합니다.")
                return
            else:
                print(f"📊 선택된 CSV 파일: {csv_file}")

        except Exception as e:
            print(f"❌ GUI를 사용할 수 없습니다: {e}")
            print("💡 사용 가능한 CSV 파일들:")

            if os.path.exists(args.data_dir):
                print(f"\n📁 {args.data_dir}:")
                for file in os.listdir(args.data_dir):
                    if file.endswith('.csv'):
                        print(f"   - {file}")

            print("\n사용법: python3 state_vector_analysis.py --file <csv_file>")
            return

    # 파일 경로 처리
    elif not os.path.isabs(csv_file):
        csv_file = os.path.join(args.data_dir, csv_file)

    # 맵 파일 선택
    map_yaml = args.map
    if not map_yaml:
        try:
            print("🗺️  맵 파일을 선택하세요...")

            # GUI 초기화 (숨김)
            root = tk.Tk()
            root.withdraw()

            # 맵 디렉터리들 확인
            possible_map_dirs = [
                '/home/f1/f1tenth_ws/joon_path_generate/maps',
                '/home/f1/f1tenth_ws/path_generate/steven_gong/Raceline-Optimization/maps',
                '/home/f1/f1tenth_ws',
                os.path.expanduser('~')
            ]

            initial_dir = '/home/f1/f1tenth_ws'
            for map_dir in possible_map_dirs:
                if os.path.exists(map_dir):
                    initial_dir = map_dir
                    break

            # 파일 선택 다이얼로그
            print(f"📁 시작 디렉터리: {initial_dir}")
            map_yaml = filedialog.askopenfilename(
                title="맵 YAML 파일 선택",
                initialdir=initial_dir,
                filetypes=[
                    ("YAML files", "*.yaml"),
                    ("YML files", "*.yml"),
                    ("All files", "*.*")
                ]
            )

            root.destroy()

            if not map_yaml:
                print("❌ 맵 파일이 선택되지 않았습니다. 맵 없이 분석을 진행합니다.")
            else:
                print(f"📍 선택된 맵: {map_yaml}")

        except Exception as e:
            print(f"❌ GUI를 사용할 수 없습니다: {e}")
            print("💡 사용 가능한 맵 파일들:")

            # 맵 파일 목록 표시
            map_dirs = [
                '/home/f1/f1tenth_ws/joon_path_generate/maps',
                '/home/f1/f1tenth_ws/path_generate/steven_gong/Raceline-Optimization/maps'
            ]

            for map_dir in map_dirs:
                if os.path.exists(map_dir):
                    print(f"\n📁 {map_dir}:")
                    for file in os.listdir(map_dir):
                        if file.endswith(('.yaml', '.yml')):
                            print(f"   - {file}")

            print("\n사용법: python3 state_vector_analysis.py --file <csv_file> --map <map_yaml_file>")
            map_yaml = None

    analyzer = StateVectorAnalyzer(csv_file, map_yaml)
    analyzer.generate_report()

if __name__ == '__main__':
    main()