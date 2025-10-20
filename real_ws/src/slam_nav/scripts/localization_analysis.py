#!/usr/bin/env python3

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

        print(f"\n📊 사용 가능한 세션들 ({len(sessions)}개):")
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
        print(f"\n📈 세션 {session_id} 분석 중...")

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
        print(f"\n🔄 세션 비교: {', '.join(session_ids)}")

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
        print(f"\n📋 세션 {session_id} 요약 통계:")
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
                    print(f"\n{name}:")
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
