# launch/bound_detector.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bound_obstacle_detector',      # 패키지명
            executable='bound_detector',            # 실행 파일(엔트리포인트)명 (새 코드가 이 엔트리로 빌드되어 있어야 함)
            name='bound_obstacle_detector',
            output='screen',
            parameters=[{
                # --- 토픽 ---
                'scan_topic': '/scan',

                # --- TF / 프레임 설정 ---
                # CSV는 map 프레임 좌표라고 가정
                'map_frame_id': 'map',
                # LiDAR 실제 frame_id (빈 문자열이면 LaserScan.header.frame_id 사용)
                'laser_frame_id': '',
                'tf_timeout_sec': 0.05,

                # --- 경계 CSV (map 프레임) ---
                'outer_csv':  '/home/moon/sim_ws/bounds_out/outer_bound_world.csv',
                'inner_csvs': '/home/moon/sim_ws/bounds_out/inner_bound_world.csv',
                # 예) 여러 개: 'bounds_out/inner_*.csv' 또는 'a.csv,b.csv;c.csv'
                'include_boundary': True,

                # --- 래스터 / CORE-RING 설정 ---
                # map 해상도만 주면 outer/inner bbox에 맞춰 래스터 자동 생성
                'map_resolution': 0.05,       # (기존 mask_resolution)
                'ring_width_m':   0.20,       # (기존 core_shrink_m) CORE를 이만큼 침식
                'padding_m':      1.0,        # 외곽 패딩(m), 필요시 조절

                # --- 스캔/ROI ---
                'roi_deg':   90.0,
                'range_max': 10.0,

                # --- 경계 마진 정책 ---
                # CORE에는 boundary_margin 사용,
                # RING에는 max(boundary_margin, ring_boundary_margin) 적용(더 보수적으로 컷하려면 ring_boundary_margin을 더 크게)
                'boundary_margin':      0.20,   # 기본 마진 (CORE)
                'ring_boundary_margin': 0.08,   # (기존 ring_small_margin). RING을 더 보수적으로 하려면 0.30~0.50로 키워봐.

                # --- 클러스터링 ---
                'cluster_epsilon':        0.28,  # (기존 euclid_eps)
                'cluster_min_points':     3,     # (기존 euclid_min_pts)
                'cluster_confirm_points': 6,     # 확정용 임계(현장서 5~8 권장)

                # --- 시각화 ---
                # RViz Fixed Frame을 'map'으로 둘 것을 권장
                'marker_frame_id': 'map',
                'marker_scale':    0.35,

                # --- (이전 파이프라인 전용 파라미터는 삭제/미사용) ---
                # 'odom_topic': ...                # 사용 안 함 (TF 사용)
                # 'mask_width': ..., 'mask_height': ..., 'mask_origin': ...  # 사용 안 함
                # 'laser_yaw_offset_deg': ...     # TF 캘리브레이션으로 해결 권장(미사용)
                # 'lane_block_half_width': ..., 'lane_block_dist': ...  # 새 코드에서 미사용
                # 'cluster_method': 'euclid'       # 미사용 (고정 유클리드)
                # 'free_mask_path': ..., 'map_yaml': ...  # 미사용
            }]
        )
    ])
