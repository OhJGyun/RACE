#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optimized LaserScan → map 변환 → 초경량 클러스터링 → 중심만 RViz 시각화
+ CSV로 읽은 outer/inner 경계 사이(outer 안 ∧ inner 밖)에 있는 중심만 통과

Key Optimizations:
1. Index-based FOV filtering (no angle normalization in loop)
2. NumPy vectorized TF transformation
3. Config-based parameters
4. Optional size-based filtering
"""

from __future__ import annotations
import os, csv, math
from typing import List, Tuple, Sequence, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclTime

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from transforms3d.quaternions import quat2mat


# -------------------------------
# 타입
# -------------------------------
Point2D = Tuple[float, float]

# -------------------------------
# CSV 유틸
# -------------------------------
def load_world_csv(path: str) -> List[Point2D]:
    """CSV에서 (x,y) 목록 로드. 필요시 폐곡선으로 닫음."""
    pts: List[Point2D] = []
    path = os.path.expanduser(path)
    if not path or not os.path.exists(path):
        return pts
    with open(path, "r") as f:
        rd = csv.reader(f)
        for row in rd:
            if len(row) < 2:
                continue
            try:
                x = float(row[0]); y = float(row[1])
            except ValueError:
                continue
            pts.append((x, y))
    if len(pts) >= 3 and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


# -------------------------------
# 폴리곤 판정
# -------------------------------
def point_in_polygon(x: float, y: float, poly: Sequence[Point2D], include_boundary: bool = True) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        if (y1 > y) != (y0 > y):
            t = (y - y0) / (y1 - y0 + 1e-12)
            xin = x0 + (x1 - x0) * t
            cmp = (x <= xin) if include_boundary else (x < xin)
            if cmp:
                inside = not inside
        x0, y0 = x1, y1
    return inside


def in_ring(x: float, y: float, outer: Optional[Sequence[Point2D]], inner: Optional[Sequence[Point2D]]) -> bool:
    """outer 안 AND inner 밖이면 True (inner가 없으면 무시)."""
    if outer and not point_in_polygon(x, y, outer, True):
        return False
    if inner and point_in_polygon(x, y, inner, True):
        return False
    return True


# -------------------------------
# 초경량 스캔라인 기반 클러스터링 (O(N))
# -------------------------------
def fast_scanline_clusters(r_list, ang_inc, original_indices=None, min_samples=4, max_samples=50, eps0=0.12, k=0.06, max_index_gap=2):
    """
    연속한 빔 간 거리로 클러스터링 (정렬된 스캔 전제)

    Args:
        r_list: 거리 리스트
        ang_inc: 연속된 빔 간 각도 간격 (라디안, 샘플링 반영됨)
        original_indices: 원본 스캔에서의 인덱스 (선택, 연속성 체크용)
        min_samples: 최소 클러스터 크기
        max_samples: 최대 클러스터 크기 (큰 클러스터는 벽면/경계선으로 간주)
        eps0: 최소 연결 임계 (m)
        k: 거리 비례 계수
    """
    n = len(r_list)
    if n == 0:
        return []

    # 최적화: 연속된 빔 간 각도가 일정하므로 cos(ang_inc) 한 번만 계산
    cos_ang_inc = math.cos(ang_inc)

    clusters = []
    cur = [0]

    for i in range(1, n):
        # 최적화: 원본 인덱스가 연속적이지 않으면 다른 클러스터
        if original_indices is not None:
            idx_gap = original_indices[i] - original_indices[i - 1]
            # allow gaps up to max_index_gap (handles down-sampling)
            if idx_gap > max_index_gap:
                # 인덱스 비연속 → 무조건 새 클러스터
                # min/max 범위 체크
                if min_samples <= len(cur) <= max_samples:
                    clusters.append(cur)
                cur = [i]
                continue

        # 거리 기반 클러스터링
        r0, r1 = r_list[i - 1], r_list[i]
        # 최적화: cos(ang_inc) 재사용 (연속된 빔 간 각도는 일정)
        dij = math.sqrt(r0 * r0 + r1 * r1 - 2.0 * r0 * r1 * cos_ang_inc)
        link = max(eps0, k * min(r0, r1))

        if dij <= link:
            cur.append(i)
        else:
            # min/max 범위 체크
            if min_samples <= len(cur) <= max_samples:
                clusters.append(cur)
            cur = [i]

    # 마지막 클러스터 처리 (min/max 범위 체크)
    if min_samples <= len(cur) <= max_samples:
        clusters.append(cur)

    return clusters


# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    """
    Optimized LaserScan → map 변환 → 초경량 클러스터링 → 중심 표시
    + CSV outer/inner 링 필터
    """

    def __init__(self):
        super().__init__("simple_scan_viz")

        # 파라미터 선언
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("min_samples", 8)
        self.declare_parameter("roi_min_dist", 0.00)
        self.declare_parameter("roi_max_dist", 3.00)
        self.declare_parameter("center_scale", 0.12)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("fov_center_deg", 0.0)
        self.declare_parameter("outer_bound_csv", "/home/ircv7/RACE/bound/1031_1/outer_bound.csv")
        self.declare_parameter("inner_bound_csv", "/home/ircv7/RACE/bound/1031_1/inner_bound.csv")

        # New parameters
        self.declare_parameter("cluster_eps0", 0.12)
        self.declare_parameter("cluster_k", 0.06)
        self.declare_parameter("max_samples", 50)
        self.declare_parameter("max_obstacle_size", 0.5)
        self.declare_parameter("min_obstacle_size", 0.0)
        self.declare_parameter("use_vectorized_tf", True)
        self.declare_parameter("enable_size_filter", False)
        self.declare_parameter("sample_stride", 2)

        # 파라미터 로드
        self.scan_topic = self.get_parameter("scan_topic").value
        self.marker_frame = self.get_parameter("marker_frame_id").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.roi_min_dist = float(self.get_parameter("roi_min_dist").value)
        self.roi_max_dist = float(self.get_parameter("roi_max_dist").value)
        self.center_scale = float(self.get_parameter("center_scale").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.fov_center_deg = float(self.get_parameter("fov_center_deg").value)
        self.outer_csv = self.get_parameter("outer_bound_csv").value
        self.inner_csv = self.get_parameter("inner_bound_csv").value

        # New parameters
        self.cluster_eps0 = float(self.get_parameter("cluster_eps0").value)
        self.cluster_k = float(self.get_parameter("cluster_k").value)
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.max_obstacle_size = float(self.get_parameter("max_obstacle_size").value)
        self.min_obstacle_size = float(self.get_parameter("min_obstacle_size").value)
        self.use_vectorized_tf = bool(self.get_parameter("use_vectorized_tf").value)
        self.enable_size_filter = bool(self.get_parameter("enable_size_filter").value)
        self.sample_stride = int(self.get_parameter("sample_stride").value)

        # 경계 로드
        self.outer_poly = load_world_csv(self.outer_csv) if self.outer_csv else []
        self.inner_poly = load_world_csv(self.inner_csv) if self.inner_csv else []
        if self.outer_poly:
            self.get_logger().info(f"[bounds] outer: {len(self.outer_poly)} pts")
        if self.inner_poly:
            self.get_logger().info(f"[bounds] inner: {len(self.inner_poly)} pts")

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)
        self.pub_obstacles = self.create_publisher(PoseArray, "scan_viz/obstacles", 10)
        self.pub_debug = self.create_publisher(MarkerArray, "scan_viz/debug_markers", 10)
        self.pub_scan_debug = self.create_publisher(MarkerArray, "scan_viz/scan_process_debug", 10)

        # FOV 인덱스 (첫 스캔에서 한 번만 계산)
        self.fov_idx_min = None
        self.fov_idx_max = None
        self.fov_indices_computed = False

        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}]m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°, "
            f"cluster_size=[{self.min_samples},{self.max_samples}], "
            f"vectorized_tf={self.use_vectorized_tf}, "
            f"size_filter={self.enable_size_filter}, "
            f"sample_stride={self.sample_stride}"
        )

    # ---------------------------
    # TF lookup
    # ---------------------------
    def _lookup_latest(self, target_frame: str, source_frame: str):
        tf = self.tf_buffer.lookup_transform(
            target_frame, source_frame, RclTime(),
            timeout=Duration(seconds=self.tf_timeout)
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T

    # ---------------------------
    # 퍼블리시 헬퍼
    # ---------------------------
    def _publish_clear(self):
        arr = MarkerArray()
        m = Marker(); m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub.publish(arr)

        # Clear debug markers too
        debug_arr = MarkerArray()
        m_debug = Marker(); m_debug.action = Marker.DELETEALL
        debug_arr.markers.append(m_debug)
        self.pub_debug.publish(debug_arr)

    def _publish_debug_centers(self, all_centers: List[Point], filtered_centers: List[Point]):
        """
        Publish debug visualization with different colors:
        - RED: All detected cluster centers (before in_ring filter)
        - GREEN: Centers that passed in_ring filter
        - BLUE: Centers that were filtered out
        """
        debug_arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Convert to sets for comparison
        filtered_set = set((p.x, p.y) for p in filtered_centers)

        # 1. All centers (RED) - before filter
        m_all = Marker()
        m_all.header.frame_id = self.marker_frame
        m_all.header.stamp = now
        m_all.ns = "debug_all_centers"
        m_all.id = 0
        m_all.type = Marker.SPHERE_LIST
        m_all.action = Marker.ADD
        m_all.pose.orientation.w = 1.0
        m_all.scale.x = 0.20
        m_all.scale.y = 0.20
        m_all.scale.z = 0.20
        m_all.color.r = 1.0  # RED
        m_all.color.g = 0.0
        m_all.color.b = 0.0
        m_all.color.a = 0.5
        m_all.lifetime = Duration(seconds=0.5).to_msg()
        m_all.points.extend(all_centers)
        debug_arr.markers.append(m_all)

        # 2. Filtered out centers (BLUE)
        m_rejected = Marker()
        m_rejected.header.frame_id = self.marker_frame
        m_rejected.header.stamp = now
        m_rejected.ns = "debug_rejected"
        m_rejected.id = 1
        m_rejected.type = Marker.SPHERE_LIST
        m_rejected.action = Marker.ADD
        m_rejected.pose.orientation.w = 1.0
        m_rejected.scale.x = 0.25
        m_rejected.scale.y = 0.25
        m_rejected.scale.z = 0.25
        m_rejected.color.r = 0.0
        m_rejected.color.g = 0.0
        m_rejected.color.b = 1.0  # BLUE
        m_rejected.color.a = 0.8
        m_rejected.lifetime = Duration(seconds=0.5).to_msg()

        rejected_points = [p for p in all_centers if (p.x, p.y) not in filtered_set]
        m_rejected.points.extend(rejected_points)
        debug_arr.markers.append(m_rejected)

        # 3. Accepted centers (GREEN)
        m_accepted = Marker()
        m_accepted.header.frame_id = self.marker_frame
        m_accepted.header.stamp = now
        m_accepted.ns = "debug_accepted"
        m_accepted.id = 2
        m_accepted.type = Marker.SPHERE_LIST
        m_accepted.action = Marker.ADD
        m_accepted.pose.orientation.w = 1.0
        m_accepted.scale.x = 0.23
        m_accepted.scale.y = 0.23
        m_accepted.scale.z = 0.23
        m_accepted.color.r = 0.0
        m_accepted.color.g = 1.0  # GREEN
        m_accepted.color.b = 0.0
        m_accepted.color.a = 1.0
        m_accepted.lifetime = Duration(seconds=0.5).to_msg()
        m_accepted.points.extend(filtered_centers)
        debug_arr.markers.append(m_accepted)

        self.pub_debug.publish(debug_arr)

    def _publish_scan_process_debug(self,
                                     all_points: List[Point],
                                     fov_points: List[Point],
                                     sampled_points: List[Point],
                                     clustered_points: List[Point]):
        """
        스캔 처리 과정 디버그 시각화
        - GRAY: 전체 스캔 포인트 (ROI 필터링 후)
        - CYAN: FOV 필터링 후 포인트
        - MAGENTA: 샘플링 후 포인트
        - ORANGE: 클러스터링된 포인트
        """
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # GRAY: 전체 스캔 포인트 (ROI 필터 후)
        m_all = Marker()
        m_all.header.frame_id = self.marker_frame
        m_all.header.stamp = now
        m_all.ns = "scan_all_points"
        m_all.id = 0
        m_all.type = Marker.POINTS
        m_all.action = Marker.ADD
        m_all.pose.orientation.w = 1.0
        m_all.scale.x = 0.03
        m_all.scale.y = 0.03
        m_all.color.r = 0.5
        m_all.color.g = 0.5
        m_all.color.b = 0.5
        m_all.color.a = 0.3
        m_all.lifetime = Duration(seconds=0.5).to_msg()
        m_all.points.extend(all_points)
        arr.markers.append(m_all)

        # CYAN: FOV 필터링 후
        m_fov = Marker()
        m_fov.header.frame_id = self.marker_frame
        m_fov.header.stamp = now
        m_fov.ns = "scan_fov_filtered"
        m_fov.id = 1
        m_fov.type = Marker.POINTS
        m_fov.action = Marker.ADD
        m_fov.pose.orientation.w = 1.0
        m_fov.scale.x = 0.04
        m_fov.scale.y = 0.04
        m_fov.color.r = 0.0
        m_fov.color.g = 1.0
        m_fov.color.b = 1.0
        m_fov.color.a = 0.5
        m_fov.lifetime = Duration(seconds=0.5).to_msg()
        m_fov.points.extend(fov_points)
        arr.markers.append(m_fov)

        # MAGENTA: 샘플링 후
        m_sampled = Marker()
        m_sampled.header.frame_id = self.marker_frame
        m_sampled.header.stamp = now
        m_sampled.ns = "scan_sampled"
        m_sampled.id = 2
        m_sampled.type = Marker.POINTS
        m_sampled.action = Marker.ADD
        m_sampled.pose.orientation.w = 1.0
        m_sampled.scale.x = 0.05
        m_sampled.scale.y = 0.05
        m_sampled.color.r = 1.0
        m_sampled.color.g = 0.0
        m_sampled.color.b = 1.0
        m_sampled.color.a = 0.7
        m_sampled.lifetime = Duration(seconds=0.5).to_msg()
        m_sampled.points.extend(sampled_points)
        arr.markers.append(m_sampled)

        # ORANGE: 클러스터링된 포인트
        m_clustered = Marker()
        m_clustered.header.frame_id = self.marker_frame
        m_clustered.header.stamp = now
        m_clustered.ns = "scan_clustered"
        m_clustered.id = 3
        m_clustered.type = Marker.POINTS
        m_clustered.action = Marker.ADD
        m_clustered.pose.orientation.w = 1.0
        m_clustered.scale.x = 0.06
        m_clustered.scale.y = 0.06
        m_clustered.color.r = 1.0
        m_clustered.color.g = 0.5
        m_clustered.color.b = 0.0
        m_clustered.color.a = 0.9
        m_clustered.lifetime = Duration(seconds=0.5).to_msg()
        m_clustered.points.extend(clustered_points)
        arr.markers.append(m_clustered)

        self.pub_scan_debug.publish(arr)

    def _publish_centers(self, centers: List[Point]):
        # MarkerArray for visualization
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cluster_centers"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.center_scale
        m.scale.y = self.center_scale
        m.scale.z = self.center_scale
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = Duration(seconds=0.5).to_msg()
        m.points.extend(centers)
        arr.markers.append(m)
        self.pub.publish(arr)

        # PoseArray for lane_selector
        pose_arr = PoseArray()
        pose_arr.header.frame_id = self.marker_frame
        pose_arr.header.stamp = self.get_clock().now().to_msg()
        for pt in centers:
            pose = Pose()
            pose.position.x = pt.x
            pose.position.y = pt.y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_arr.poses.append(pose)
        self.pub_obstacles.publish(pose_arr)

    # ---------------------------
    # FOV 인덱스 계산 (최적화: 각도 범위 → 인덱스 범위)
    # ---------------------------
    def _compute_fov_indices(self, ang_min: float, ang_inc: float, n_ranges: int):
        """
        FOV 각도 범위를 인덱스 범위로 미리 계산

        Returns:
            (idx_min, idx_max): 유효한 인덱스 범위 [idx_min, idx_max)
        """
        fov_rad = math.radians(self.fov_deg)
        fov_center = math.radians(self.fov_center_deg)
        half_fov = 0.5 * fov_rad

        # FOV 범위 계산
        angle_min_fov = fov_center - half_fov
        angle_max_fov = fov_center + half_fov

        # 인덱스로 변환
        idx_min = max(0, int((angle_min_fov - ang_min) / ang_inc))
        idx_max = min(n_ranges, int((angle_max_fov - ang_min) / ang_inc) + 1)

        return idx_min, idx_max

    # ---------------------------
    # 스캔 콜백 (최적화 버전)
    # ---------------------------
    def _on_scan(self, scan: LaserScan):
        laser_frame = scan.header.frame_id or "laser"
        try:
            R_ml, T_ml = self._lookup_latest(self.marker_frame, laser_frame)
        except TransformException as e:
            self.get_logger().warn(f"TF not available: {e}")
            return

        ang_min, ang_inc = scan.angle_min, scan.angle_increment
        rmin, rmax = scan.range_min, scan.range_max
        ranges = np.array(scan.ranges)
        n_ranges = len(ranges)

        if n_ranges == 0 or ang_inc == 0.0:
            return

        # FOV 인덱스 계산 (첫 스캔에서만 한 번)
        if not self.fov_indices_computed:
            self.fov_idx_min, self.fov_idx_max = self._compute_fov_indices(ang_min, ang_inc, n_ranges)
            self.fov_indices_computed = True
            self.get_logger().info(
                f"[FOV] Computed indices: [{self.fov_idx_min}, {self.fov_idx_max}) "
                f"out of {n_ranges} total beams"
            )

        # ==========================================
        # 1) ROI/FOV 필터링 (최적화: 인덱스 기반)
        # ==========================================

        # 디버그: 전체 스캔 포인트 수집 (ROI 필터 전, laser frame)
        all_angles = ang_min + np.arange(n_ranges) * ang_inc
        all_valid_mask = ~(np.isnan(ranges) | np.isinf(ranges))
        all_valid_mask &= (ranges >= rmin) & (ranges <= rmax)
        all_valid_indices = np.where(all_valid_mask)[0]

        # FOV 인덱스 범위로 슬라이싱
        idx_start = self.fov_idx_min
        idx_end = self.fov_idx_max

        ranges_fov = ranges[idx_start:idx_end]

        # Valid range mask (NaN, Inf, rmin, rmax 체크)
        valid_mask = ~(np.isnan(ranges_fov) | np.isinf(ranges_fov))
        valid_mask &= (ranges_fov >= rmin) & (ranges_fov <= rmax)

        # ROI distance mask
        use_roi = self.roi_max_dist > self.roi_min_dist
        if use_roi:
            valid_mask &= (ranges_fov >= self.roi_min_dist) & (ranges_fov <= self.roi_max_dist)

        # 유효한 인덱스 추출
        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            self._publish_clear()
            return

        # 디버그: FOV 필터링 후 인덱스 (전역 인덱스)
        fov_global_indices = valid_indices + idx_start

        # 샘플링 (stride 적용)
        sampled_indices = valid_indices
        if self.sample_stride > 1:
            sampled_indices = valid_indices[::self.sample_stride]
            if len(sampled_indices) == 0:
                self._publish_clear()
                return

        # 디버그: 샘플링 후 인덱스 (전역 인덱스)
        sampled_global_indices = sampled_indices + idx_start

        # 필터링된 거리
        r_list = ranges_fov[sampled_indices].tolist()

        # 각도 계산 (FOV 범위 내에서만)
        angles_fov = ang_min + np.arange(idx_start, idx_end) * ang_inc
        th_list = angles_fov[sampled_indices].tolist()

        # 원본 인덱스 추적 (연속성 체크용)
        original_indices = valid_indices.tolist()

        # 샘플링을 고려한 유효 각도 간격 및 클러스터 파라미터 계산
        # sample_stride=2이면 연속된 빔 간 각도는 ang_inc × 2
        effective_ang_inc = ang_inc * self.sample_stride

        # 샘플링 비율에 따라 클러스터링 임계값도 조정
        # stride가 클수록 빔 간 거리가 멀어지므로 eps0도 비례하여 증가
        effective_eps0 = self.cluster_eps0 * self.sample_stride
        effective_k = self.cluster_k * self.sample_stride

        # ==========================================
        # 2) 초경량 클러스터링 (크기 필터링 포함)
        # ==========================================
        idx_clusters = fast_scanline_clusters(
            r_list,
            ang_inc=effective_ang_inc,
            original_indices=original_indices,
            min_samples=self.min_samples,
            max_samples=self.max_samples,
            eps0=effective_eps0,
            k=effective_k,
            max_index_gap=self.sample_stride
        )

        if len(idx_clusters) == 0:
            self._publish_clear()
            return

        # ==========================================
        # 3) 중심 계산 및 map 변환 (최적화: 벡터화)
        # ==========================================

        if self.use_vectorized_tf:
            # 벡터화 버전: 전체 스캔을 한 번에 변환
            angles = np.array(th_list)
            ranges_arr = np.array(r_list)

            # Cartesian 좌표 (laser frame)
            x_laser = ranges_arr * np.cos(angles)
            y_laser = ranges_arr * np.sin(angles)
            z_laser = np.zeros_like(ranges_arr)

            # 한 번에 map frame으로 변환
            xyz_laser = np.vstack([x_laser, y_laser, z_laser])
            xyz_map = R_ml @ xyz_laser + T_ml[:, np.newaxis]

            # 디버그: 전체 스캔을 map frame으로 변환
            all_angles_valid = all_angles[all_valid_indices]
            all_ranges_valid = ranges[all_valid_indices]
            all_x_laser = all_ranges_valid * np.cos(all_angles_valid)
            all_y_laser = all_ranges_valid * np.sin(all_angles_valid)
            all_z_laser = np.zeros_like(all_ranges_valid)
            all_xyz_laser = np.vstack([all_x_laser, all_y_laser, all_z_laser])
            all_xyz_map = R_ml @ all_xyz_laser + T_ml[:, np.newaxis]

            # 디버그: FOV 필터링 후 포인트
            fov_angles = all_angles[fov_global_indices]
            fov_ranges = ranges[fov_global_indices]
            fov_x_laser = fov_ranges * np.cos(fov_angles)
            fov_y_laser = fov_ranges * np.sin(fov_angles)
            fov_z_laser = np.zeros_like(fov_ranges)
            fov_xyz_laser = np.vstack([fov_x_laser, fov_y_laser, fov_z_laser])
            fov_xyz_map = R_ml @ fov_xyz_laser + T_ml[:, np.newaxis]

            # 디버그: 샘플링 후 포인트 (이미 xyz_map에 있음)
            sampled_xyz_map = xyz_map

            # 디버그: 클러스터링된 포인트만 수집
            clustered_indices = []
            for idxs in idx_clusters:
                clustered_indices.extend(idxs)
            clustered_xyz_map = xyz_map[:, clustered_indices]

            # 디버그 포인트 리스트 생성
            all_points_debug = [Point(x=all_xyz_map[0, i], y=all_xyz_map[1, i], z=0.0)
                               for i in range(all_xyz_map.shape[1])]
            fov_points_debug = [Point(x=fov_xyz_map[0, i], y=fov_xyz_map[1, i], z=0.0)
                               for i in range(fov_xyz_map.shape[1])]
            sampled_points_debug = [Point(x=sampled_xyz_map[0, i], y=sampled_xyz_map[1, i], z=0.0)
                                   for i in range(sampled_xyz_map.shape[1])]
            clustered_points_debug = [Point(x=clustered_xyz_map[0, i], y=clustered_xyz_map[1, i], z=0.0)
                                     for i in range(clustered_xyz_map.shape[1])]

            # 클러스터별 중심 계산
            centers = []
            obstacle_sizes = []

            for idxs in idx_clusters:
                cluster_x = xyz_map[0, idxs]
                cluster_y = xyz_map[1, idxs]

                # 중심
                center_x = np.mean(cluster_x)
                center_y = np.mean(cluster_y)

                # Size 계산 (optional)
                if self.enable_size_filter:
                    x_min, x_max = np.min(cluster_x), np.max(cluster_x)
                    y_min, y_max = np.min(cluster_y), np.max(cluster_y)
                    size = max(x_max - x_min, y_max - y_min)

                    # Size 필터링
                    if size > self.max_obstacle_size or size < self.min_obstacle_size:
                        continue

                    obstacle_sizes.append(size)

                centers.append(Point(x=center_x, y=center_y, z=0.0))

        else:
            # 기존 버전: 개별 변환 (호환성 유지)
            # 디버그 포인트는 벡터화 모드에서만 지원
            all_points_debug = []
            fov_points_debug = []
            sampled_points_debug = []
            clustered_points_debug = []

            centers = []
            for idxs in idx_clusters:
                sx = sy = 0.0
                for j in idxs:
                    r, th = r_list[j], th_list[j]
                    p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0])
                    p_m = R_ml @ p_l + T_ml
                    sx += p_m[0]; sy += p_m[1]
                n = len(idxs)
                centers.append(Point(x=sx/n, y=sy/n, z=0.0))

        # ==========================================
        # 4) CSV 링 필터
        # ==========================================
        centers_ring = [p for p in centers if in_ring(p.x, p.y,
                                                      self.outer_poly or None,
                                                      self.inner_poly or None)]

        # ==========================================
        # 4.5) 디버그 시각화 퍼블리시
        # ==========================================
        # Always publish debug visualization to see filtering effect
        self._publish_debug_centers(centers, centers_ring)

        # 4.6) 스캔 처리 과정 디버그 시각화 (벡터화 모드에서만)
        if self.use_vectorized_tf:
            self._publish_scan_process_debug(all_points_debug, fov_points_debug,
                                            sampled_points_debug, clustered_points_debug)

        # ==========================================
        # 5) 로그 및 퍼블리시
        # ==========================================
        if centers_ring:
            # 변화가 있을 때만 로그 (성능 최적화)
            if not hasattr(self, '_prev_obstacle_count') or \
               len(centers_ring) != self._prev_obstacle_count:
                self.get_logger().info(
                    f"[OBSTACLE DETECTED] {len(centers_ring)} obstacle(s) in ring "
                    f"(total detected: {len(centers)}, filtered out: {len(centers) - len(centers_ring)})"
                )
                self._prev_obstacle_count = len(centers_ring)

            self._publish_centers(centers_ring)
        else:
            if not hasattr(self, '_prev_obstacle_count') or self._prev_obstacle_count != 0:
                self.get_logger().info(f"[CLEAR] No obstacles in ring (total detected: {len(centers)})")
                self._prev_obstacle_count = 0
            self._publish_clear()


# -------------------------------
# main
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SimpleScanViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()