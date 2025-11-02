#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optimized LaserScan → map 변환 → 초경량 클러스터링 → 중심만 RViz 시각화
+ CSV로 읽은 outer/inner 경계 사이(outer 안 ∧ inner 밖)에 있는 중심만 통과
+ 소형 트래커 기반 정적/동적 판정
+ PoseArray로 출력할 때 z=0(정적), z=1(동적)

Key Optimizations:
1. Index-based FOV filtering (no angle normalization in loop)
2. NumPy vectorized TF transformation
3. Config-based parameters
4. Optional size-based filtering
5. Lightweight anchor-based tracker with EMA velocity
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


def in_ring(x: float, y: float,
            outer: Optional[Sequence[Point2D]],
            inner: Optional[Sequence[Point2D]]) -> bool:
    """outer 안 AND inner 밖이면 True (inner가 없으면 무시)."""
    if outer and not point_in_polygon(x, y, outer, True):
        return False
    if inner and point_in_polygon(x, y, inner, True):
        return False
    return True


# -------------------------------
# 초경량 스캔라인 기반 클러스터링 (O(N))
# -------------------------------
def fast_scanline_clusters(
    r_list,
    ang_inc,
    original_indices=None,
    min_samples=4,
    max_samples=50,
    eps0=0.12,
    k=0.06,
    max_index_gap=2
):
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
        # 인덱스가 크게 튀면 새 클러스터 시작
        if original_indices is not None:
            idx_gap = original_indices[i] - original_indices[i - 1]
            if idx_gap > max_index_gap:
                if min_samples <= len(cur) <= max_samples:
                    clusters.append(cur)
                cur = [i]
                continue

        # 거리 기반 클러스터링
        r0, r1 = r_list[i - 1], r_list[i]
        dij = math.sqrt(r0 * r0 + r1 * r1 - 2.0 * r0 * r1 * cos_ang_inc)
        link = max(eps0, k * min(r0, r1))

        if dij <= link:
            cur.append(i)
        else:
            if min_samples <= len(cur) <= max_samples:
                clusters.append(cur)
            cur = [i]

    # 마지막 클러스터 처리
    if min_samples <= len(cur) <= max_samples:
        clusters.append(cur)

    return clusters


# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    """
    LaserScan → map 변환 → 초경량 클러스터링 → 중심 표시
    + CSV outer/inner 링 필터
    + 정적/동적 판정 후 PoseArray에 z=0/1로 라벨
    """

    def __init__(self):
        super().__init__("simple_scan_viz")

        # ---------------------------
        # ROS 파라미터 선언
        # ---------------------------
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

        # 클러스터 파라미터
        self.declare_parameter("cluster_eps0", 0.12)
        self.declare_parameter("cluster_k", 0.06)
        self.declare_parameter("max_samples", 50)

        # 장애물 크기 필터
        self.declare_parameter("max_obstacle_size", 0.5)
        self.declare_parameter("min_obstacle_size", 0.0)
        self.declare_parameter("enable_size_filter", False)

        # TF 및 샘플링
        self.declare_parameter("use_vectorized_tf", True)
        self.declare_parameter("sample_stride", 2)

        # 정적/동적 판정용 파라미터 (방식1 트래커)
        self.declare_parameter("static_radius", 0.25)         # r: 앵커 기준 반경
        self.declare_parameter("static_min_hits", 5)          # 누적 관측 최소 횟수
        self.declare_parameter("static_max_age", 3.0)         # 미관측 허용 시간(sec)
        self.declare_parameter("static_vel_thresh", 0.08)     # v_th (m/s)
        self.declare_parameter("vel_ema_alpha", 0.3)          # 속도 EMA 알파

        # ---------------------------
        # 파라미터 로드
        # ---------------------------
        self.scan_topic       = self.get_parameter("scan_topic").value
        self.marker_frame     = self.get_parameter("marker_frame_id").value
        self.tf_timeout       = float(self.get_parameter("tf_timeout").value)

        self.min_samples      = int(self.get_parameter("min_samples").value)
        self.roi_min_dist     = float(self.get_parameter("roi_min_dist").value)
        self.roi_max_dist     = float(self.get_parameter("roi_max_dist").value)
        self.center_scale     = float(self.get_parameter("center_scale").value)

        self.fov_deg          = float(self.get_parameter("fov_deg").value)
        self.fov_center_deg   = float(self.get_parameter("fov_center_deg").value)

        self.outer_csv        = self.get_parameter("outer_bound_csv").value
        self.inner_csv        = self.get_parameter("inner_bound_csv").value

        self.cluster_eps0     = float(self.get_parameter("cluster_eps0").value)
        self.cluster_k        = float(self.get_parameter("cluster_k").value)
        self.max_samples      = int(self.get_parameter("max_samples").value)

        self.max_obstacle_size= float(self.get_parameter("max_obstacle_size").value)
        self.min_obstacle_size= float(self.get_parameter("min_obstacle_size").value)
        self.enable_size_filter = bool(self.get_parameter("enable_size_filter").value)

        self.use_vectorized_tf= bool(self.get_parameter("use_vectorized_tf").value)
        self.sample_stride    = int(self.get_parameter("sample_stride").value)

        self.static_radius    = float(self.get_parameter("static_radius").value)
        self.static_min_hits  = int(self.get_parameter("static_min_hits").value)
        self.static_max_age   = float(self.get_parameter("static_max_age").value)
        self.static_vel_thresh= float(self.get_parameter("static_vel_thresh").value)
        self.vel_ema_alpha    = float(self.get_parameter("vel_ema_alpha").value)

        # ---------------------------
        # 경계 로드 (outer / inner)
        # ---------------------------
        self.outer_poly = load_world_csv(self.outer_csv) if self.outer_csv else []
        self.inner_poly = load_world_csv(self.inner_csv) if self.inner_csv else []
        if self.outer_poly:
            self.get_logger().info(f"[bounds] outer: {len(self.outer_poly)} pts")
        if self.inner_poly:
            self.get_logger().info(f"[bounds] inner: {len(self.inner_poly)} pts")

        # ---------------------------
        # TF 버퍼
        # ---------------------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------
        # ROS I/O
        # ---------------------------
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)

        # RViz 시각화용 MarkerArray
        self.pub_markers = self.create_publisher(MarkerArray, "scan_viz/markers", 1)

        # lane_selector 등 downstream용 PoseArray
        # (여기서 pose.position.{x,y} = obstacle pos, pose.position.z = 0(static) or 1(dynamic))
        self.pub_obstacles = self.create_publisher(PoseArray, "scan_viz/obstacles", 10)

        # ---------------------------
        # FOV 캐싱
        # ---------------------------
        self.fov_idx_min = None
        self.fov_idx_max = None
        self.fov_indices_computed = False

        # ---------------------------
        # 트래커 상태(방식1)
        # ---------------------------
        # track_id -> dict(
        #   anchor_x, anchor_y,
        #   last_x, last_y,
        #   t0, t_last,
        #   hits,
        #   v_avg
        # )
        self._tracks = {}
        self._next_track_id = 1

        # 마지막 로그용
        self._prev_obstacle_count = 0

        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}]m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°, "
            f"cluster_size=[{self.min_samples},{self.max_samples}], "
            f"vectorized_tf={self.use_vectorized_tf}, "
            f"size_filter={self.enable_size_filter}, "
            f"sample_stride={self.sample_stride}, "
            f"static_radius={self.static_radius}, "
            f"static_min_hits={self.static_min_hits}, "
            f"static_vel_thresh={self.static_vel_thresh}"
        )

    # ---------------------------
    # 내부 유틸: 거리
    # ---------------------------
    def _dist(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    # ---------------------------
    # 트랙 정리: 오래 안 보인 트랙 제거
    # ---------------------------
    def _prune_tracks(self, now_sec: float):
        drop_ids = [
            tid for tid, tr in self._tracks.items()
            if now_sec - tr["t_last"] > self.static_max_age
        ]
        for tid in drop_ids:
            self._tracks.pop(tid, None)

    # ---------------------------
    # 트랙이 정적인가?
    # 조건:
    #   hits ≥ static_min_hits
    #   anchor 근처 반경 static_radius 안에 last 위치 유지
    #   v_avg ≤ static_vel_thresh
    # ---------------------------
    def _is_static_track(self, tr: dict) -> bool:
        if tr["hits"] < self.static_min_hits:
            return False
        near_anchor = self._dist(
            tr["last_x"], tr["last_y"],
            tr["anchor_x"], tr["anchor_y"]
        ) <= self.static_radius
        slow_enough = (tr["v_avg"] <= self.static_vel_thresh)
        return near_anchor and slow_enough

    # ---------------------------
    # 트랙 갱신 / 신규 생성
    # - centers_xy: [(x,y), ...] 이번 프레임 감지된 장애물들
    # ---------------------------
    def _ingest_observations(self, centers_xy: List[Tuple[float,float]], now_sec: float):
        for (cx, cy) in centers_xy:
            best_id = None
            best_d = 1e9

            # 앵커 기준 r 안에서 가장 가까운 트랙 찾기
            for tid, tr in self._tracks.items():
                d = self._dist(cx, cy, tr["anchor_x"], tr["anchor_y"])
                if d <= self.static_radius and d < best_d:
                    best_id = tid
                    best_d = d

            if best_id is None:
                # 새 트랙 생성
                tid = self._next_track_id
                self._next_track_id += 1
                self._tracks[tid] = {
                    "anchor_x": cx,
                    "anchor_y": cy,
                    "last_x": cx,
                    "last_y": cy,
                    "t0": now_sec,
                    "t_last": now_sec,
                    "hits": 1,
                    "v_avg": 0.0,
                }
            else:
                # 기존 트랙 갱신
                tr = self._tracks[best_id]
                dt = max(1e-3, now_sec - tr["t_last"])
                dx = cx - tr["last_x"]
                dy = cy - tr["last_y"]
                v_inst = math.hypot(dx, dy) / dt

                a = self.vel_ema_alpha
                tr["v_avg"] = a * v_inst + (1.0 - a) * tr["v_avg"]

                tr["hits"]   += 1
                tr["last_x"]  = cx
                tr["last_y"]  = cy
                tr["t_last"]  = now_sec

    # ---------------------------
    # PoseArray 퍼블리시:
    #   position.x, position.y = 장애물 center
    #   position.z = 0(정적) / 1(동적)
    # ---------------------------
    def _publish_flagged_obstacles(self,
                                   centers_ring: List[Point],
                                   static_mask: List[bool]):
        pose_arr = PoseArray()
        pose_arr.header.frame_id = self.marker_frame
        pose_arr.header.stamp = self.get_clock().now().to_msg()

        for pt, is_static in zip(centers_ring, static_mask):
            pose = Pose()
            pose.position.x = pt.x
            pose.position.y = pt.y
            pose.position.z = 0.0 if is_static else 1.0  # <- 라벨
            pose.orientation.w = 1.0
            pose_arr.poses.append(pose)

        self.pub_obstacles.publish(pose_arr)

    # ---------------------------
    # RViz 마커만 퍼블리시
    # ---------------------------
    def _publish_centers_marker(self, centers: List[Point]):
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
        self.pub_markers.publish(arr)

    # ---------------------------
    # RViz 마커 클리어
    # ---------------------------
    def _publish_clear_markers(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub_markers.publish(arr)

    # ---------------------------
    # TF lookup
    # ---------------------------
    def _lookup_latest(self, target_frame: str, source_frame: str):
        tf = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            RclTime(),
            timeout=Duration(seconds=self.tf_timeout)
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])  # note quat2mat([w,x,y,z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T

    # ---------------------------
    # FOV 인덱스 계산 (각도 범위를 인덱스 범위로 캐싱)
    # ---------------------------
    def _compute_fov_indices(self, ang_min: float, ang_inc: float, n_ranges: int):
        fov_rad = math.radians(self.fov_deg)
        fov_center = math.radians(self.fov_center_deg)
        half_fov = 0.5 * fov_rad

        angle_min_fov = fov_center - half_fov
        angle_max_fov = fov_center + half_fov

        idx_min = max(0, int((angle_min_fov - ang_min) / ang_inc))
        idx_max = min(n_ranges, int((angle_max_fov - ang_min) / ang_inc) + 1)

        return idx_min, idx_max

    # ---------------------------
    # 스캔 콜백
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
        ranges = np.array(scan.ranges, dtype=float)
        n_ranges = len(ranges)

        if n_ranges == 0 or ang_inc == 0.0:
            return

        # FOV 인덱스 처음만 계산
        if not self.fov_indices_computed:
            self.fov_idx_min, self.fov_idx_max = self._compute_fov_indices(ang_min, ang_inc, n_ranges)
            self.fov_indices_computed = True
            self.get_logger().info(
                f"[FOV] Computed indices: [{self.fov_idx_min}, {self.fov_idx_max}) "
                f"out of {n_ranges} total beams"
            )

        # ==========================================
        # 1) ROI/FOV 필터링
        # ==========================================
        idx_start = self.fov_idx_min
        idx_end   = self.fov_idx_max

        ranges_fov = ranges[idx_start:idx_end]

        # valid range mask
        valid_mask = ~(np.isnan(ranges_fov) | np.isinf(ranges_fov))
        valid_mask &= (ranges_fov >= rmin) & (ranges_fov <= rmax)

        # ROI distance mask
        use_roi = self.roi_max_dist > self.roi_min_dist
        if use_roi:
            valid_mask &= (ranges_fov >= self.roi_min_dist) & (ranges_fov <= self.roi_max_dist)

        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            self._publish_clear_markers()
            # 트랙도 일정 시간 지나면 정리
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self._prune_tracks(now_sec)
            # 빈 obstacles도 보낼까? 상황에 따라 결정.
            empty_arr = PoseArray()
            empty_arr.header.frame_id = self.marker_frame
            empty_arr.header.stamp = self.get_clock().now().to_msg()
            self.pub_obstacles.publish(empty_arr)
            self._prev_obstacle_count = 0
            return

        # stride 샘플링
        if self.sample_stride > 1:
            valid_indices = valid_indices[::self.sample_stride]
            if len(valid_indices) == 0:
                self._publish_clear_markers()
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                self._prune_tracks(now_sec)
                empty_arr = PoseArray()
                empty_arr.header.frame_id = self.marker_frame
                empty_arr.header.stamp = self.get_clock().now().to_msg()
                self.pub_obstacles.publish(empty_arr)
                self._prev_obstacle_count = 0
                return

        # r_list (거리들)
        r_list = ranges_fov[valid_indices].tolist()

        # 각도 계산 (원래 스캔 전체에서의 인덱스를 이용해서)
        angles_fov = ang_min + np.arange(idx_start, idx_end) * ang_inc
        th_list = angles_fov[valid_indices].tolist()

        # 원본 인덱스(연속성 체크용)
        original_indices = valid_indices.tolist()

        # stride 반영 각도 간격 / 클러스터 파라미터
        effective_ang_inc = ang_inc * self.sample_stride
        effective_eps0 = self.cluster_eps0 * self.sample_stride
        effective_k    = self.cluster_k * self.sample_stride

        # ==========================================
        # 2) 초경량 클러스터링
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
            self._publish_clear_markers()
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self._prune_tracks(now_sec)
            empty_arr = PoseArray()
            empty_arr.header.frame_id = self.marker_frame
            empty_arr.header.stamp = self.get_clock().now().to_msg()
            self.pub_obstacles.publish(empty_arr)
            self._prev_obstacle_count = 0
            return

        # ==========================================
        # 3) 클러스터 중심 계산 + TF(laser->map)
        # ==========================================
        if self.use_vectorized_tf:
            angles_arr = np.array(th_list, dtype=float)
            ranges_arr = np.array(r_list, dtype=float)

            # laser frame 좌표
            x_laser = ranges_arr * np.cos(angles_arr)
            y_laser = ranges_arr * np.sin(angles_arr)
            z_laser = np.zeros_like(ranges_arr)

            xyz_laser = np.vstack([x_laser, y_laser, z_laser])  # (3, N)
            xyz_map = R_ml @ xyz_laser + T_ml[:, np.newaxis]    # (3, N)

            centers = []
            for idxs in idx_clusters:
                cluster_x = xyz_map[0, idxs]
                cluster_y = xyz_map[1, idxs]

                cx = float(np.mean(cluster_x))
                cy = float(np.mean(cluster_y))

                # optional size filter
                if self.enable_size_filter:
                    x_min, x_max = float(np.min(cluster_x)), float(np.max(cluster_x))
                    y_min, y_max = float(np.min(cluster_y)), float(np.max(cluster_y))
                    size = max(x_max - x_min, y_max - y_min)
                    if size > self.max_obstacle_size or size < self.min_obstacle_size:
                        continue

                centers.append(Point(x=cx, y=cy, z=0.0))

        else:
            centers = []
            for idxs in idx_clusters:
                sx = 0.0
                sy = 0.0
                for j in idxs:
                    r = r_list[j]
                    th = th_list[j]
                    p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0])
                    p_m = R_ml @ p_l + T_ml
                    sx += p_m[0]
                    sy += p_m[1]
                n = len(idxs)
                centers.append(Point(x=sx/n, y=sy/n, z=0.0))

        # ==========================================
        # 4) CSV 링 필터 (outer 안 AND inner 밖)
        # ==========================================
        centers_ring: List[Point] = [
            p for p in centers
            if in_ring(
                p.x, p.y,
                self.outer_poly or None,
                self.inner_poly or None
            )
        ]

        # ==========================================
        # 5) 정적/동적 라벨링 + 퍼블리시
        # ==========================================
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._prune_tracks(now_sec)

        if not centers_ring:
            # 마커 clear
            self._publish_clear_markers()

            # 빈 PoseArray (장애물 없음)
            empty_arr = PoseArray()
            empty_arr.header.frame_id = self.marker_frame
            empty_arr.header.stamp = self.get_clock().now().to_msg()
            self.pub_obstacles.publish(empty_arr)

            if self._prev_obstacle_count != 0:
                self.get_logger().info("[CLEAR] No obstacles in ring")
                self._prev_obstacle_count = 0
            return

        # 이번 프레임 관측 좌표 목록
        centers_xy = [(p.x, p.y) for p in centers_ring]

        # 트랙 갱신 (EMA 속도 포함)
        self._ingest_observations(centers_xy, now_sec)

        # 각 center가 정적인지 여부 계산
        static_mask: List[bool] = []
        for (cx, cy) in centers_xy:
            # 가장 가까운 앵커 트랙 찾기 (반경 체크 없이 NN로만 붙여도 됨)
            best_id = None
            best_d = 1e9
            for tid, tr in self._tracks.items():
                d = self._dist(cx, cy, tr["anchor_x"], tr["anchor_y"])
                if d < best_d:
                    best_d = d
                    best_id = tid

            is_static_here = False
            if best_id is not None:
                tr = self._tracks[best_id]
                is_static_here = self._is_static_track(tr)

            static_mask.append(is_static_here)

        n_total = len(centers_ring)
        n_static = sum(static_mask)
        n_moving = n_total - n_static

        # 🚧 상세 로그: 장애물 개수 + 각 장애물 정보
        if n_total > 0:
            # 정적/동적 장애물 위치 분리
            static_positions = [centers_ring[i] for i in range(n_total) if static_mask[i]]
            dynamic_positions = [centers_ring[i] for i in range(n_total) if not static_mask[i]]

            self.get_logger().info(
                f"[OBSTACLE] Total={n_total} | Static={n_static} | Dynamic={n_moving}",
                throttle_duration_sec=0.5
            )

            # 정적 장애물 위치 출력 (Point 객체에서 x, y 추출)
            if static_positions:
                static_str = ", ".join([f"({p.x:.2f},{p.y:.2f})" for p in static_positions[:3]])  # 최대 3개만
                if len(static_positions) > 3:
                    static_str += f" +{len(static_positions)-3} more"
                self.get_logger().info(
                    f"  🔴 Static: {static_str}",
                    throttle_duration_sec=0.5
                )

            # 동적 장애물 위치 출력 (Point 객체에서 x, y 추출)
            if dynamic_positions:
                dynamic_str = ", ".join([f"({p.x:.2f},{p.y:.2f})" for p in dynamic_positions[:3]])  # 최대 3개만
                if len(dynamic_positions) > 3:
                    dynamic_str += f" +{len(dynamic_positions)-3} more"
                self.get_logger().info(
                    f"  🟢 Dynamic: {dynamic_str}",
                    throttle_duration_sec=0.5
                )

        # 장애물 개수 변화 시 추가 로그
        if n_total != self._prev_obstacle_count:
            self.get_logger().warn(
                f"⚠️ [OBSTACLE COUNT CHANGED] {self._prev_obstacle_count} → {n_total} "
                f"(Static: {n_static}, Dynamic: {n_moving})"
            )
            self._prev_obstacle_count = n_total

        # RViz 마커 (노란점들)
        self._publish_centers_marker(centers_ring)

        # PoseArray: z=0(static) / z=1(dynamic)
        self._publish_flagged_obstacles(centers_ring, static_mask)


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
