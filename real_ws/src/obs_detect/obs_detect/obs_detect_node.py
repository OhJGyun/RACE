#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle detection node for the obs_detect package.

LiDAR 기반 자유공간 장애물 감지 (경계 필터, 클러스터링, RViz 시각화)
- 오직 TF만 사용: map ← laser 변환을 스캔 타임스탬프에 맞춰 조회
- odom / amcl 폴백 없음
"""

from __future__ import annotations

import csv
import glob
import math
import os
import re
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple, Dict

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from transforms3d.quaternions import quat2mat
from rclpy.time import Time as RclTime
from builtin_interfaces.msg import Time as BuiltinTime
from tf2_ros import TransformException

# ============================================================================ #
# 공통 유틸
# ============================================================================ #

Point2D = Tuple[float, float]


def load_world_csv(path: str) -> List[Point2D]:
    """CSV에서 (x, y) 좌표 목록을 불러온다. (map 프레임 기준 가정)"""
    pts: List[Point2D] = []
    path = os.path.expanduser(path)
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            pts.append((float(row[0]), float(row[1])))
    if pts and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def load_multi_inner(spec: str) -> List[List[Point2D]]:
    """여러 내측 경계를 한꺼번에 불러온다 (구분자/글롭 지원)."""
    spec = os.path.expanduser(spec)
    if any(ch in spec for ch in ",; "):
        parts = [p for p in re.split(r"[,\s;]+", spec) if p]
    else:
        parts = [spec]

    paths: List[str] = []
    for p in parts:
        if any(ch in p for ch in "*?[]"):
            paths.extend(glob.glob(p))
        else:
            paths.append(p)

    inners: List[List[Point2D]] = []
    for p in paths:
        if not os.path.exists(p):
            continue
        poly = load_world_csv(p)
        if len(poly) >= 3:
            inners.append(poly)
    return inners


def point_in_polygon(x: float, y: float, poly: Sequence[Point2D], include_boundary: bool) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        cond = (y1 > y) != (y0 > y)
        if cond:
            t = (y - y0) / (y1 - y0 + 1e-12)
            xin = x0 + (x1 - x0) * t
            cmp = x <= xin if include_boundary else x < xin
            if cmp:
                inside = not inside
        x0, y0 = x1, y1
    return inside


def dist_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def min_dist_to_poly(px: float, py: float, poly: Sequence[Point2D]) -> float:
    if len(poly) < 2:
        return float("inf")
    dmin = float("inf")
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        d = dist_point_to_segment(px, py, x0, y0, x1, y1)
        if d < dmin:
            dmin = d
        x0, y0 = x1, y1
    return dmin


# ============================================================================ #
# 장애물 클러스터 구조체
# ============================================================================ #

@dataclass
class Cluster:
    points: List[Point2D]

    @property
    def center(self) -> Point2D:
        sx = sum(p[0] for p in self.points)
        sy = sum(p[1] for p in self.points)
        n = len(self.points) if self.points else 1
        return sx / n, sy / n


# ============================================================================ #
# 장애물 감지 노드 (TF-only)
# ============================================================================ #

class ObsDetectNode(Node):
    def __init__(self) -> None:
        super().__init__("obs_detect_node")

        # -------------------- 파라미터 선언 -------------------- #
        self.declare_parameter("scan_topic", "/scan")

        # Localization / TF 설정 (TF-only)
        self.declare_parameter("tf_timeout", 0.1)  # 초

        # 경계 정보 (map 프레임)
        self.declare_parameter("outer_csv", "outer_bound_world.csv")
        self.declare_parameter("inner_csvs", "inner_bound_world.csv")
        self.declare_parameter("include_boundary", False)

        # LiDAR ROI / 센서 설정
        self.declare_parameter("roi_deg", 90.0)          # 전방 ±90도
        self.declare_parameter("range_max", 12.0)
        # 디버깅용 추가 yaw 오프셋(가능하면 0으로 두고 TF에 맡기기)
        self.declare_parameter("laser_yaw_offset_deg", 0.0)

        # 장애물 필터링 & 클러스터링
        self.declare_parameter("boundary_margin", 0.50)
        self.declare_parameter("cluster_epsilon", 0.35)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("cluster_confirm_points", 6)

        # 시각화
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_scale", 0.35)

        # -------------------- 파라미터 로딩 -------------------- #
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        self.outer_csv = str(self.get_parameter("outer_csv").value)
        self.inner_spec = str(self.get_parameter("inner_csvs").value)
        self.include_boundary = bool(self.get_parameter("include_boundary").value)

        self.roi_rad = math.radians(float(self.get_parameter("roi_deg").value))
        self.range_max = float(self.get_parameter("range_max").value)
        self.laser_yaw = math.radians(float(self.get_parameter("laser_yaw_offset_deg").value))

        self.boundary_margin = float(self.get_parameter("boundary_margin").value)
        self.cluster_epsilon = float(self.get_parameter("cluster_epsilon").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_confirm_points = int(self.get_parameter("cluster_confirm_points").value)

        self.marker_frame = str(self.get_parameter("marker_frame_id").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)

        # -------------------- 경계 데이터 로딩 -------------------- #
        self.outer = load_world_csv(self.outer_csv)
        if len(self.outer) < 3:
            raise RuntimeError(f"Invalid outer boundary CSV: {self.outer_csv}")
        self.inners = load_multi_inner(self.inner_spec)
        self.get_logger().info(
            f"Bounds loaded: outer={len(self.outer)} pts, inners={len(self.inners)} polygon(s)"
        )

        # -------------------- TF2 초기화 -------------------- #
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------- ROS 인터페이스 -------------------- #
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)

        self.pub_obstacle_markers = self.create_publisher(MarkerArray, "obs_detect/markers", 1)
        self.pub_obstacle_centers = self.create_publisher(PoseArray, "obs_detect/obstacles", 10)
        self.pub_track_bounds = self.create_publisher(MarkerArray, "obs_detect/track_bounds", 1)

        # Track bounds 시각화 타이머 (2초마다)
        self.bound_viz_timer = self.create_timer(2.0, self._publish_track_bounds)
        # 상단 파라미터에 추가 (기본 0.08s 권장)
        self.declare_parameter("tf_time_tolerance", 0.08)
        self.tf_tol = float(self.get_parameter("tf_time_tolerance").value)


        self.get_logger().info("Obstacle detection node ready. Localization: TF-only (map<-laser @ scan time)")

    # -------------------- TF 헬퍼 -------------------- #
    def _lookup_map_from(self, source_frame: str, stamp_msg: BuiltinTime):
        """(map <- source_frame) 변환을 스캔 시간에 맞춰 조회하되,
        미래 외삽 방지를 위해 tolerance만큼 과거로 당겨보고,
        여전히 실패하면 latest로 폴백한다.
        """
        def do_lookup(query_time: RclTime):
            tfmsg = self.tf_buffer.lookup_transform(
                'map', source_frame, query_time, timeout=Duration(seconds=self.tf_timeout)
            )
            t = tfmsg.transform.translation
            q = tfmsg.transform.rotation
            R = quat2mat([q.w, q.x, q.y, q.z])
            T = np.array([t.x, t.y, t.z], dtype=float)
            return R, T

        # (A) 스캔 시각에서 tol만큼 과거로
        try:
            qt = RclTime.from_msg(stamp_msg) - Duration(seconds=self.tf_tol)
            return do_lookup(qt)
        except TransformException as e:
            self.get_logger().warn(f"TF @scan_time-tol failed: {e}", throttle_duration_sec=1.0)

        # (B) 스캔 시각 그대로 한 번 더 시도 (간혹 교차 타이밍으로 성공하는 케이스)
        try:
            qt = RclTime.from_msg(stamp_msg)
            return do_lookup(qt)
        except TransformException as e:
            self.get_logger().warn(f"TF @scan_time failed: {e}", throttle_duration_sec=1.0)

        # (C) 마지막 폴백: latest
        try:
            self.get_logger().warn("TF fallback to latest (may introduce slight spatial lag).",
                                throttle_duration_sec=2.0)
            return do_lookup(RclTime())
        except TransformException as e:
            self.get_logger().warn(f"TF latest failed: {e}", throttle_duration_sec=1.0)
            return None


    # -------------------- LiDAR 스캔 콜백 (장애물 감지) -------------------- #
    def _on_scan(self, scan: LaserScan) -> None:
        # 스캔 프레임명 확인 (반드시 TF에 존재해야 함)
        laser_frame = scan.header.frame_id or 'laser'

        # 스캔 시각에 맞춰 map <- laser TF 조회
        got = self._lookup_map_from(laser_frame, scan.header.stamp)
        if got is None:
            return
        R_ml, T_ml = got  # map <- laser

        ang_min = scan.angle_min
        ang_inc = scan.angle_increment
        n = len(scan.ranges)
        if n == 0 or ang_inc == 0.0:
            return

        max_range = min(self.range_max, scan.range_max)
        free_points: List[Point2D] = []
        boundary_filtered = 0
        roi_skipped = 0

        # 레이저 프레임에서 빔을 만들고, map으로 투영
        for i in range(n):
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > max_range:
                continue

            angle = ang_min + i * ang_inc  # laser 프레임 기준
            # ROI: 전방 ±roi_rad (laser x+가 전방이라는 표준 가정)
            if abs(angle) > self.roi_rad:
                roi_skipped += 1
                continue

            # 디버깅용 yaw 오프셋(가능하면 0으로 유지)
            a = angle + self.laser_yaw
            ex = r * math.cos(a)
            ey = r * math.sin(a)
            p_laser = np.array([ex, ey, 0.0])  # laser 좌표

            # map 좌표로 투영
            p_map = R_ml @ p_laser + T_ml
            wx, wy = float(p_map[0]), float(p_map[1])

            # 트랙 영역 필터 (map 프레임 경계)
            if not point_in_polygon(wx, wy, self.outer, self.include_boundary):
                continue
            in_inner = any(
                len(poly) >= 3 and point_in_polygon(wx, wy, poly, self.include_boundary)
                for poly in self.inners
            )
            if in_inner:
                continue

            # 경계 마진
            dist_outer = min_dist_to_poly(wx, wy, self.outer)
            dist_inner = min(
                (min_dist_to_poly(wx, wy, poly) for poly in self.inners if len(poly) >= 3),
                default=float("inf"),
            )
            if dist_outer < self.boundary_margin or dist_inner < self.boundary_margin:
                boundary_filtered += 1
                continue

            free_points.append((wx, wy))

        # 클러스터링
        clusters = self._cluster_points(free_points, self.cluster_epsilon, self.cluster_min_points)
        confirmed = [c for c in clusters if len(c.points) >= self.cluster_confirm_points]
        centers = [cluster.center for cluster in confirmed]

        if confirmed:
            self.get_logger().info(
                f"[OBSTACLE] confirmed={len(confirmed)} "
                f"(raw_points={len(free_points)}, roi_skip={roi_skipped}, boundary_filtered={boundary_filtered})"
            )

        # 시각화
        self._publish_obstacle_markers(free_points, confirmed)
        self._publish_obstacle_centers(centers)

    # -------------------- 장애물 클러스터링 -------------------- #
    def _cluster_points(self, points: Sequence[Point2D], epsilon: float, min_points: int) -> List[Cluster]:
        n = len(points)
        if n == 0:
            return []

        parent = list(range(n))

        def find(a: int) -> int:
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        eps2 = epsilon * epsilon
        for i in range(n):
            xi, yi = points[i]
            for j in range(i + 1, n):
                xj, yj = points[j]
                dx = xi - xj
                dy = yi - yj
                if dx * dx + dy * dy <= eps2:
                    union(i, j)

        buckets: Dict[int, List[int]] = {}
        for idx in range(n):
            root = find(idx)
            buckets.setdefault(root, []).append(idx)

        clusters: List[Cluster] = []
        for idxs in buckets.values():
            if len(idxs) < min_points:
                continue
            clusters.append(Cluster(points=[points[k] for k in idxs]))
        return clusters

    # -------------------- 장애물 시각화 -------------------- #
    def _publish_obstacle_markers(self, points: Sequence[Point2D], clusters: Iterable[Cluster]) -> None:
        arr = MarkerArray()
        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        now = self.get_clock().now().to_msg()

        # 원시 포인트 (map 프레임)
        m_pts = Marker()
        m_pts.header.frame_id = self.marker_frame
        m_pts.header.stamp = now
        m_pts.ns = "obs_detect"
        m_pts.id = 0
        m_pts.type = Marker.POINTS
        m_pts.scale.x = 0.06
        m_pts.scale.y = 0.06
        m_pts.color.r = 0.1
        m_pts.color.g = 0.55
        m_pts.color.b = 1.0
        m_pts.color.a = 0.9
        for x, y in points:
            m_pts.points.append(Point(x=float(x), y=float(y), z=0.0))
        arr.markers.append(m_pts)

        # 클러스터 중심 (map 프레임)
        m_centers = Marker()
        m_centers.header.frame_id = self.marker_frame
        m_centers.header.stamp = now
        m_centers.ns = "obs_detect"
        m_centers.id = 1
        m_centers.type = Marker.SPHERE_LIST
        m_centers.scale.x = self.marker_scale
        m_centers.scale.y = self.marker_scale
        m_centers.scale.z = self.marker_scale
        m_centers.color.r = 1.0
        m_centers.color.g = 0.2
        m_centers.color.b = 0.2
        m_centers.color.a = 0.95
        for cluster in clusters:
            cx, cy = cluster.center
            m_centers.points.append(Point(x=float(cx), y=float(cy), z=0.0))
        arr.markers.append(m_centers)

        self.pub_obstacle_markers.publish(arr)

    def _publish_obstacle_centers(self, centers: Sequence[Point2D]) -> None:
        arr = PoseArray()
        arr.header.frame_id = self.marker_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in centers:
            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            arr.poses.append(pose)
        self.pub_obstacle_centers.publish(arr)

    # -------------------- Track Bounds 시각화 -------------------- #
    def _publish_track_bounds(self) -> None:
        """Publish track boundaries (outer and inner) as LINE_STRIP markers."""
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Outer boundary - RED
        m_outer = Marker()
        m_outer.header.frame_id = self.marker_frame
        m_outer.header.stamp = now
        m_outer.ns = "track_bounds"
        m_outer.id = 0
        m_outer.type = Marker.LINE_STRIP
        m_outer.action = Marker.ADD
        m_outer.scale.x = 0.08  # Line width
        m_outer.color.r = 1.0
        m_outer.color.g = 0.0
        m_outer.color.b = 0.0
        m_outer.color.a = 0.8
        m_outer.pose.orientation.w = 1.0

        for x, y in self.outer:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            m_outer.points.append(p)
        markers.markers.append(m_outer)

        # Inner boundaries - BLUE
        for idx, inner_poly in enumerate(self.inners):
            m_inner = Marker()
            m_inner.header.frame_id = self.marker_frame
            m_inner.header.stamp = now
            m_inner.ns = "track_bounds"
            m_inner.id = idx + 1
            m_inner.type = Marker.LINE_STRIP
            m_inner.action = Marker.ADD
            m_inner.scale.x = 0.08
            m_inner.color.r = 0.0
            m_inner.color.g = 0.5
            m_inner.color.b = 1.0
            m_inner.color.a = 0.8
            m_inner.pose.orientation.w = 1.0

            for x, y in inner_poly:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.0
                m_inner.points.append(p)
            markers.markers.append(m_inner)

        self.pub_track_bounds.publish(markers)


# ============================================================================ #
# main
# ============================================================================ #

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObsDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
