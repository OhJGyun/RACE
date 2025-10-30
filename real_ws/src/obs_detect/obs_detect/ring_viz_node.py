#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple LaserScan -> map visualization

기능 개요:
-----------
- /scan 토픽에서 LaserScan 메시지를 구독하여 각 레이저 빔을 읽음
- TF(Buffer, TransformListener)를 통해 laser → map 변환을 조회
- 변환된 포인트를 visualization_msgs/MarkerArray로 RViz에 시각화
- 최신 TF만 사용 (타임스탬프 기반 보간/외삽 없음)
- Marker.header.stamp는 LaserScan.header.stamp 또는 현재 시간 사용 가능
"""

from __future__ import annotations
import math
from typing import List, Tuple, Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from builtin_interfaces.msg import Time as BuiltinTime

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# TF2 (좌표 변환용)
from tf2_ros import Buffer, TransformListener, TransformException
from transforms3d.quaternions import quat2mat

# csv 파일 용
import os
import csv


# 2D 포인트 타입 정의
Point2D = Tuple[float, float]

# -------------------------------
# OBJ 유틸리티
# -------------------------------

# (1) CSV 파일에서 (x, y) 좌표 불러오기 
def load_world_csv(path: str) -> List[Point2D]:
    """CSV에서 (x, y) 좌표 목록을 불러온다."""
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

# (2) 내부 외부 판정 (반 직선 통과하는 선분 개수로 판정)
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

# (3) 점과 선분 사이의 최소 거리 계산
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

# 점과 다각형(여러 선분) 사이의 최단 거리 계산
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

# -------------------------------
# DBSCAN (브루트포스 구현)
# -------------------------------
# eps : 이웃 반경
# min_samples : 코어 포인트 최소 이웃 수

UNVISITED = 0
NOISE = -1

# (1) 입력 점 i에 대해 모든 점과의 거리 계산 후 eps 이내인 점들의 인덱스 반환 
def _region_query(pts: List[Point2D], idx: int, eps: float) -> List[int]:
    """idx 포인트의 eps-이웃 인덱스 리스트"""
    x0, y0 = pts[idx]
    eps2 = eps * eps
    neigh = []
    for j, (x, y) in enumerate(pts):
        dx = x - x0; dy = y - y0
        if dx*dx + dy*dy <= eps2:
            neigh.append(j)
    return neigh

# (2) 코어 포인트 idx를 포함한 이웃을 확장하여 cluster_id 부여
# => 다른 점들(len(neigh) < min_samples)은 노이즈로 분류 
def _expand_cluster(labels: List[int], pts: List[Point2D], idx: int,
                    neighbors: List[int], cluster_id: int, eps: float, min_samples: int) -> None:
    """core 포인트 idx를 포함한 이웃을 확장하여 cluster_id 부여"""
    labels[idx] = cluster_id
    # queue 기반 확장
    q = list(neighbors)
    qi = 0
    while qi < len(q):
        j = q[qi]; qi += 1
        if labels[j] == NOISE:
            labels[j] = cluster_id
        if labels[j] != UNVISITED:
            continue
        labels[j] = cluster_id
        neigh_j = _region_query(pts, j, eps)
        if len(neigh_j) >= min_samples:
            # core면 주변을 더 확장
            # (DBSCAN 정의: border 포인트는 확장하지 않지만 core는 확장)
            q.extend(neigh_j)

#  (3) DBSCAN 메인 함수
def dbscan(pts: List[Point2D], eps: float, min_samples: int) -> List[List[int]]:
    """
    입력: pts (N x 2)
    출력: 클러스터별 인덱스 리스트들의 리스트  (예: [[0,1,8,...], [5,6,...], ...])
    """
    n = len(pts)
    if n == 0:
        return []
    labels = [UNVISITED] * n
    cluster_id = 0
    for i in range(n):
        if labels[i] != UNVISITED:
            continue
        neigh = _region_query(pts, i, eps)
        if len(neigh) < min_samples:
            labels[i] = NOISE
            continue
        cluster_id += 1
        _expand_cluster(labels, pts, i, neigh, cluster_id, eps, min_samples)

    # cluster_id는 1..K, NOISE=-1
    clusters: List[List[int]] = []
    for cid in range(1, cluster_id + 1):
        clusters.append([i for i, lab in enumerate(labels) if lab == cid])
    # 필요하다면 노이즈 반환:
    # noise = [i for i, lab in enumerate(labels) if lab == NOISE]
    return clusters

###
# Core o----o----o   (eps 안에 서로 연결됨)
#        \  / \
#         o o  o  ← border points
#              \
#               . (noise)
###


# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    """
    LaserScan → map 변환 → DBSCAN → 클러스터만 RViz로 시각화
    """

    def __init__(self) -> None:
        super().__init__("simple_scan_viz")

        # 파라미터
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("use_scan_stamp_for_markers", True)

        # DBSCAN 파라미터
        self.declare_parameter("db_eps", 0.25)          # [m]
        self.declare_parameter("db_min_samples", 5)     # core 판정 이웃 수
        self.declare_parameter("point_scale", 0.04)     # 점 크기 (클러스터 점 표시용)
        self.declare_parameter("show_centers", True)    # 클러스터 중심 점(구)도 표시할지

        # 로드
        self.scan_topic     = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.marker_frame   = self.get_parameter("marker_frame_id").get_parameter_value().string_value
        self.tf_timeout     = float(self.get_parameter("tf_timeout").value)
        self.use_scan_stamp = bool(self.get_parameter("use_scan_stamp_for_markers").value)

        self.db_eps         = float(self.get_parameter("db_eps").value)
        self.db_min_samples = int(self.get_parameter("db_min_samples").value)
        self.pt_scale       = float(self.get_parameter("point_scale").value)
        self.show_centers   = bool(self.get_parameter("show_centers").value)

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IO
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)

        # 준비 체크
        self.clock_ready = False
        self.tf_ready    = False
        self._ready_timer = self.create_timer(0.1, self._check_ready)

        self.get_logger().info(
            f"[init] Clustering viz | scan={self.scan_topic}, frame={self.marker_frame}, "
            f"DBSCAN(eps={self.db_eps}, min_samples={self.db_min_samples})"
        )

    def _check_ready(self):
        if not self.clock_ready:
            if self.get_clock().now().nanoseconds > 0:
                self.clock_ready = True
                self.get_logger().info("[ready] sim clock active")
            else:
                return
        if not self.tf_ready:
            if self.tf_buffer.can_transform(self.marker_frame, "laser", RclTime(),
                                            timeout=Duration(seconds=self.tf_timeout)):
                self.tf_ready = True
                self.get_logger().info(f"[ready] TF available: {self.marker_frame} <- laser")

    def _lookup_latest(self, source_frame: str):
        tf = self.tf_buffer.lookup_transform(self.marker_frame, source_frame, RclTime(),
                                             timeout=Duration(seconds=self.tf_timeout))
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T

    def _on_scan(self, scan: LaserScan):
        if not (self.clock_ready and self.tf_ready):
            return

        laser_frame = scan.header.frame_id or "laser"

        # 최신 TF로 map <- laser
        try:
            R_ml, T_ml = self._lookup_latest(laser_frame)
        except TransformException:
            return

        ang_min = float(scan.angle_min)
        ang_inc = float(scan.angle_increment)
        ranges  = scan.ranges
        if not ranges or ang_inc == 0.0:
            return

        rmin = float(scan.range_min); rmax = float(scan.range_max)

        # laser -> map 투영
        pts_map: List[Point2D] = []
        for i, r in enumerate(ranges):
            r = float(r)
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            th = ang_min + i * ang_inc
            p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0], dtype=float)
            p_m = R_ml @ p_l + T_ml
            pts_map.append((float(p_m[0]), float(p_m[1])))

        # DBSCAN
        clusters = dbscan(pts_map, eps=self.db_eps, min_samples=self.db_min_samples)

        # 마커 타임스탬프
        stamp_for_markers: BuiltinTime = (
            scan.header.stamp if self.use_scan_stamp else self.get_clock().now().to_msg()
        )

        # 클러스터만 그리는 시각화
        def _publish_clusters(self, pts: Sequence[Point2D], clusters: List[List[int]], stamp: BuiltinTime):
            """
            클러스터 중심점만 RViz에 표시
            """
            arr = MarkerArray()

            # 기존 마커 모두 삭제
            m_clear = Marker()
            m_clear.action = Marker.DELETEALL
            arr.markers.append(m_clear)

            # 클러스터 중심 마커
            mc = Marker()
            mc.header.frame_id = self.marker_frame
            mc.header.stamp    = stamp
            mc.ns = "cluster_centers"
            mc.id = 0
            mc.type = Marker.SPHERE_LIST
            mc.scale.x = self.pt_scale * 3.0
            mc.scale.y = self.pt_scale * 3.0
            mc.scale.z = self.pt_scale * 3.0
            mc.color.r = 1.0
            mc.color.g = 0.5
            mc.color.b = 0.1
            mc.color.a = 0.95

            # 각 클러스터의 중심 계산
            for idxs in clusters:
                if not idxs:
                    continue
                cx = sum(pts[i][0] for i in idxs) / len(idxs)
                cy = sum(pts[i][1] for i in idxs) / len(idxs)
                mc.points.append(Point(x=cx, y=cy, z=0.0))

            arr.markers.append(mc)
            self.pub.publish(arr)

# 메인
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
