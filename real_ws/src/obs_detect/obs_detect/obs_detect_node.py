#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaserScan → map 변환 → DBSCAN → '중심만' RViz 시각화
+ CSV로 읽은 outer/inner 경계 사이(outer 안 ∧ inner 밖)에 있는 중심만 통과
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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from transforms3d.quaternions import quat2mat
# -------------------------------
# 타입
# -------------------------------
Point2D = Tuple[float, float]
UNVISITED = 0
NOISE = -1
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
# 폴리곤 판정 (점-다각형 교차수 기반)
# -------------------------------
def point_in_polygon(x: float, y: float, poly: Sequence[Point2D], include_boundary: bool = True) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        # y축 교차 여부
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
# DBSCAN (O(N^2) 단순구현)
# -------------------------------
def _region_query(pts: List[Point2D], idx: int, eps: float) -> List[int]:
    x0, y0 = pts[idx]; eps2 = eps * eps
    out = []
    for j, (x, y) in enumerate(pts):
        dx = x - x0; dy = y - y0
        if dx*dx + dy*dy <= eps2:
            out.append(j)
    return out
def _expand_cluster(labels, pts, idx, neigh, cid, eps, min_samples):
    labels[idx] = cid
    q = list(neigh); qi = 0
    while qi < len(q):
        j = q[qi]; qi += 1
        if labels[j] == NOISE:
            labels[j] = cid
        if labels[j] != UNVISITED:
            continue
        labels[j] = cid
        neigh_j = _region_query(pts, j, eps)
        if len(neigh_j) >= min_samples:
            q.extend(neigh_j)
def dbscan(pts: List[Point2D], eps: float, min_samples: int) -> List[List[int]]:
    n = len(pts)
    if n == 0:
        return []
    labels = [UNVISITED] * n
    cid = 0
    for i in range(n):
        if labels[i] != UNVISITED:
            continue
        neigh = _region_query(pts, i, eps)
        if len(neigh) < min_samples:
            labels[i] = NOISE
            continue
        cid += 1
        _expand_cluster(labels, pts, i, neigh, cid, eps, min_samples)
    return [[i for i, l in enumerate(labels) if l == c] for c in range(1, cid + 1)]
# 각도 정규화 & FOV
def _ang_norm(a: float) -> float:
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    """
    LaserScan → marker_frame 투영 → DBSCAN → 중심만 표시
    + CSV outer/inner 사이에 있는 중심만 통과
    """
    def __init__(self):
        super().__init__("simple_scan_viz")
        # 기본
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)
        # DBSCAN & ROI & FOV
        self.declare_parameter("db_eps", 0.25)
        self.declare_parameter("db_min_samples", 5)
        self.declare_parameter("roi_min_dist", 0.20)
        self.declare_parameter("roi_max_dist", 6.00)
        self.declare_parameter("center_scale", 0.12)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("fov_center_deg", 0.0)
        # 경계 CSV
        self.declare_parameter("outer_bound_csv", "")
        self.declare_parameter("inner_bound_csv", "")
        # 파라미터 로드
        self.scan_topic     = self.get_parameter("scan_topic").value
        self.marker_frame   = self.get_parameter("marker_frame_id").value
        self.tf_timeout     = float(self.get_parameter("tf_timeout").value)
        self.db_eps         = float(self.get_parameter("db_eps").value)
        self.db_min_samples = int(self.get_parameter("db_min_samples").value)
        self.roi_min_dist   = float(self.get_parameter("roi_min_dist").value)
        self.roi_max_dist   = float(self.get_parameter("roi_max_dist").value)
        self.center_scale   = float(self.get_parameter("center_scale").value)
        self.fov_deg        = float(self.get_parameter("fov_deg").value)
        self.fov_center_deg = float(self.get_parameter("fov_center_deg").value)
        self.outer_csv = self.get_parameter("outer_bound_csv").value
        self.inner_csv = self.get_parameter("inner_bound_csv").value
        # 경계 로드 (marker_frame 좌표계 기준이어야 함)
        self.outer_poly: List[Point2D] = load_world_csv(self.outer_csv) if self.outer_csv else []
        self.inner_poly: List[Point2D] = load_world_csv(self.inner_csv) if self.inner_csv else []
        if self.outer_poly:
            self.get_logger().info(f"[bounds] loaded outer: {len(self.outer_poly)} pts from {self.outer_csv}")
        if self.inner_poly:
            self.get_logger().info(f"[bounds] loaded inner: {len(self.inner_poly)} pts from {self.inner_csv}")
        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # IO
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)
        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"DBSCAN(eps={self.db_eps}, min_samples={self.db_min_samples}), "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}] m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°"
        )
    def _lookup_latest(self, target_frame: str, source_frame: str):
        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, RclTime(),
                                             timeout=Duration(seconds=self.tf_timeout))
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T
    def _publish_clear(self):
        arr = MarkerArray()
        m = Marker(); m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub.publish(arr)
    def _publish_centers(self, centers: List[Point]):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cluster_centers_in_ring"
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
    def _on_scan(self, scan: LaserScan):
        laser_frame = scan.header.frame_id or "laser"
        try:
            R_ml, T_ml = self._lookup_latest(self.marker_frame, laser_frame)
        except TransformException as e:
            self.get_logger().warn_once(f"TF not available: {e}")
            return
        ang_min = float(scan.angle_min)
        ang_inc = float(scan.angle_increment)
        rmin = float(scan.range_min)
        rmax = float(scan.range_max)
        ranges = scan.ranges
        if not ranges or ang_inc == 0.0:
            return
        # FOV/ROI
        fov_rad = math.radians(self.fov_deg)
        fov_center = math.radians(self.fov_center_deg)
        half_fov = 0.5 * fov_rad
        use_roi = self.roi_max_dist > self.roi_min_dist
        # laser → marker_frame
        pts_map: List[Point2D] = []
        for i, r in enumerate(ranges):
            r = float(r)
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            if use_roi and (r < self.roi_min_dist or r > self.roi_max_dist):
                continue
            th = ang_min + i * ang_inc  # x+ 기준, CCW +
            dth = _ang_norm(th - fov_center)
            if abs(dth) > half_fov:
                continue
            p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0], dtype=float)
            p_m = R_ml @ p_l + T_ml
            pts_map.append((float(p_m[0]), float(p_m[1])))
        if not pts_map:
            self._publish_clear()
            return
        # 클러스터링 → 중심
        clusters = dbscan(pts_map, eps=self.db_eps, min_samples=self.db_min_samples)
        centers_all: List[Point] = []
        for idxs in clusters:
            if not idxs:
                continue
            cx = sum(pts_map[i][0] for i in idxs) / len(idxs)
            cy = sum(pts_map[i][1] for i in idxs) / len(idxs)
            centers_all.append(Point(x=cx, y=cy, z=0.0))
        # :작은_파란색_다이아몬드: 링(inner/outer) 필터
        centers_ring: List[Point] = []
        for p in centers_all:
            if in_ring(p.x, p.y, self.outer_poly if self.outer_poly else None,
                                  self.inner_poly if self.inner_poly else None):
                centers_ring.append(p)
        # 퍼블리시
        if centers_ring:
            self._publish_centers(centers_ring)
        else:
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