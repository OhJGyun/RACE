#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real LaserScan -> map clustering visualization (DBSCAN)
- 실제 차량용: sim_time / clock 관련 코드 없음
- LaserScan → map 변환 → DBSCAN → 중심점 RViz 표시
"""

from __future__ import annotations
import math, random
from typing import List, Tuple, Sequence
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from builtin_interfaces.msg import Time as BuiltinTime

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from transforms3d.quaternions import quat2mat

Point2D = Tuple[float, float]

# -------------------------------
# DBSCAN (O(N^2) 구현)
# -------------------------------
UNVISITED = 0
NOISE = -1

def _region_query(pts: List[Point2D], idx: int, eps: float) -> List[int]:
    x0, y0 = pts[idx]
    eps2 = eps * eps
    neigh = []
    for j, (x, y) in enumerate(pts):
        dx = x - x0; dy = y - y0
        if dx*dx + dy*dy <= eps2:
            neigh.append(j)
    return neigh

def _expand_cluster(labels: List[int], pts: List[Point2D], idx: int,
                    neighbors: List[int], cluster_id: int, eps: float, min_samples: int) -> None:
    labels[idx] = cluster_id
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
            q.extend(neigh_j)

def dbscan(pts: List[Point2D], eps: float, min_samples: int) -> List[List[int]]:
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

    clusters: List[List[int]] = []
    for cid in range(1, cluster_id + 1):
        clusters.append([i for i, lab in enumerate(labels) if lab == cid])
    return clusters


# -------------------------------
# Node
# -------------------------------
class SimpleScanViz(Node):
    def __init__(self) -> None:
        super().__init__("simple_scan_viz")

        # Parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("db_eps", 0.25)
        self.declare_parameter("db_min_samples", 5)
        self.declare_parameter("point_scale", 0.05)

        # Load params
        self.scan_topic = self.get_parameter("scan_topic").value
        self.marker_frame = self.get_parameter("marker_frame_id").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.db_eps = float(self.get_parameter("db_eps").value)
        self.db_min_samples = int(self.get_parameter("db_min_samples").value)
        self.pt_scale = float(self.get_parameter("point_scale").value)

        # TF listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)

        self.tf_ready = False
        self._ready_timer = self.create_timer(0.5, self._check_tf)

        self.get_logger().info(
            f"[init] Real-time clustering viz | frame={self.marker_frame}, eps={self.db_eps}, min_samples={self.db_min_samples}"
        )

    def _check_tf(self):
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
        if not self.tf_ready:
            return

        laser_frame = scan.header.frame_id or "laser"
        try:
            R_ml, T_ml = self._lookup_latest(laser_frame)
        except TransformException:
            return

        ang_min = float(scan.angle_min)
        ang_inc = float(scan.angle_increment)
        ranges = scan.ranges
        if not ranges or ang_inc == 0.0:
            return

        rmin, rmax = float(scan.range_min), float(scan.range_max)
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
        self._publish_centers(pts_map, clusters)

    # 중심점만 표시
    def _publish_centers(self, pts: Sequence[Point2D], clusters: List[List[int]]):
        arr = MarkerArray()

        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cluster_centers"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.scale.x = self.pt_scale * 3.0
        m.scale.y = self.pt_scale * 3.0
        m.scale.z = self.pt_scale * 3.0
        m.color.r = 1.0
        m.color.g = 0.4
        m.color.b = 0.0
        m.color.a = 0.9

        for idxs in clusters:
            if not idxs:
                continue
            cx = sum(pts[i][0] for i in idxs) / len(idxs)
            cy = sum(pts[i][1] for i in idxs) / len(idxs)
            m.points.append(Point(x=cx, y=cy, z=0.0))

        arr.markers.append(m)
        self.pub.publish(arr)

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
