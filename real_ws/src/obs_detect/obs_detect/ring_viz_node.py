#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaserScan → map 변환 → DBSCAN → 중심만 RViz 시각화
"""

from __future__ import annotations
import math
from typing import List, Tuple, Sequence
import numpy as np
import random

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
# 타입 정의
# -------------------------------
Point2D = Tuple[float, float]

# -------------------------------
# DBSCAN (O(N²) 단순 구현)
# -------------------------------
UNVISITED = 0
NOISE = -1

def _region_query(pts: List[Point2D], idx: int, eps: float) -> List[int]:
    x0, y0 = pts[idx]
    eps2 = eps * eps
    neigh = []
    for j, (x, y) in enumerate(pts):
        dx = x - x0
        dy = y - y0
        if dx * dx + dy * dy <= eps2:
            neigh.append(j)
    return neigh

def _expand_cluster(labels, pts, idx, neighbors, cluster_id, eps, min_samples):
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
    return [[i for i, l in enumerate(labels) if l == cid] for cid in range(1, cluster_id + 1)]

# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    def __init__(self):
        super().__init__("simple_scan_viz")

        # 파라미터
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("db_eps", 0.25)
        self.declare_parameter("db_min_samples", 5)
        self.declare_parameter("roi_min_dist", 2.00)
        self.declare_parameter("roi_max_dist", 3.00)
        self.declare_parameter("center_scale", 0.12)

        # 값 로드
        self.scan_topic     = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.marker_frame   = self.get_parameter("marker_frame_id").get_parameter_value().string_value
        self.tf_timeout     = float(self.get_parameter("tf_timeout").value)
        self.db_eps         = float(self.get_parameter("db_eps").value)
        self.db_min_samples = int(self.get_parameter("db_min_samples").value)
        self.roi_min_dist   = float(self.get_parameter("roi_min_dist").value)
        self.roi_max_dist   = float(self.get_parameter("roi_max_dist").value)
        self.center_scale   = float(self.get_parameter("center_scale").value)

        # TF 버퍼
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 인터페이스
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)

        self.get_logger().info(f"[init] Scan={self.scan_topic}, Frame={self.marker_frame}")

    def _lookup_latest(self, target_frame: str, source_frame: str):
        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, RclTime(),
                                             timeout=Duration(seconds=self.tf_timeout))
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T

    def _on_scan(self, scan: LaserScan):
        laser_frame = scan.header.frame_id or "laser"

        # TF 조회
        try:
            R_ml, T_ml = self._lookup_latest(self.marker_frame, laser_frame)
        except TransformException:
            return

        ang_min = scan.angle_min
        ang_inc = scan.angle_increment
        rmin = scan.range_min
        rmax = scan.range_max
        ranges = scan.ranges

        if not ranges or ang_inc == 0.0:
            return

        use_roi = self.roi_max_dist > self.roi_min_dist
        pts_map: List[Point2D] = []

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            if use_roi and (r < self.roi_min_dist or r > self.roi_max_dist):
                continue
            th = ang_min + i * ang_inc
            p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0])
            p_m = R_ml @ p_l + T_ml
            pts_map.append((float(p_m[0]), float(p_m[1])))

        if not pts_map:
            self._publish_clear()
            return

        clusters = dbscan(pts_map, eps=self.db_eps, min_samples=self.db_min_samples)
        centers = [
            Point(x=sum(pts_map[i][0] for i in idxs) / len(idxs),
                  y=sum(pts_map[i][1] for i in idxs) / len(idxs),
                  z=0.0)
            for idxs in clusters if idxs
        ]

        self._publish_centers(centers)

    def _publish_clear(self):
        arr = MarkerArray()
        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)
        self.pub.publish(arr)

    def _publish_centers(self, centers: List[Point]):
        arr = MarkerArray()

        # 중심 구(SPHERE_LIST)
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cluster_centers"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD  # 명시적으로 ADD 설정
        m.pose.orientation.w = 1.0  # 유효한 quaternion
        m.scale.x = self.center_scale
        m.scale.y = self.center_scale
        m.scale.z = self.center_scale
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0  # 노란색으로 변경 (더 잘 보임)
        m.color.a = 1.0  # 완전 불투명
        m.lifetime = Duration(seconds=0.5).to_msg()  # 0.5초 lifetime
        m.points.extend(centers)
        arr.markers.append(m)

        self.pub.publish(arr)

# -------------------------------
# 메인
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
