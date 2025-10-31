#!/usr/bin/env python3
"""
Obstacle detection node for real vehicle (ROS2 Humble).
Based on sim obs_detect logic but using TF-based localization and publishing to /scan_viz/markers.
"""

import math
import csv
import glob
import os
import re
from dataclasses import dataclass
from typing import List, Tuple, Optional, Sequence, Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from transforms3d.euler import quat2euler


################################################################################
# 공통 유틸
################################################################################

Point2D = Tuple[float, float]


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


def load_multi_inner(spec: str) -> List[List[Point2D]]:
    """여러 내측 경계를 한꺼번에 불러온다 (글롭/구분자 지원)."""
    spec = os.path.expanduser(spec)
    parts = re.split(r"[,\s;]+", spec)
    paths = []
    for p in parts:
        if not p:
            continue
        if any(ch in p for ch in "*?[]"):
            paths.extend(glob.glob(p))
        else:
            paths.append(p)

    inners = []
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


def dist_point_to_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def min_dist_to_poly(px, py, poly):
    if len(poly) < 2:
        return float("inf")
    dmin = float("inf")
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        d = dist_point_to_segment(px, py, x0, y0, x1, y1)
        dmin = min(dmin, d)
        x0, y0 = x1, y1
    return dmin


################################################################################
# 장애물 클러스터 구조체
################################################################################

@dataclass
class Cluster:
    points: List[Point2D]

    @property
    def center(self) -> Point2D:
        sx = sum(p[0] for p in self.points)
        sy = sum(p[1] for p in self.points)
        n = len(self.points)
        return sx / n, sy / n


################################################################################
# 장애물 감지 노드 (실차용)
################################################################################

class ObsDetectRealNode(Node):
    def __init__(self):
        super().__init__('obs_detect_real_node')

        # ------------------------------
        # Parameters
        # ------------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("outer_csv", "/home/ircv7/RACE/bound/1030/outer_bound_world.csv")
        self.declare_parameter("inner_csvs", "/home/ircv7/RACE/bound/1030/inner_bound_world.csv")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("roi_deg", 90.0)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("laser_yaw_offset_deg", 0.0)
        self.declare_parameter("boundary_margin", 0.5)
        self.declare_parameter("cluster_epsilon", 0.35)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("cluster_confirm_points", 6)
        self.declare_parameter("marker_scale", 0.35)
        self.declare_parameter("tf_timeout", 0.3)

        # ------------------------------
        # Load parameters
        # ------------------------------
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.outer_csv = str(self.get_parameter("outer_csv").value)
        self.inner_spec = str(self.get_parameter("inner_csvs").value)
        self.marker_frame = str(self.get_parameter("marker_frame_id").value)
        self.roi_rad = math.radians(float(self.get_parameter("roi_deg").value))
        self.range_max = float(self.get_parameter("range_max").value)
        self.laser_yaw = math.radians(float(self.get_parameter("laser_yaw_offset_deg").value))
        self.boundary_margin = float(self.get_parameter("boundary_margin").value)
        self.cluster_epsilon = float(self.get_parameter("cluster_epsilon").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_confirm_points = int(self.get_parameter("cluster_confirm_points").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        # ------------------------------
        # Load boundaries
        # ------------------------------
        self.outer = load_world_csv(self.outer_csv)
        self.inners = load_multi_inner(self.inner_spec)
        self.get_logger().info(f"Loaded boundaries: outer={len(self.outer)}, inner={len(self.inners)}")

        # ------------------------------
        # TF setup
        # ------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------
        # ROS interfaces
        # ------------------------------
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/scan_viz/markers", 1)
        self.pub_centers = self.create_publisher(PoseArray, "/scan_viz/obstacles", 10)

        self.get_logger().info("✅ ObsDetectRealNode initialized (TF-based pose, publishing to /scan_viz/markers)")

    # ----------------------------------------------------------------------
    # Pose 획득 (TF 기반)
    # ----------------------------------------------------------------------
    def get_current_pose_from_tf(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time().to_msg(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            return (x, y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed for map->base_link", throttle_duration_sec=1.0)
            return None

    # ----------------------------------------------------------------------
    # LiDAR Scan Callback
    # ----------------------------------------------------------------------
    def _on_scan(self, scan: LaserScan) -> None:
        pose = self.get_current_pose_from_tf()
        if pose is None:
            return

        px, py, psi = pose
        ang_min = scan.angle_min
        ang_inc = scan.angle_increment
        n = len(scan.ranges)
        if n == 0:
            return

        max_range = min(self.range_max, scan.range_max)
        free_points: List[Point2D] = []

        for i in range(n):
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > max_range:
                continue

            angle = ang_min + i * ang_inc
            if abs(angle) > self.roi_rad:
                continue

            world_angle = psi + self.laser_yaw + angle
            wx = px + r * math.cos(world_angle)
            wy = py + r * math.sin(world_angle)

            if not point_in_polygon(wx, wy, self.outer, False):
                continue
            in_inner = any(point_in_polygon(wx, wy, poly, False) for poly in self.inners)
            if in_inner:
                continue

            dist_outer = min_dist_to_poly(wx, wy, self.outer)
            dist_inner = min((min_dist_to_poly(wx, wy, poly) for poly in self.inners), default=float("inf"))
            if dist_outer < self.boundary_margin or dist_inner < self.boundary_margin:
                continue

            free_points.append((wx, wy))

        clusters = self._cluster_points(free_points, self.cluster_epsilon, self.cluster_min_points)
        confirmed = [c for c in clusters if len(c.points) >= self.cluster_confirm_points]
        centers = [c.center for c in confirmed]

        if confirmed:
            self.get_logger().info(f"[OBSTACLE] detected {len(confirmed)} clusters")

        self._publish_markers(free_points, confirmed)
        self._publish_centers(centers)

    # ----------------------------------------------------------------------
    # DBSCAN-style clustering
    # ----------------------------------------------------------------------
    def _cluster_points(self, points: Sequence[Point2D], epsilon: float, min_points: int) -> List[Cluster]:
        n = len(points)
        if n == 0:
            return []
        parent = list(range(n))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        eps2 = epsilon * epsilon
        for i in range(n):
            xi, yi = points[i]
            for j in range(i + 1, n):
                xj, yj = points[j]
                if (xi - xj) ** 2 + (yi - yj) ** 2 <= eps2:
                    union(i, j)

        buckets = {}
        for idx in range(n):
            root = find(idx)
            buckets.setdefault(root, []).append(idx)

        clusters = []
        for idxs in buckets.values():
            if len(idxs) < min_points:
                continue
            clusters.append(Cluster(points=[points[k] for k in idxs]))
        return clusters

    # ----------------------------------------------------------------------
    # Visualization
    # ----------------------------------------------------------------------
    def _publish_markers(self, points: Sequence[Point2D], clusters: Iterable[Cluster]):
        arr = MarkerArray()
        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        now = self.get_clock().now().to_msg()

        # Raw points
        m_pts = Marker()
        m_pts.header.frame_id = self.marker_frame
        m_pts.header.stamp = now
        m_pts.ns = "obs_points"
        m_pts.id = 0
        m_pts.type = Marker.POINTS
        m_pts.scale.x = 0.06
        m_pts.scale.y = 0.06
        m_pts.color.r = 0.2
        m_pts.color.g = 0.6
        m_pts.color.b = 1.0
        m_pts.color.a = 0.9
        for x, y in points:
            m_pts.points.append(Point(x=float(x), y=float(y), z=0.0))
        arr.markers.append(m_pts)

        # Cluster centers
        m_centers = Marker()
        m_centers.header.frame_id = self.marker_frame
        m_centers.header.stamp = now
        m_centers.ns = "obs_centers"
        m_centers.id = 1
        m_centers.type = Marker.SPHERE_LIST
        m_centers.scale.x = self.marker_scale
        m_centers.scale.y = self.marker_scale
        m_centers.scale.z = self.marker_scale
        m_centers.color.r = 1.0
        m_centers.color.g = 0.3
        m_centers.color.b = 0.3
        m_centers.color.a = 0.95
        for c in clusters:
            cx, cy = c.center
            m_centers.points.append(Point(x=float(cx), y=float(cy), z=0.0))
        arr.markers.append(m_centers)

        self.pub_markers.publish(arr)

    def _publish_centers(self, centers: Sequence[Point2D]):
        arr = PoseArray()
        arr.header.frame_id = self.marker_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in centers:
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.orientation.w = 1.0
            arr.poses.append(pose)
        self.pub_centers.publish(arr)


################################################################################
# main
################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = ObsDetectRealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
