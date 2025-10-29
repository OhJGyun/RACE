#!/usr/bin/env python3
"""
Obstacle detection (TF 기반 map 프레임 일원화 + CORE/RING 래스터 파이프라인)

- CSV 경계(outer/inner)는 map 프레임 좌표라고 가정
- TF로 map←laser 변환을 조회해 스캔점을 map 프레임으로 투영
- 이후 모든 필터링/클러스터링/시각화는 map 프레임에서 수행
"""

from __future__ import annotations

import csv, glob, math, os, re
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations as tft
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# ------------- 유틸/래스터 (기존과 동일) -------------
Point2D = Tuple[float, float]

def load_world_csv(path: str) -> List[Point2D]:
    pts: List[Point2D] = []
    path = os.path.expanduser(path)
    with open(path, "r") as f:
        for row in csv.reader(f):
            if not row: continue
            pts.append((float(row[0]), float(row[1])))
    if pts and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts

def load_multi_inner(spec: str) -> List[List[Point2D]]:
    spec = os.path.expanduser(spec)
    parts = [p for p in re.split(r"[,\s;]+", spec)] if any(ch in spec for ch in ",; ") else [spec]
    paths: List[str] = []
    for p in parts:
        paths.extend(glob.glob(p) if any(ch in p for ch in "*?[]") else [p])
    inners: List[List[Point2D]] = []
    for p in paths:
        if os.path.exists(p):
            poly = load_world_csv(p)
            if len(poly) >= 3: inners.append(poly)
    return inners

def point_in_polygon(x: float, y: float, poly: Sequence[Point2D], include_boundary: bool) -> bool:
    inside = False
    n = len(poly)
    if n < 3: return False
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        if (y1 > y) != (y0 > y):
            t = (y - y0) / (y1 - y0 + 1e-12)
            xin = x0 + (x1 - x0) * t
            if (x <= xin) if include_boundary else (x < xin):
                inside = not inside
        x0, y0 = x1, y1
    return inside

def dist_point_to_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx*vx + vy*vy
    if vv == 0.0: return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx*vx + wy*vy) / vv))
    cx, cy = ax + t*vx, ay + t*vy
    return math.hypot(px - cx, py - cy)

def min_dist_to_poly(px: float, py: float, poly: Sequence[Point2D]) -> float:
    if len(poly) < 2: return float("inf")
    dmin = float("inf")
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        dmin = min(dmin, dist_point_to_segment(px, py, x0, y0, x1, y1))
        x0, y0 = x1, y1
    return dmin

def binary_erosion_box(mask: np.ndarray, win: int) -> np.ndarray:
    H, W = mask.shape
    if win <= 1: return (mask > 0).astype(np.uint8)
    I = np.pad(mask.astype(np.int32), ((1,0),(1,0))).cumsum(0).cumsum(1)
    h, w = H - win + 1, W - win + 1
    if h <= 0 or w <= 0: return np.zeros_like(mask, dtype=np.uint8)
    A = I[0:h, 0:w]; B = I[0:h, win:W+1]; C = I[win:H+1, 0:w]; D = I[win:H+1, win:W+1]
    S = D - B - C + A
    k = (win // 2)
    out = np.zeros_like(mask, dtype=np.uint8)
    out[k:H-k, k:W-k] = (S == win*win).astype(np.uint8)
    return out

def build_raster_bounds(outer, inners, map_resolution=0.05, ring_width_m=0.5, padding_m=1.0, include_boundary=False):
    xs = [p[0] for p in outer]; ys = [p[1] for p in outer]
    xmin, xmax = min(xs)-padding_m, max(xs)+padding_m
    ymin, ymax = min(ys)-padding_m, max(ys)+padding_m
    W = int(math.ceil((xmax-xmin)/map_resolution))
    H = int(math.ceil((ymax-ymin)/map_resolution))
    def world_to_grid(wx, wy):
        return int((wx - xmin)/map_resolution), int((wy - ymin)/map_resolution)
    def grid_to_world(ix, iy):
        return xmin + (ix+0.5)*map_resolution, ymin + (iy+0.5)*map_resolution
    inner_mask = np.zeros((H, W), dtype=np.uint8)
    for iy in range(H):
        wy = ymin + (iy + 0.5) * map_resolution
        for ix in range(W):
            wx = xmin + (ix + 0.5) * map_resolution
            inner_mask[iy, ix] = 1 if any(
                len(poly) >= 3 and point_in_polygon(wx, wy, poly, include_boundary)
                for poly in inners
            ) else 0
    k = max(1, int(round(ring_width_m / map_resolution)))
    core_mask = binary_erosion_box(inner_mask, 2*k+1)
    ring_mask = (inner_mask == 1) & (core_mask == 0)
    return inner_mask, core_mask.astype(np.uint8), ring_mask.astype(np.uint8), world_to_grid, grid_to_world

# ------------- 데이터 구조 -------------
@dataclass
class Cluster:
    points: List[Point2D]
    @property
    def center(self) -> Point2D:
        n = len(self.points) or 1
        return (sum(x for x,_ in self.points)/n, sum(y for _,y in self.points)/n)

# ------------- 노드 -------------
class ObsDetectNode(Node):
    def __init__(self) -> None:
        super().__init__("obs_detect_node_tf")

        # 파라미터
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("outer_csv", "outer_bound_world.csv")  # map 프레임
        self.declare_parameter("inner_csvs", "inner_bound_world.csv") # map 프레임
        self.declare_parameter("include_boundary", False)

        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("laser_frame_id", "")  # 기본은 LaserScan.header.frame_id 사용
        self.declare_parameter("tf_timeout_sec", 0.05)

        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("ring_width_m", 0.5)
        self.declare_parameter("padding_m", 1.0)

        self.declare_parameter("roi_deg", 90.0)
        self.declare_parameter("range_max", 12.0)

        self.declare_parameter("boundary_margin", 0.50)
        self.declare_parameter("ring_boundary_margin", 0.20)
        self.declare_parameter("cluster_epsilon", 0.35)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("cluster_confirm_points", 6)

        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_scale", 0.35)

        # 로딩
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.outer_csv = str(self.get_parameter("outer_csv").value)
        self.inner_spec = str(self.get_parameter("inner_csvs").value)
        self.include_boundary = bool(self.get_parameter("include_boundary").value)

        self.map_frame = str(self.get_parameter("map_frame_id").value)
        self.laser_frame_param = str(self.get_parameter("laser_frame_id").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self.map_resolution = float(self.get_parameter("map_resolution").value)
        self.ring_width_m = float(self.get_parameter("ring_width_m").value)
        self.padding_m = float(self.get_parameter("padding_m").value)

        self.roi_rad = math.radians(float(self.get_parameter("roi_deg").value))
        self.range_max = float(self.get_parameter("range_max").value)

        self.boundary_margin = float(self.get_parameter("boundary_margin").value)
        self.ring_boundary_margin = float(self.get_parameter("ring_boundary_margin").value)
        self.cluster_epsilon = float(self.get_parameter("cluster_epsilon").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_confirm_points = int(self.get_parameter("cluster_confirm_points").value)

        self.marker_frame = str(self.get_parameter("marker_frame_id").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)

        # 경계/래스터(map 프레임)
        self.outer = load_world_csv(self.outer_csv)
        self.inners = load_multi_inner(self.inner_spec)
        (self.inner_mask,
         self.core_mask,
         self.ring_mask,
         self.world_to_grid,
         self.grid_to_world) = build_raster_bounds(
            self.outer, self.inners,
            map_resolution=self.map_resolution,
            ring_width_m=self.ring_width_m,
            padding_m=self.padding_m,
            include_boundary=self.include_boundary,
        )

        H, W = self.inner_mask.shape
        self.get_logger().info(f"Bounds loaded. Raster {W}x{H}px @ {self.map_resolution:.3f} m/px")

        # TF
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub_obstacle_markers = self.create_publisher(MarkerArray, "obs_detect/markers", 1)
        self.pub_obstacle_centers = self.create_publisher(PoseArray, "obs_detect/obstacles", 10)

    # -------- TF 헬퍼: map←laser 변환 행렬 2D --------
    def _lookup_map_from_laser(self, target_time: Time, laser_frame_id: str):
        try:
            ts = self.tf_buffer.lookup_transform(
                self.map_frame,   # target
                laser_frame_id,   # source
                target_time,
                rclpy.duration.Duration(seconds=self.tf_timeout)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        t = ts.transform.translation
        q = ts.transform.rotation
        # 2D 평면 가정: yaw만 사용
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x, t.y, c, s)  # (tx, ty, cos_yaw, sin_yaw)

    # -------- LaserScan 콜백 --------
    def _on_scan(self, scan: LaserScan) -> None:
        # Laser 프레임 결정: 파라미터 우선, 없으면 메시지의 frame_id
        laser_frame = self.laser_frame_param if self.laser_frame_param else scan.header.frame_id
        if not laser_frame:
            self.get_logger().error("Laser frame id is unknown.")
            return

        # TF (map←laser) at scan time
        tf2_time = Time.from_msg(scan.header.stamp) if scan.header.stamp.sec or scan.header.stamp.nanosec else Time()
        tf2 = self._lookup_map_from_laser(tf2_time, laser_frame)
        if tf2 is None:
            return
        tx, ty, c, s = tf2

        # 스캔 변환 및 CORE/RING 정책
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return

        max_range = min(self.range_max, scan.range_max)
        ang = scan.angle_min

        core_pts_map: List[Point2D] = []
        ring_pts_map: List[Point2D] = []
        accepted_pts_map: List[Point2D] = []

        roi_skip = boundary_filtered = inner_reject = 0

        for r in scan.ranges:
            if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > max_range:
                ang += scan.angle_increment
                continue

            # ROI: laser 프레임 전방 ±roi
            if abs(ang) > self.roi_rad:
                roi_skip += 1
                ang += scan.angle_increment
                continue

            # laser→map : [x_l, y_l] → [x_m, y_m]
            lx, ly = r * math.cos(ang), r * math.sin(ang)
            wx = tx + c*lx - s*ly
            wy = ty + s*lx + c*ly

            # 벡터 폴리곤 1차 필터(map 프레임)
            if not point_in_polygon(wx, wy, self.outer, self.include_boundary):
                ang += scan.angle_increment
                continue
            if not any(len(poly) >= 3 and point_in_polygon(wx, wy, poly, self.include_boundary) for poly in self.inners):
                inner_reject += 1
                ang += scan.angle_increment
                continue

            # 래스터 CORE/RING 분류
            ix, iy = self.world_to_grid(wx, wy)
            in_bounds = (0 <= ix < self.inner_mask.shape[1]) and (0 <= iy < self.inner_mask.shape[0])
            if not in_bounds or self.inner_mask[iy, ix] == 0:
                inner_reject += 1
                ang += scan.angle_increment
                continue

            is_core = self.core_mask[iy, ix] == 1
            is_ring = (not is_core) and (self.ring_mask[iy, ix] == 1)

            # 경계 마진
            dist_outer = min_dist_to_poly(wx, wy, self.outer)
            dist_inner = min(
                (min_dist_to_poly(wx, wy, poly) for poly in self.inners if len(poly) >= 3),
                default=float("inf"),
            )

            if is_core:
                if dist_outer < self.boundary_margin or dist_inner < self.boundary_margin:
                    boundary_filtered += 1
                else:
                    core_pts_map.append((wx, wy)); accepted_pts_map.append((wx, wy))
            elif is_ring:
                ring_margin = max(self.boundary_margin, self.ring_boundary_margin)
                if dist_outer < ring_margin or dist_inner < ring_margin:
                    boundary_filtered += 1
                else:
                    ring_pts_map.append((wx, wy)); accepted_pts_map.append((wx, wy))
            else:
                boundary_filtered += 1

            ang += scan.angle_increment

        clusters = self._cluster_points(accepted_pts_map, self.cluster_epsilon, self.cluster_min_points)
        confirmed = [c for c in clusters if len(c.points) >= self.cluster_confirm_points]
        centers_map = [c.center for c in confirmed]

        if confirmed:
            self.get_logger().info(
                f"[OBSTACLE] confirmed={len(confirmed)} acc={len(accepted_pts_map)} "
                f"roi_skip={roi_skip} inner_reject={inner_reject} boundary_filtered={boundary_filtered}"
            )

        self._publish_markers(core_pts_map, ring_pts_map, confirmed)
        self._publish_centers_map(centers_map)

    # -------- 클러스터 --------
    def _cluster_points(self, points: Sequence[Point2D], eps: float, min_pts: int) -> List[Cluster]:
        n = len(points)
        if n == 0: return []
        parent = list(range(n))
        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        def union(a,b):
            ra, rb = find(a), find(b)
            if ra != rb: parent[ra] = rb
        eps2 = eps*eps
        for i in range(n):
            xi, yi = points[i]
            for j in range(i+1, n):
                dx, dy = xi - points[j][0], yi - points[j][1]
                if dx*dx + dy*dy <= eps2: union(i,j)
        buckets: dict[int, List[int]] = {}
        for i in range(n):
            buckets.setdefault(find(i), []).append(i)
        out: List[Cluster] = []
        for ids in buckets.values():
            if len(ids) >= min_pts:
                out.append(Cluster([points[k] for k in ids]))
        return out

    # -------- 시각화/출력 (map 프레임) --------
    def _publish_markers(self, core_pts, ring_pts, clusters: Iterable[Cluster]) -> None:
        arr = MarkerArray()
        m_clear = Marker(); m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        now = self.get_clock().now().to_msg()

        def make_points(id_, pts, rgba):
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.header.stamp = now
            m.ns = "obs_detect"
            m.id = id_
            m.type = Marker.POINTS
            m.scale.x = 0.06; m.scale.y = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            for x,y in pts:
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            return m

        arr.markers.append(make_points(0, core_pts, (0.1, 0.55, 1.0, 0.9)))  # 파랑
        arr.markers.append(make_points(1, ring_pts, (1.0, 0.85, 0.2, 0.9))) # 노랑

        m_cent = Marker()
        m_cent.header.frame_id = self.marker_frame
        m_cent.header.stamp = now
        m_cent.ns = "obs_detect"
        m_cent.id = 2
        m_cent.type = Marker.SPHERE_LIST
        m_cent.scale.x = self.marker_scale
        m_cent.scale.y = self.marker_scale
        m_cent.scale.z = self.marker_scale
        m_cent.color.r, m_cent.color.g, m_cent.color.b, m_cent.color.a = (1.0, 0.2, 0.2, 0.95)
        for cl in clusters:
            cx, cy = cl.center
            m_cent.points.append(Point(x=float(cx), y=float(cy), z=0.0))
        arr.markers.append(m_cent)

        self.pub_obstacle_markers.publish(arr)

    def _publish_centers_map(self, centers_map: Sequence[Point2D]) -> None:
        arr = PoseArray()
        arr.header.frame_id = self.marker_frame  # 기본 map
        arr.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in centers_map:
            p = Pose()
            p.position.x = float(cx); p.position.y = float(cy); p.position.z = 0.0
            p.orientation.w = 1.0
            arr.poses.append(p)
        self.pub_obstacle_centers.publish(arr)

# ------------- main -------------
def main(args=None):
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
