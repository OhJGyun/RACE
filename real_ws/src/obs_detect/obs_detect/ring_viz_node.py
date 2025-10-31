#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaserScan → map 변환 → 초경량 클러스터링 → 중심만 RViz 시각화
+ CSV로 읽은 outer/inner 경계 사이(outer 안 ∧ inner 밖)에 있는 중심만 통과
+ 직전 프레임과 비교해서 static / dynamic 구분
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
# 각도 정규화
# -------------------------------
def _ang_norm(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# -------------------------------
# 초경량 스캔라인 기반 클러스터링 (O(N))
# -------------------------------
def fast_scanline_clusters(r_list, th_list, min_samples=5, eps0=0.12, k=0.06):
    """
    연속한 빔 간 거리로 클러스터링 (정렬된 스캔 전제)
    eps0: 최소 연결 임계 (m)
    k: 거리 비례 계수 (멀리 있는 빔 허용 폭)
    """
    n = len(r_list)
    if n == 0:
        return []

    clusters = []
    cur = [0]
    for i in range(1, n):
        r0, r1 = r_list[i - 1], r_list[i]
        dth = th_list[i] - th_list[i - 1]
        dij = math.sqrt(r0 * r0 + r1 * r1 - 2.0 * r0 * r1 * math.cos(dth))
        link = max(eps0, k * min(r0, r1))
        if dij <= link:
            cur.append(i)
        else:
            if len(cur) >= min_samples:
                clusters.append(cur)
            cur = [i]
    if len(cur) >= min_samples:
        clusters.append(cur)
    return clusters


# -------------------------------
# 노드
# -------------------------------
class SimpleScanViz(Node):
    """
    LaserScan → map 변환 → 초경량 클러스터링 → 중심 표시
    + CSV outer/inner 링 필터
    + 직전 프레임과 비교해 정적 / 동적 분류
    """

    def __init__(self):
        super().__init__("simple_scan_viz")

        # 파라미터
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("db_min_samples", 5)
        self.declare_parameter("roi_min_dist", 0.20)
        self.declare_parameter("roi_max_dist", 6.00)
        self.declare_parameter("center_scale", 0.12)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("fov_center_deg", 0.0)
        self.declare_parameter("outer_bound_csv", "")
        self.declare_parameter("inner_bound_csv", "")

        # (신규) 동적/정적 판정 관련 파라미터
        # match_radius: 같은 물체로 매칭했다고 볼 최대 위치 차이 (m)
        # dyn_vel_thresh: 이 속도(m/s) 이상이면 dynamic으로 간주
        self.declare_parameter("match_radius", 0.3)
        self.declare_parameter("dyn_vel_thresh", 0.4)

        # 로드
        self.scan_topic = self.get_parameter("scan_topic").value
        self.marker_frame = self.get_parameter("marker_frame_id").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.db_min_samples = int(self.get_parameter("db_min_samples").value)
        self.roi_min_dist = float(self.get_parameter("roi_min_dist").value)
        self.roi_max_dist = float(self.get_parameter("roi_max_dist").value)
        self.center_scale = float(self.get_parameter("center_scale").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.fov_center_deg = float(self.get_parameter("fov_center_deg").value)
        self.outer_csv = self.get_parameter("outer_bound_csv").value
        self.inner_csv = self.get_parameter("inner_bound_csv").value

        self.match_radius = float(self.get_parameter("match_radius").value)
        self.dyn_vel_thresh = float(self.get_parameter("dyn_vel_thresh").value)

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

        # (신규) 지난 프레임 상태 저장용
        self.prev_centers: List[Tuple[float, float]] = []
        self.prev_stamp: Optional[float] = None  # sec 단위 float

        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}]m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°"
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

    def _make_marker(self, centers: List[Point],
                     r: float, g: float, b: float,
                     ns: str, mid: int):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.center_scale
        m.scale.y = self.center_scale
        m.scale.z = self.center_scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.lifetime = Duration(seconds=0.5).to_msg()
        m.points.extend(centers)
        return m

    def _publish_centers_two_colors(self,
                                    static_pts: List[Point],
                                    dynamic_pts: List[Point]):
        arr = MarkerArray()

        # static: 초록 (0,1,0)
        if static_pts:
            arr.markers.append(
                self._make_marker(static_pts, 0.0, 1.0, 0.0,
                                  "cluster_static", 0)
            )

        # dynamic: 빨강 (1,0,0)
        if dynamic_pts:
            arr.markers.append(
                self._make_marker(dynamic_pts, 1.0, 0.0, 0.0,
                                  "cluster_dynamic", 1)
            )

        # 아무것도 없으면 clear
        if not arr.markers:
            self._publish_clear()
        else:
            self.pub.publish(arr)

    # ---------------------------
    # (신규) 동적 / 정적 분류
    # ---------------------------
    def _classify_motion(self,
                          centers_ring: List[Point],
                          stamp_sec: float
                          ) -> Tuple[List[Point], List[Point]]:
        """
        centers_ring: 이번 프레임에서 링 필터 통과한 Center들
        stamp_sec: 이번 프레임 time(sec)

        prev_centers / prev_stamp 와 비교해서
        - 매칭된 점의 속도가 dyn_vel_thresh 이상이면 dynamic
        - 아니면 static
        매칭 안되면 (새로 튀어나온 애) 일단 dynamic으로 분류하는 쪽으로 둠.
        """
        if self.prev_stamp is None or not self.prev_centers:
            # 첫 프레임이면 다 static 취급해도 되고,
            # 혹은 다 dynamic 취급해도 되고. 여기선 static으로 시작.
            return centers_ring, []

        dt = stamp_sec - self.prev_stamp
        if dt <= 1e-3:
            # 시간 차 없으면 속도 못 구하니까 전부 static으로.
            return centers_ring, []

        static_pts: List[Point] = []
        dynamic_pts: List[Point] = []

        # 매칭 방식: 현재 center마다 prev_centers 중 가장 가까운 점 찾기
        for p in centers_ring:
            cx, cy = p.x, p.y

            best_d2 = None
            best_prev = None
            for (px, py) in self.prev_centers:
                dx = cx - px
                dy = cy - py
                d2 = dx*dx + dy*dy
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best_prev = (px, py)

            if best_d2 is None:
                # 매칭 불가 -> dynamic 가정
                dynamic_pts.append(p)
                continue

            dist = math.sqrt(best_d2)
            if dist > self.match_radius:
                # 너무 멀면 같은 물체로 안 본다 -> 새로운 애, dynamic 가정
                dynamic_pts.append(p)
                continue

            # 같은 애로 보고 속도 계산
            speed = dist / dt  # m/s
            if speed >= self.dyn_vel_thresh:
                dynamic_pts.append(p)
            else:
                static_pts.append(p)

        return static_pts, dynamic_pts

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
        ranges = scan.ranges
        if not ranges or ang_inc == 0.0:
            self._publish_clear()
            return

        # timestamp (sec) for velocity calc
        now_msg = self.get_clock().now().to_msg()
        now_sec = now_msg.sec + now_msg.nanosec * 1e-9

        fov_rad = math.radians(self.fov_deg)
        fov_center = math.radians(self.fov_center_deg)
        half_fov = 0.5 * fov_rad
        use_roi = self.roi_max_dist > self.roi_min_dist

        # 1) ROI/FOV 필터링
        r_list, th_list = [], []
        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            if use_roi and (r < self.roi_min_dist or r > self.roi_max_dist):
                continue
            th = ang_min + i * ang_inc
            dth = _ang_norm(th - fov_center)
            if abs(dth) > half_fov:
                continue
            r_list.append(r)
            th_list.append(th)

        if not r_list:
            self._publish_clear()
            # prev 갱신은 안 함 (관측 없음)
            return

        # 2) 초경량 클러스터링
        idx_clusters = fast_scanline_clusters(
            r_list, th_list,
            min_samples=self.db_min_samples,
            eps0=0.12, k=0.06
        )

        # 3) 중심 계산 및 map 변환
        centers: List[Point] = []
        for idxs in idx_clusters:
            sx = sy = 0.0
            for j in idxs:
                r, th = r_list[j], th_list[j]
                p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0])
                p_m = R_ml @ p_l + T_ml
                sx += p_m[0]; sy += p_m[1]
            n = len(idxs)
            centers.append(Point(x=sx/n, y=sy/n, z=0.0))

        # 4) CSV 링 필터
        centers_ring = [
            p for p in centers
            if in_ring(p.x, p.y,
                       self.outer_poly or None,
                       self.inner_poly or None)
        ]

        # 5) 정적/동적 분류
        static_pts, dynamic_pts = self._classify_motion(centers_ring, now_sec)

        # 6) 퍼블리시
        if static_pts or dynamic_pts:
            self._publish_centers_two_colors(static_pts, dynamic_pts)
        else:
            self._publish_clear()

        # 7) 이번 프레임을 다음 프레임 비교용으로 저장
        self.prev_centers = [(p.x, p.y) for p in centers_ring]
        self.prev_stamp = now_sec


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
