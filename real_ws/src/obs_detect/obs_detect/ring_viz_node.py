#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimpleScanViz (LaserScan → map → 초경량 클러스터링 → 중심 시각화 + 정적/동적/노이즈 분류)

기능 요약
- LaserScan 입력을 map 좌표계로 변환(스캔 '시각'의 TF 사용: time-synced)
- FOV/ROI 필터 → O(N) 스캔라인 클러스터링 → 클러스터 중심 계산
- CSV로 읽은 outer/inner 링(outer 안 ∧ inner 밖) 필터링
- 슬라이딩 윈도우(초 단위) 히스토리로 '정적/동적/노이즈' 분류
  * 정적:  Rs(1.0m) 내에 커버리지시간 ≥ Ts(0.1s)  또는  프레임수 ≥ Ns(자동: ceil(f_eff*Ts))
  * 동적:  (정적 아님) AND Rd(2.0m) 내에 커버리지시간 ≥ Td(0.5s) 또는 프레임수 ≥ Nd(ceil(f_eff*Td))
  * 노이즈: 나머지
- MarkerArray 를 /scan_viz/markers 로 퍼블리시 (static=green, dynamic=red, noise=gray)

주의
- RViz Fixed Frame은 marker_frame_id(기본 "map")로 맞추세요.
- ros2 bag 재생 시 --clock 사용 권장. TF가 스캔 시각에 존재해야 합니다.
"""

from __future__ import annotations
import os, csv, math
from typing import List, Tuple, Sequence, Optional
from collections import deque

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


def in_ring(x: float, y: float, outer: Optional[Sequence[Point2D]], inner: Optional[Sequence[Point2D]]) -> bool:
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
    + 슬라이딩 윈도우 기반 정적/동적/노이즈 분류
    """

    def __init__(self):
        super().__init__("simple_scan_viz")

        # ----- 파라미터 선언 -----
        # 입력/출력/TF
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)

        # 전처리/표시
        self.declare_parameter("db_min_samples", 5)
        self.declare_parameter("roi_min_dist", 0.20)
        self.declare_parameter("roi_max_dist", 6.00)
        self.declare_parameter("center_scale", 0.12)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("fov_center_deg", 0.0)
        self.declare_parameter("outer_bound_csv", "")
        self.declare_parameter("inner_bound_csv", "")

        # 분류(윈도우/임계)
        self.declare_parameter("window_sec", 1.0)         # W: 최소 Td 이상 권장
        self.declare_parameter("lidar_hz", 40.0)          # 기대 Hz (fallback)
        self.declare_parameter("Ts", 0.1)                 # static coverage (s)
        self.declare_parameter("Td", 0.5)                 # dynamic coverage (s)
        self.declare_parameter("static_radius", 1.0)      # Rs (m)
        self.declare_parameter("dynamic_radius", 2.0)     # Rd (m)

        # ----- 로드 -----
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

        self.window_sec = float(self.get_parameter("window_sec").value)
        self.lidar_hz = float(self.get_parameter("lidar_hz").value)
        self.Ts = float(self.get_parameter("Ts").value)
        self.Td = float(self.get_parameter("Td").value)
        self.static_radius = float(self.get_parameter("static_radius").value)
        self.dynamic_radius = float(self.get_parameter("dynamic_radius").value)

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

        # 히스토리(최근 window_sec 동안): (stamp_sec: float, pts_xy: np.ndarray[N,2])
        self.history = deque()

        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}]m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°, "
            f"Rs={self.static_radius}m, Rd={self.dynamic_radius}m, "
            f"Ts={self.Ts}s, Td={self.Td}s, W={self.window_sec}s"
        )

    # ---------------------------
    # TF lookup @ 특정 시각 (스캔 시각)
    # ---------------------------
    def _lookup_at(self, target_frame: str, source_frame: str, t_ros: RclTime):
        if not self.tf_buffer.can_transform(
            target_frame, source_frame, t_ros, timeout=Duration(seconds=self.tf_timeout)
        ):
            raise TransformException(f"TF not available at time={t_ros.nanoseconds}")
        tf = self.tf_buffer.lookup_transform(
            target_frame, source_frame, t_ros, timeout=Duration(seconds=self.tf_timeout)
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

    def _make_marker(self, ns: str, rgba: Tuple[float, float, float, float], points: List[Point], stamp):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = stamp
        m.ns = ns
        # 같은 ns라도 RViz가 갱신하도록 id를 고정값 하나로 사용
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.center_scale
        m.scale.y = self.center_scale
        m.scale.z = self.center_scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime = Duration(seconds=0.5).to_msg()
        m.points.extend(points)
        return m

    def _publish_centers_classed(self, centers: List[Point], stamp, idx_s, idx_d, idx_n):
        arr = MarkerArray()
        if idx_s:
            arr.markers.append(self._make_marker("centers_static",  (0.0, 1.0, 0.0, 1.0), [centers[i] for i in idx_s], stamp))  # green
        if idx_d:
            arr.markers.append(self._make_marker("centers_dynamic", (1.0, 0.0, 0.0, 1.0), [centers[i] for i in idx_d], stamp))  # red
        if idx_n:
            arr.markers.append(self._make_marker("centers_noise",   (0.5, 0.5, 0.5, 0.6), [centers[i] for i in idx_n], stamp))  # gray
        if arr.markers:
            self.pub.publish(arr)
        else:
            self._publish_clear()

    # ---------------------------
    # 히스토리 관리 & 커버리지 계산
    # ---------------------------
    def _prune_history(self, now_sec: float):
        cut = now_sec - self.window_sec
        while self.history and self.history[0][0] < cut:
            self.history.pop left()  # type: ignore[attr-defined]

    # deque에는 popleft가 맞습니다. (IDE 헷갈림 방지용 별도 메서드)
    def pop left(self):  # noqa: E999  (문법 하이라이트 방지용 이름)
        pass

    def _coverage_seconds(self, hit_ts: np.ndarray) -> float:
        """
        간단 근사: (히트 프레임 수 - 1) * median(Δt)
        - 균등 샘플이 아니어도 robust
        """
        n = hit_ts.size
        if n <= 1:
            return 0.0
        t_sorted = np.sort(hit_ts)
        dts = np.diff(t_sorted)
        if dts.size == 0:
            return 0.0
        return float((n - 1) * float(np.median(dts)))

    # ---------------------------
    # 분류기 (하이브리드: 시간 OR 프레임)
    # ---------------------------
    def _classify_centers(self, centers_xy: np.ndarray, now_sec: float):
        M = centers_xy.shape[0]
        if M == 0:
            return [], [], []

        # 오래된 프레임 제거
        cut = now_sec - self.window_sec
        while self.history and self.history[0][0] < cut:
            self.history.popleft()

        # 유효 주파수 추정
        if len(self.history) >= 2:
            ts_all = np.array([t for (t, _) in self.history], dtype=float)
            span = float(ts_all[-1] - ts_all[0])
            total_frames = len(self.history)
            f_eff = (total_frames - 1) / span if span > 1e-6 else self.lidar_hz
        else:
            f_eff = self.lidar_hz

        Ns = int(math.ceil(f_eff * self.Ts))  # ~ ceil(40*0.1)=4
        Nd = int(math.ceil(f_eff * self.Td))  # ~ ceil(40*0.5)=20

        rs2 = self.static_radius * self.static_radius
        rd2 = self.dynamic_radius * self.dynamic_radius

        idx_s, idx_d, idx_n = [], [], []

        # 각 중심에 대해 윈도우 내 히트 프레임 시각 수집
        for i in range(M):
            c = centers_xy[i]  # (2,)
            hit_ts_s = []
            hit_ts_d = []
            for (t, pts) in self.history:
                if pts.size == 0:
                    continue
                dif = pts - c[None, :]
                d2 = np.einsum('ij,ij->i', dif, dif)
                if (d2 <= rs2).any():
                    hit_ts_s.append(t)
                if (d2 <= rd2).any():
                    hit_ts_d.append(t)

            cnt_s = len(hit_ts_s)
            cnt_d = len(hit_ts_d)
            cov_s = self._coverage_seconds(np.array(hit_ts_s, dtype=float)) if cnt_s > 0 else 0.0
            cov_d = self._coverage_seconds(np.array(hit_ts_d, dtype=float)) if cnt_d > 0 else 0.0

            is_static  = (cov_s >= self.Ts) or (cnt_s >= Ns)
            is_dynamic = (not is_static) and ((cov_d >= self.Td) or (cnt_d >= Nd))

            if is_static:
                idx_s.append(i)
            elif is_dynamic:
                idx_d.append(i)
            else:
                idx_n.append(i)

        return idx_s, idx_d, idx_n

    # ---------------------------
    # 스캔 콜백
    # ---------------------------
    def _on_scan(self, scan: LaserScan):
        laser_frame = scan.header.frame_id or "laser"

        # 스캔 '시각'의 TF 조회 (time-synced)
        scan_time_ros = RclTime.from_msg(scan.header.stamp)
        try:
            R_ml, T_ml = self._lookup_at(self.marker_frame, laser_frame, scan_time_ros)
        except TransformException as e:
            self.get_logger().warn(f"TF not available: {e}")
            return

        ang_min, ang_inc = scan.angle_min, scan.angle_increment
        rmin, rmax = scan.range_min, scan.range_max
        ranges = scan.ranges
        if not ranges or ang_inc == 0.0:
            self._publish_clear()
            return

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
            # 히스토리에도 비어있는 프레임을 적재(시간축 유지)
            stamp_sec = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            self.history.append((stamp_sec, np.empty((0, 2), dtype=float)))
            cut = stamp_sec - self.window_sec
            while self.history and self.history[0][0] < cut:
                self.history.popleft()
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
        centers_ring = [p for p in centers if in_ring(
            p.x, p.y, self.outer_poly or None, self.inner_poly or None
        )]

        # 5) 히스토리 적재 (프레임 단위)
        stamp_sec = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if centers_ring:
            pts_xy = np.array([[p.x, p.y] for p in centers_ring], dtype=float)
        else:
            pts_xy = np.empty((0, 2), dtype=float)
        self.history.append((stamp_sec, pts_xy))
        cut = stamp_sec - self.window_sec
        while self.history and self.history[0][0] < cut:
            self.history.popleft()

        # 6) 분류 & 퍼블리시 (스캔 시각으로 타임스탬프)
        if centers_ring:
            centers_xy = np.array([[p.x, p.y] for p in centers_ring], dtype=float)
            now_sec = stamp_sec
            idx_s, idx_d, idx_n = self._classify_centers(centers_xy, now_sec)
            self._publish_centers_classed(centers_ring, stamp=scan.header.stamp,
                                          idx_s=idx_s, idx_d=idx_d, idx_n=idx_n)
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
