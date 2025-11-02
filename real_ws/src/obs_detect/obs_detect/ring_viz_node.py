#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaserScan → map 변환 → 초경량 클러스터링 → 중심만 RViz 시각화
+ CSV로 읽은 outer/inner 경계 사이(outer 안 ∧ inner 밖)에 있는 중심만 통과
+ 프레임 간 트래킹으로 정적/동적 분류
+ PoseArray에 z=0(정적) / z=1(동적) 담아서 lane_selector에서 바로 사용
"""

from __future__ import annotations
import os, csv, math
from typing import List, Tuple, Sequence, Optional, Dict
import numpy as np

import rclpy
from rclpy.node import Node
        # Duration은 wall time / lifetime 등에 사용
from rclpy.duration import Duration
        # RclTime은 TF lookup에 "latest"로 넘길 때 사용
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
    + 간단한 트래커 기반 정적/동적 분리
    + PoseArray(z=0 정적 / z=1 동적) 퍼블리시
    """

    def __init__(self):
        super().__init__("simple_scan_viz")

        # ---------------------------
        # (원래 코드에 있던) 파라미터 선언
        # ---------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("db_min_samples", 8)
        self.declare_parameter("roi_min_dist", 0.00)
        self.declare_parameter("roi_max_dist", 3.00)
        self.declare_parameter("center_scale", 0.12)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("fov_center_deg", 0.0)
        self.declare_parameter("outer_bound_csv", "/home/ircv7/RACE/bound/1031_1/outer_bound.csv")
        self.declare_parameter("inner_bound_csv", "/home/ircv7/RACE/bound/1031_1/inner_bound.csv")

        # ---------------------------
        # (추가) 트래커 관련 파라미터 선언
        # ---------------------------
        # 같은 위치 근처로 여러 프레임 동안 반복 관측되고
        # 속도가 충분히 느리면 "정적"이라고 간주
        self.declare_parameter("static_radius", 0.25)        # 같은 물체로 본다고 허용하는 반경 [m]
        self.declare_parameter("static_min_hits", 5)          # 정적으로 인정하려면 최소 몇 번이나 다시 봤는지
        self.declare_parameter("static_max_age", 3.0)         # 마지막으로 본 뒤로 유지시키는 시간 [s]
        self.declare_parameter("static_vel_thresh", 0.08)     # 평균속도 [m/s] 이하면 정적
        self.declare_parameter("vel_ema_alpha", 0.3)          # 속도 EMA 필터 알파

        # ---------------------------
        # 로드
        # ---------------------------
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

        # (추가) 트래커 관련 로드
        self.static_radius     = float(self.get_parameter("static_radius").value)
        self.static_min_hits   = int(self.get_parameter("static_min_hits").value)
        self.static_max_age    = float(self.get_parameter("static_max_age").value)
        self.static_vel_thresh = float(self.get_parameter("static_vel_thresh").value)
        self.vel_ema_alpha     = float(self.get_parameter("vel_ema_alpha").value)

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
        self.pub_obstacles = self.create_publisher(PoseArray, "scan_viz/obstacles", 10)

        # (추가) 트래커 상태
        # track_id -> {
        #   "anchor_x","anchor_y": 기준 위치(고정점 느낌)
        #   "last_x","last_y":     마지막 관측 위치
        #   "t_last","t0":         마지막/처음 관측된 시간[s]
        #   "hits":                누적 관측 횟수
        #   "v_avg":               EMA 속도 [m/s]
        # }
        self._tracks: Dict[int, dict] = {}
        self._next_track_id = 1

        # 로깅 최적화 (같은 개수 계속 안 찍게)
        self._prev_count_logged = 0

        self.get_logger().info(
            f"[init] scan={self.scan_topic}, frame={self.marker_frame}, "
            f"ROI=[{self.roi_min_dist},{self.roi_max_dist}]m, "
            f"FOV={self.fov_deg}°@{self.fov_center_deg}°, "
            f"static_radius={self.static_radius}, static_vel_thresh={self.static_vel_thresh}"
        )

    # ---------------------------
    # 내부 유틸: 거리
    # ---------------------------
    def _dist(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x1 - x2, y1 - y2)

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
    # 오래된 트랙 정리
    # ---------------------------
    def _prune_tracks(self, now_sec: float):
        dead_ids = [
            tid for tid, tr in self._tracks.items()
            if (now_sec - tr["t_last"]) > self.static_max_age
        ]
        for tid in dead_ids:
            self._tracks.pop(tid, None)

    # ---------------------------
    # 이 트랙이 "정적"이라고 볼 수 있나?
    # ---------------------------
    def _is_static_track(self, tr: dict) -> bool:
        # 충분히 여러 번 관측돼야 함
        if tr["hits"] < self.static_min_hits:
            return False
        # anchor에서 많이 안 움직였고, 속도도 느리면 정적
        near_anchor = (
            self._dist(tr["last_x"], tr["last_y"], tr["anchor_x"], tr["anchor_y"])
            <= self.static_radius
        )
        slow_enough = (tr["v_avg"] <= self.static_vel_thresh)
        return near_anchor and slow_enough

    # ---------------------------
    # 이번 프레임에서 뽑은 center들을 트래커에 반영
    # ---------------------------
    def _ingest_centers_to_tracks(self,
                                  centers_xy: List[Tuple[float, float]],
                                  now_sec: float):
        for (cx, cy) in centers_xy:
            # 기존 트랙 중 anchor 근처(=same object로 추정되는 것) 찾기
            best_id = None
            best_d = 1e9
            for tid, tr in self._tracks.items():
                d = self._dist(cx, cy, tr["anchor_x"], tr["anchor_y"])
                if d <= self.static_radius and d < best_d:
                    best_d = d
                    best_id = tid

            if best_id is None:
                # 새로운 트랙 생성
                tid_new = self._next_track_id
                self._next_track_id += 1
                self._tracks[tid_new] = {
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
                v_inst = math.hypot(dx, dy) / dt  # 순간 속도

                a = self.vel_ema_alpha
                tr["v_avg"] = a * v_inst + (1.0 - a) * tr["v_avg"]

                tr["hits"]   += 1
                tr["last_x"]  = cx
                tr["last_y"]  = cy
                tr["t_last"]  = now_sec

    # ---------------------------
    # 퍼블리시 헬퍼
    # ---------------------------
    def _publish_clear(self):
        # RViz에서 기존 마커 지우기
        arr = MarkerArray()
        m = Marker(); m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub.publish(arr)

        # lane_selector 쪽에도 빈 PoseArray 쏴줘야 상위 로직이 "장애물 없다"로 인식 가능
        empty_pa = PoseArray()
        empty_pa.header.frame_id = self.marker_frame
        empty_pa.header.stamp = self.get_clock().now().to_msg()
        self.pub_obstacles.publish(empty_pa)

    def _publish_centers(self,
                         centers: List[Point],
                         static_mask: List[bool]):
        """
        centers: map 좌표계에서의 장애물 중심들
        static_mask[i] == True이면 정적, False면 동적
        """
        # ---------- MarkerArray (노란 점들) ----------
        arr = MarkerArray()

        # 먼저 기존 마커 전부 삭제 신호
        m_del = Marker()
        m_del.action = Marker.DELETEALL
        arr.markers.append(m_del)

        if centers:
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

        self.pub.publish(arr)

        # ---------- PoseArray (lane_selector용) ----------
        # position.x, position.y = 장애물 map 좌표
        # position.z = 0.0(정적) / 1.0(동적)
        pose_arr = PoseArray()
        pose_arr.header.frame_id = self.marker_frame
        pose_arr.header.stamp = self.get_clock().now().to_msg()
        for pt, is_static in zip(centers, static_mask):
            pose = Pose()
            pose.position.x = pt.x
            pose.position.y = pt.y
            pose.position.z = 0.0 if is_static else 1.0
            pose.orientation.w = 1.0
            pose_arr.poses.append(pose)
        self.pub_obstacles.publish(pose_arr)

    # ---------------------------
    # 스캔 콜백
    # ---------------------------
    def _on_scan(self, scan: LaserScan):
        laser_frame = scan.header.frame_id or "laser"

        # 최신 TF (marker_frame <- laser)
        try:
            R_ml, T_ml = self._lookup_latest(self.marker_frame, laser_frame)
        except TransformException as e:
            self.get_logger().warn(f"TF not available: {e}")
            return

        ang_min, ang_inc = scan.angle_min, scan.angle_increment
        rmin, rmax = scan.range_min, scan.range_max
        ranges = scan.ranges
        if not ranges or ang_inc == 0.0:
            return

        fov_rad = math.radians(self.fov_deg)
        fov_center = math.radians(self.fov_center_deg)
        half_fov = 0.5 * fov_rad
        use_roi = self.roi_max_dist > self.roi_min_dist

        # 1) ROI/FOV 필터링
        r_list: List[float] = []
        th_list: List[float] = []
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
            # 빈 경우 → 비워주고 끝
            self._publish_clear()
            self._prev_count_logged = 0
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self._prune_tracks(now_sec)
            return

        # 2) 초경량 클러스터링
        idx_clusters = fast_scanline_clusters(
            r_list, th_list,
            min_samples=self.db_min_samples,
            eps0=0.12, k=0.06
        )

        if not idx_clusters:
            self._publish_clear()
            self._prev_count_logged = 0
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self._prune_tracks(now_sec)
            return

        # 3) 각 클러스터 중심 계산 후 map 프레임으로 변환
        centers: List[Point] = []
        for idxs in idx_clusters:
            sx = 0.0
            sy = 0.0
            for j in idxs:
                r, th = r_list[j], th_list[j]
                # laser frame point
                p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0])
                # map frame point
                p_m = R_ml @ p_l + T_ml
                sx += p_m[0]
                sy += p_m[1]
            n = len(idxs)
            centers.append(Point(x=sx/n, y=sy/n, z=0.0))

        # 4) CSV 링 필터 (outer 안 ∧ inner 밖)
        centers_ring: List[Point] = [
            p for p in centers
            if in_ring(p.x, p.y,
                       self.outer_poly or None,
                       self.inner_poly or None)
        ]

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        # 오래 안 보인 트랙은 정리
        self._prune_tracks(now_sec)

        if not centers_ring:
            # 링 안 장애물 없으면 클리어
            self._publish_clear()
            if self._prev_count_logged != 0:
                self.get_logger().info("[CLEAR] No obstacles in ring")
            self._prev_count_logged = 0
            return

        # 5) 트래커 업데이트 (정적/동적 판별용)
        centers_xy = [(p.x, p.y) for p in centers_ring]
        self._ingest_centers_to_tracks(centers_xy, now_sec)

        static_mask: List[bool] = []
        for (cx, cy) in centers_xy:
            # 이 center와 가장 anchor가 가까운 트랙 찾고,
            # 그 트랙이 static인지 보고 결정
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

        # 6) 로그 + 퍼블리시
        n_total = len(centers_ring)
        n_static = sum(static_mask)
        n_moving = n_total - n_static

        if n_total != self._prev_count_logged:
            self.get_logger().info(
                f"[OBSTACLE DETECTED] total={n_total}, static={n_static}, moving={n_moving}"
            )
            self._prev_count_logged = n_total

        # RViz (MarkerArray) + lane_selector용 PoseArray(z=0/1)
        self._publish_centers(centers_ring, static_mask)


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
