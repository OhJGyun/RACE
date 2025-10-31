#!/usr/bin/env python3
"""
Lane selector node for the lane_selector package.

감지된 장애물을 이용한 레인 선택 및 RViz 시각화
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray


################################################################################
# 공통 유틸
################################################################################

Point2D = Tuple[float, float]


################################################################################
# 레인 데이터 구조체
################################################################################

@dataclass
class LaneData:
    name: str
    points: np.ndarray
    arc_lengths: np.ndarray
    track_length: float


################################################################################
# 레인 선택 노드
################################################################################

class LaneSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("lane_selector_node")

        # ------------------------------------------------------------------
        # 파라미터 선언
        # ------------------------------------------------------------------
        # 차량 자세
        self.declare_parameter("odom_topic", "/odom")

        # 장애물 정보
        self.declare_parameter("obstacles_topic", "obs_detect/obstacles")

        # 레인 관련
        self.declare_parameter("lane_csv_paths", [""])
        self.declare_parameter("lane_topic", "/lane_selector/target_lane")
        self.declare_parameter("lookahead_distance", 12.0)
        self.declare_parameter("obstacle_clearance", 0.7)
        self.declare_parameter("forward_min_distance", 0.2)
        self.declare_parameter("min_switch_interval", 1.5)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("detection_hold_time", 0.05)

        # 시각화
        self.declare_parameter("marker_frame_id", "map")

        # ------------------------------------------------------------------
        # 파라미터 로딩
        # ------------------------------------------------------------------
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)
        self.lane_topic = str(self.get_parameter("lane_topic").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.obstacle_clearance = float(self.get_parameter("obstacle_clearance").value)
        self.forward_min_distance = float(self.get_parameter("forward_min_distance").value)
        self.min_switch_interval = float(self.get_parameter("min_switch_interval").value)
        self.update_rate = float(self.get_parameter("update_rate_hz").value)
        self.detection_hold_time = max(0.0, float(self.get_parameter("detection_hold_time").value))
        self.marker_frame = str(self.get_parameter("marker_frame_id").value)

        raw_paths = self.get_parameter("lane_csv_paths").value
        self.lane_paths: List[str] = [p for p in raw_paths if isinstance(p, str) and p]
        if not self.lane_paths:
            raise RuntimeError("lane_csv_paths 파라미터가 비어 있습니다.")

        # ------------------------------------------------------------------
        # 레인 데이터 로딩
        # ------------------------------------------------------------------
        self.lanes = self._load_lanes(self.lane_paths)
        if not self.lanes:
            raise RuntimeError("레인 CSV를 하나도 불러오지 못했습니다.")

        # ------------------------------------------------------------------
        # 상태 변수 초기화
        # ------------------------------------------------------------------
        self.pose: Optional[Tuple[float, float, float]] = None
        self.obstacles_np: List[np.ndarray] = []
        self.current_lane_idx: Optional[int] = None
        self.last_switch_time: Optional[float] = None
        self.last_eval_time: Optional[float] = None
        self.detection_durations: List[float] = [0.0] * len(self.lanes)

        # ------------------------------------------------------------------
        # ROS 인터페이스
        # ------------------------------------------------------------------
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.sub_obstacles = self.create_subscription(
            PoseArray, self.obstacles_topic, self._on_obstacles, 10
        )

        self.pub_lane_target = self.create_publisher(Int32, self.lane_topic, 10)
        self.pub_lane_markers = self.create_publisher(MarkerArray, "lane_selector/markers", 10)
        self.pub_lane_paths = [
            self.create_publisher(Path, f"lane_selector/lane_{idx}", 10)
            for idx in range(len(self.lanes))
        ]
        self.pub_lane_posearrays = [
            self.create_publisher(PoseArray, f"lane_selector/lane_{idx}/poses", 10)
            for idx in range(len(self.lanes))
        ]
        self.pub_selected_lane = self.create_publisher(Path, "lane_selector/selected_lane", 10)

        if self.update_rate > 0.0:
            self.eval_timer = self.create_timer(1.0 / self.update_rate, self._evaluate_lanes)
        else:
            self.eval_timer = None
        self.viz_timer = self.create_timer(1.0, self._publish_lane_visualization)

        self.get_logger().info(
            f"Lane selector node ready. Loaded {len(self.lanes)} lane(s)"
        )

    # ----------------------------------------------------------------------
    # Odometry 콜백
    # ----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x, y, yaw)

    # ----------------------------------------------------------------------
    # 장애물 콜백
    # ----------------------------------------------------------------------
    def _on_obstacles(self, msg: PoseArray) -> None:
        self.obstacles_np = []
        for pose in msg.poses:
            self.obstacles_np.append(
                np.array([pose.position.x, pose.position.y], dtype=np.float64)
            )

    # ----------------------------------------------------------------------
    # 레인 평가 타이머
    # ----------------------------------------------------------------------
    def _evaluate_lanes(self) -> None:
        if self.pose is None or not self.lanes:
            return

        now = self._now()
        if self.last_eval_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.last_eval_time)
        self.last_eval_time = now

        if self.detection_hold_time > 0.0 and dt > 0.0:
            effective_dt = min(dt, self.detection_hold_time * 0.5)
        else:
            effective_dt = dt

        blocked_flags = self._lane_blocked_confirmed(effective_dt)
        desired_idx = self._choose_lane(blocked_flags)
        if desired_idx is None:
            return

        if self.current_lane_idx is None:
            self.current_lane_idx = desired_idx
            self._publish_lane(desired_idx)
            self.get_logger().info(
                f"🛣️ 레인 선택: {desired_idx} ({self.lanes[desired_idx].name})"
            )
            return

        if desired_idx == self.current_lane_idx:
            return

        if not self._can_switch():
            return

        prev = self.current_lane_idx
        self.current_lane_idx = desired_idx
        self.last_switch_time = now
        self._publish_lane(desired_idx)
        self.get_logger().info(
            f"🛣️ 레인 변경: {prev} ({self.lanes[prev].name}) -> "
            f"{desired_idx} ({self.lanes[desired_idx].name})"
        )

    # ----------------------------------------------------------------------
    # 레인 차단 판별
    # ----------------------------------------------------------------------
    def _lane_blocked(self, lane: LaneData) -> bool:
        if not self.obstacles_np or self.pose is None:
            return False

        px, py, yaw = self.pose
        lane_points = lane.points
        lane_s = lane.arc_lengths
        track_length = max(lane.track_length, lane_s[-1], 1.0)

        car_idx = self._nearest_index(lane_points, np.array([px, py], dtype=np.float64))
        car_s = lane_s[car_idx]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for obs in self.obstacles_np:
            dx = float(obs[0] - px)
            dy = float(obs[1] - py)
            forward = dx * cos_yaw + dy * sin_yaw
            if forward < self.forward_min_distance:
                continue

            obs_idx = self._nearest_index(lane_points, obs)
            obs_s = lane_s[obs_idx]
            diff_s = obs_s - car_s
            if diff_s < 0.0:
                diff_s += track_length
            if diff_s < 0.0 or diff_s > self.lookahead_distance:
                continue

            lateral = math.hypot(
                lane_points[obs_idx, 0] - obs[0],
                lane_points[obs_idx, 1] - obs[1],
            )
            if lateral <= self.obstacle_clearance:
                return True

        return False

    def _lane_blocked_confirmed(self, dt: float) -> List[bool]:
        flags: List[bool] = []
        for idx, lane in enumerate(self.lanes):
            raw_blocked = self._lane_blocked(lane)
            duration = self.detection_durations[idx]
            if raw_blocked:
                duration = min(self.detection_hold_time, duration + dt)
            else:
                duration = max(0.0, duration - dt)
            self.detection_durations[idx] = duration
            flags.append(duration >= self.detection_hold_time)
        return flags

    # ----------------------------------------------------------------------
    # 레인 선택 & 퍼블리시
    # ----------------------------------------------------------------------
    def _choose_lane(self, blocked_flags: Sequence[bool]) -> Optional[int]:
        lane_count = len(blocked_flags)
        if lane_count == 0:
            return None

        # 최적 레인(0번)이 비어있으면 항상 유지
        if not blocked_flags[0]:
            return 0

        current = self.current_lane_idx if self.current_lane_idx is not None else 0
        current = max(0, min(current, lane_count - 1))
        if not blocked_flags[current]:
            return current

        candidates: List[Tuple[float, int]] = []
        for idx, blocked in enumerate(blocked_flags):
            if blocked:
                continue
            vehicle_pos = np.array(self.pose[:2], dtype=np.float64)
            lane_point = self.lanes[idx].points[self._nearest_index(self.lanes[idx].points, vehicle_pos)]
            dist = math.hypot(lane_point[0] - vehicle_pos[0], lane_point[1] - vehicle_pos[1])
            candidates.append((dist, idx))

        if not candidates:
            return current

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _publish_lane(self, lane_idx: int) -> None:
        msg = Int32()
        msg.data = int(lane_idx)
        self.pub_lane_target.publish(msg)

    # ----------------------------------------------------------------------
    # 레인 시각화
    # ----------------------------------------------------------------------
    def _publish_lane_visualization(self) -> None:
        if not self.lanes:
            return

        now = self.get_clock().now().to_msg()
        colors = [
            (0.0, 1.0, 0.0, 0.85),
            (0.0, 0.5, 1.0, 0.85),
            (1.0, 0.5, 0.0, 0.85),
            (0.8, 0.0, 0.8, 0.85),
        ]

        markers = MarkerArray()
        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        markers.markers.append(m_clear)

        for idx, lane in enumerate(self.lanes):
            r, g, b, a = colors[idx % len(colors)]

            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = now
            marker.ns = "lane_selector"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.08 if idx != self.current_lane_idx else 0.16
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0 if idx == self.current_lane_idx else a
            marker.pose.orientation.w = 1.0
            for point in lane.points:
                marker.points.append(Point(x=float(point[0]), y=float(point[1]), z=0.0))
            markers.markers.append(marker)

            path_msg = Path()
            path_msg.header.frame_id = self.marker_frame
            path_msg.header.stamp = now

            pose_array = PoseArray()
            pose_array.header = path_msg.header

            for pose_index, point in enumerate(lane.points):
                yaw = self._estimate_yaw(lane.points, pose_index)
                qz, qw = self._yaw_to_quaternion_simple(yaw)

                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = float(point[0])
                pose_stamped.pose.position.y = float(point[1])
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
                path_msg.poses.append(pose_stamped)

                pose_simple = Pose()
                pose_simple.position.x = float(point[0])
                pose_simple.position.y = float(point[1])
                pose_simple.orientation.z = qz
                pose_simple.orientation.w = qw
                pose_array.poses.append(pose_simple)

            if len(lane.points) > 1:
                first = lane.points[0]
                yaw = self._estimate_yaw(lane.points, 0)
                qz, qw = self._yaw_to_quaternion_simple(yaw)

                closing_pose = PoseStamped()
                closing_pose.header = path_msg.header
                closing_pose.pose.position.x = float(first[0])
                closing_pose.pose.position.y = float(first[1])
                closing_pose.pose.position.z = 0.0
                closing_pose.pose.orientation.z = qz
                closing_pose.pose.orientation.w = qw
                path_msg.poses.append(closing_pose)

                closing_simple = Pose()
                closing_simple.position.x = float(first[0])
                closing_simple.position.y = float(first[1])
                closing_simple.orientation.z = qz
                closing_simple.orientation.w = qw
                pose_array.poses.append(closing_simple)

            if idx < len(self.pub_lane_paths):
                self.pub_lane_paths[idx].publish(path_msg)
            if idx < len(self.pub_lane_posearrays) and pose_array.poses:
                self.pub_lane_posearrays[idx].publish(pose_array)

        self.pub_lane_markers.publish(markers)

        if self.current_lane_idx is not None and 0 <= self.current_lane_idx < len(self.lanes):
            lane = self.lanes[self.current_lane_idx]
            selected_path = Path()
            selected_path.header.frame_id = self.marker_frame
            selected_path.header.stamp = now
            for pose_index, point in enumerate(lane.points):
                yaw = self._estimate_yaw(lane.points, pose_index)
                qz, qw = self._yaw_to_quaternion_simple(yaw)

                pose_stamped = PoseStamped()
                pose_stamped.header = selected_path.header
                pose_stamped.pose.position.x = float(point[0])
                pose_stamped.pose.position.y = float(point[1])
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
                selected_path.poses.append(pose_stamped)

            if len(lane.points) > 1:
                first = lane.points[0]
                yaw = self._estimate_yaw(lane.points, 0)
                qz, qw = self._yaw_to_quaternion_simple(yaw)

                pose_stamped = PoseStamped()
                pose_stamped.header = selected_path.header
                pose_stamped.pose.position.x = float(first[0])
                pose_stamped.pose.position.y = float(first[1])
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
                selected_path.poses.append(pose_stamped)

            self.pub_selected_lane.publish(selected_path)

    # ----------------------------------------------------------------------
    # 레인 헬퍼
    # ----------------------------------------------------------------------
    @staticmethod
    def _nearest_index(points: np.ndarray, query: np.ndarray) -> int:
        diffs = points - query
        dists = np.einsum("ij,ij->i", diffs, diffs)
        return int(np.argmin(dists))

    @staticmethod
    def _estimate_yaw(points: np.ndarray, idx: int) -> float:
        n = len(points)
        if n < 2:
            return 0.0
        prev_idx = (idx - 1) % n
        next_idx = (idx + 1) % n
        prev_pt = points[prev_idx]
        next_pt = points[next_idx]
        dx = next_pt[0] - prev_pt[0]
        dy = next_pt[1] - prev_pt[1]
        if dx == 0.0 and dy == 0.0:
            return 0.0
        return math.atan2(dy, dx)

    @staticmethod
    def _yaw_to_quaternion_simple(yaw: float) -> Tuple[float, float]:
        half = 0.5 * yaw
        return math.sin(half), math.cos(half)

    def _can_switch(self) -> bool:
        if self.last_switch_time is None:
            return True
        return (self._now() - self.last_switch_time) >= self.min_switch_interval

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_lanes(self, paths: Sequence[str]) -> List[LaneData]:
        lanes: List[LaneData] = []
        for idx, path in enumerate(paths):
            try:
                pts = self._load_lane_csv(path)
            except Exception as exc:
                self.get_logger().error(f"레인 CSV 로드 실패 ({path}): {exc}")
                continue

            if len(pts) < 2:
                self.get_logger().warn(f"레인 {path}에 유효한 점이 2개 미만입니다.")
                continue

            points = np.asarray(pts, dtype=np.float64)
            arc = np.zeros(points.shape[0], dtype=np.float64)
            for i in range(1, len(points)):
                arc[i] = arc[i - 1] + math.hypot(
                    points[i, 0] - points[i - 1, 0],
                    points[i, 1] - points[i - 1, 1],
                )
            track_length = arc[-1] if arc[-1] > 0.0 else float(len(points))

            name = os.path.basename(path)
            lanes.append(LaneData(name=name, points=points, arc_lengths=arc, track_length=track_length))
            self.get_logger().info(
                f"Lane {idx} '{name}' 로드 ({len(points)} pts, length {track_length:.2f} m)"
            )

        return lanes

    @staticmethod
    def _load_lane_csv(path: str) -> List[Point2D]:
        points: List[Point2D] = []
        path = os.path.expanduser(path)
        with open(path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                except ValueError:
                    continue
                points.append((x, y))
        return points


################################################################################
# main
################################################################################

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
