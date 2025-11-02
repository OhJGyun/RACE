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
from rclpy.duration import Duration
from rclpy.time import Time as RclTime

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, Float32, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException, LookupException, ConnectivityException, ExtrapolationException


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
        # TF localization (like map_controller)
        self.declare_parameter("tf_timeout", 0.3)

        # 장애물 정보
        self.declare_parameter("obstacles_topic", "/scan_viz/obstacles")

        # 레인 관련
        self.declare_parameter("lane_csv_paths", [""])
        self.declare_parameter("lane_topic", "/lane_selector/target_lane")
        self.declare_parameter("lookahead_distance", 12.0)
        self.declare_parameter("obstacle_clearance", 0.7)
        self.declare_parameter("forward_min_distance", 0.2)
        self.declare_parameter("min_switch_interval", 1.5)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("detection_hold_time", 0.05)

        # ✅ Local Planner 확장: 로컬 인덱스 범위 설정 (전방/후방 분리)
        self.declare_parameter("local_index_margin_forward", 100)   # 전방(앞쪽) 인덱스 범위
        self.declare_parameter("local_index_margin_backward", 30)   # 후방(뒤쪽) 인덱스 범위

        # 시각화
        self.declare_parameter("marker_frame_id", "map")

        # ------------------------------------------------------------------
        # 파라미터 로딩
        # ------------------------------------------------------------------
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)
        self.lane_topic = str(self.get_parameter("lane_topic").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.obstacle_clearance = float(self.get_parameter("obstacle_clearance").value)
        self.forward_min_distance = float(self.get_parameter("forward_min_distance").value)
        self.min_switch_interval = float(self.get_parameter("min_switch_interval").value)
        self.update_rate = float(self.get_parameter("update_rate_hz").value)
        self.detection_hold_time = max(0.0, float(self.get_parameter("detection_hold_time").value))
        self.marker_frame = str(self.get_parameter("marker_frame_id").value)

        # ✅ Local Planner 확장: 로컬 인덱스 범위 파라미터 로딩 (전방/후방 분리)
        self.local_index_margin_forward = int(self.get_parameter("local_index_margin_forward").value)
        self.local_index_margin_backward = int(self.get_parameter("local_index_margin_backward").value)

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
        # 레인별 장애물까지의 최소 lateral 거리 캐시 (중복 계산 방지)
        self.lane_min_lateral_distances: List[float] = [float('inf')] * len(self.lanes)

        # ✅ Local Planner 확장: 차량의 현재 레인 인덱스 캐시 (최적화)
        self.last_car_idx: Optional[int] = None

        # ------------------------------------------------------------------
        # TF2 초기화
        # ------------------------------------------------------------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # ROS 인터페이스
        # ------------------------------------------------------------------
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

        # ✅ Local Planner 확장: LaneSegment 퍼블리셔 추가 (Int32MultiArray 사용)
        # data[0] = lane_id, data[1] = start_idx, data[2] = end_idx
        self.pub_lane_segment = self.create_publisher(Int32MultiArray, "lane_selector/current_segment", 10)

        if self.update_rate > 0.0:
            self.eval_timer = self.create_timer(1.0 / self.update_rate, self._evaluate_lanes)
        else:
            self.eval_timer = None
        self.viz_timer = self.create_timer(1.0, self._publish_lane_visualization)

        self.get_logger().info(
            f"Lane selector node ready. Loaded {len(self.lanes)} lane(s)"
        )

    # ----------------------------------------------------------------------
    # TF를 통한 Pose 획득 (map -> base_link)
    # ----------------------------------------------------------------------
    def _get_current_pose_from_tf(self) -> Optional[Tuple[float, float, float]]:
        """Get current pose from TF (map->base_link)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                RclTime(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            # Convert quaternion to yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=1.0)
            return None

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
    # ✅ Local Planner 확장: 현재 레인 세그먼트 퍼블리시
    # ----------------------------------------------------------------------
    def _publish_current_segment(self) -> None:
        """
        현재 선택된 레인의 (lane_id, start_idx, end_idx)를 퍼블리시
        Local Planner처럼 동작하여 차량 위치 기준 로컬 세그먼트를 계산
        🔄 Circular track: 마지막 인덱스와 첫 인덱스가 연결됨
        """
        if self.pose is None or not self.lanes:
            return

        # 현재 레인이 없으면 기본값 0 (optimal lane) 사용
        if self.current_lane_idx is None:
            self.current_lane_idx = 0

        # 현재 레인 데이터
        lane = self.lanes[self.current_lane_idx]
        lane_points = lane.points
        lane_s = lane.arc_lengths
        num_points = len(lane_points)

        # 차량 위치
        px, py, _ = self.pose
        vehicle_pos = np.array([px, py], dtype=np.float64)

        # ✅ 최적화: 로컬 윈도우 탐색으로 현재 인덱스 찾기
        car_idx = self._nearest_index_local(
            lane_points, lane_s, vehicle_pos,
            last_idx=self.last_car_idx, window_size=100
        )

        # 인덱스 점프 감지 및 원인 분석
        if self.last_car_idx is not None:
            # Linear jump vs Circular jump 구분
            linear_jump = abs(car_idx - self.last_car_idx)
            circular_jump = min(linear_jump, num_points - linear_jump)

            if circular_jump > 10:  # 큰 점프 감지
                # 이전/현재 위치의 실제 좌표
                last_point = lane_points[self.last_car_idx]
                curr_point = lane_points[car_idx]

                # 실제 차량 이동 거리
                vehicle_moved = math.hypot(px - last_point[0], py - last_point[1])

                # 이전 인덱스와 현재 인덱스 사이의 경로 거리
                path_dist_forward = abs(lane_s[car_idx] - lane_s[self.last_car_idx])

                # 레인 변경 확인
                lane_changed = hasattr(self, '_last_segment_lane') and self._last_segment_lane != self.current_lane_idx

                self.get_logger().warn(
                    f"[JUMP] Large index jump! "
                    f"last_idx={self.last_car_idx} -> curr_idx={car_idx}, "
                    f"linear_jump={linear_jump}, circular_jump={circular_jump}, "
                    f"vehicle_moved={vehicle_moved:.3f}m, path_dist={path_dist_forward:.3f}m, "
                    f"lane_changed={lane_changed}, "
                    f"last_point=({last_point[0]:.2f},{last_point[1]:.2f}), "
                    f"curr_point=({curr_point[0]:.2f},{curr_point[1]:.2f}), "
                    f"vehicle_pos=({px:.2f},{py:.2f})"
                )

        self._last_segment_lane = self.current_lane_idx  # 레인 변경 추적용
        self.last_car_idx = car_idx  # 다음 탐색을 위해 캐시

        # 🔄 Circular track: 인덱스 범위 계산 (wrap-around 지원)
        # start_idx(후방)와 end_idx(전방)는 서로 다른 margin 적용
        start_idx = (car_idx - self.local_index_margin_backward) % num_points
        end_idx = (car_idx + self.local_index_margin_forward) % num_points

        # ✅ Int32MultiArray로 LaneSegment 정보 퍼블리시
        # data[0] = lane_id, data[1] = start_idx, data[2] = end_idx, data[3] = num_points (circular 판별용)
        msg = Int32MultiArray()
        msg.data = [int(self.current_lane_idx), int(start_idx), int(end_idx), int(num_points)]
        self.pub_lane_segment.publish(msg)

        # 세그먼트 정보 로깅
        self.get_logger().info(
            f"[SEGMENT] lane={self.current_lane_idx}, car_idx={car_idx}, "
            f"segment=[{start_idx}, {end_idx}], num_points={num_points}",
            throttle_duration_sec=1.0
        )

        # ✅ Segment 시각화 추가
        segment_marker = Marker()
        segment_marker.header.frame_id = self.marker_frame
        segment_marker.header.stamp = self.get_clock().now().to_msg()
        segment_marker.ns = "lane_segment"
        segment_marker.id = self.current_lane_idx
        segment_marker.type = Marker.LINE_STRIP
        segment_marker.action = Marker.ADD
        segment_marker.scale.x = 0.15
        segment_marker.color.r = 1.0
        segment_marker.color.g = 1.0
        segment_marker.color.b = 0.0
        segment_marker.color.a = 1.0

        # 인덱스 범위 구간만 시각화
        if start_idx < end_idx:
            seg_points = lane_points[start_idx:end_idx]
        else:
            seg_points = np.vstack((lane_points[start_idx:], lane_points[:end_idx]))

        for p in seg_points:
            segment_marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))

        # 퍼블리시
        marker_array = MarkerArray()
        marker_array.markers.append(segment_marker)
        self.pub_lane_markers.publish(marker_array)


        # DEBUG LOG (throttled)
        # self.get_logger().info(
        #     f"[LANE SEGMENT] lane={msg.data[0]}, car_idx={car_idx}, "
        #     f"segment=[{msg.data[1]}, {msg.data[2]})",
        #     throttle_duration_sec=1.0
        # )

    # ----------------------------------------------------------------------
    # 레인 평가 타이머
    # ----------------------------------------------------------------------
    def _evaluate_lanes(self) -> None:
        # Get current pose from TF
        self.pose = self._get_current_pose_from_tf()
        if self.pose is None or not self.lanes:
            return

        # ✅ Local Planner 확장: 매 주기마다 현재 레인 세그먼트 퍼블리시
        self._publish_current_segment()

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
            # Show initial lane selection with blocked lanes
            blocked_lanes = [str(i) for i, blocked in enumerate(blocked_flags) if blocked]
            blocked_str = ", ".join(blocked_lanes) if blocked_lanes else "none"
            self.get_logger().info(f"Lane selected: {desired_idx} (blocked: {blocked_str})")
            return

        if desired_idx == self.current_lane_idx:
            return

        if not self._can_switch():
            return

        prev = self.current_lane_idx
        self.current_lane_idx = desired_idx
        self.last_switch_time = now
        self._publish_lane(desired_idx)
        # Show lane change with blocked lanes
        blocked_lanes = [str(i) for i, blocked in enumerate(blocked_flags) if blocked]
        blocked_str = ", ".join(blocked_lanes) if blocked_lanes else "none"
        self.get_logger().info(f"Lane change: {prev} -> {desired_idx} (blocked: {blocked_str})")

    # ----------------------------------------------------------------------
    # 레인 차단 판별
    # ----------------------------------------------------------------------
    def _lane_blocked(self, lane: LaneData, lane_idx: int) -> bool:
        """
        레인이 장애물에 의해 blocked 되었는지 판별
        동시에 최소 lateral 거리를 self.lane_min_lateral_distances[lane_idx]에 저장
        """
        if not self.obstacles_np or self.pose is None:
            self.lane_min_lateral_distances[lane_idx] = float('inf')
            return False

        px, py, yaw = self.pose
        lane_points = lane.points
        lane_s = lane.arc_lengths
        track_length = max(lane.track_length, lane_s[-1], 1.0)

        car_idx = self._nearest_index(lane_points, np.array([px, py], dtype=np.float64))
        car_s = lane_s[car_idx]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        min_lateral = float('inf')
        is_blocked = False

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

            # 최소 lateral 거리 업데이트
            min_lateral = min(min_lateral, lateral)

            if lateral <= self.obstacle_clearance:
                is_blocked = True

        # 계산된 최소 lateral 거리를 캐시에 저장
        self.lane_min_lateral_distances[lane_idx] = min_lateral

        return is_blocked

    def _lane_blocked_confirmed(self, dt: float) -> List[bool]:
        flags: List[bool] = []
        for idx, lane in enumerate(self.lanes):
            raw_blocked = self._lane_blocked(lane, idx)  # lane_idx 전달
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

        # 현재 레인이 막혔으면, 가용한 레인들 중 장애물과의 lateral 거리가 가장 큰 레인 선택
        candidates: List[Tuple[float, float, int]] = []  # (lateral_dist, vehicle_dist, lane_idx)

        for idx, blocked in enumerate(blocked_flags):
            if blocked:
                continue

            # 이미 _lane_blocked()에서 계산된 최소 lateral 거리를 캐시에서 가져옴 (중복 계산 방지)
            min_lateral_dist = self.lane_min_lateral_distances[idx]

            # 차량과 레인의 거리 (보조 기준)
            vehicle_pos = np.array(self.pose[:2], dtype=np.float64)
            lane_point = self.lanes[idx].points[self._nearest_index(self.lanes[idx].points, vehicle_pos)]
            vehicle_dist = math.hypot(lane_point[0] - vehicle_pos[0], lane_point[1] - vehicle_pos[1])

            candidates.append((min_lateral_dist, vehicle_dist, idx))

        if not candidates:
            return current

        # 정렬: 1순위 장애물과의 lateral 거리가 큰 순, 2순위 차량과 가까운 순
        # lateral_dist를 음수로 만들어 큰 값이 먼저 오도록 정렬
        candidates.sort(key=lambda item: (-item[0], item[1]))

        selected_idx = candidates[0][2]

        return selected_idx

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

    # ✅ Local Planner 확장: arc_length 기반 로컬 윈도우 탐색으로 최적화
    def _nearest_index_local(self, points: np.ndarray, arc_lengths: np.ndarray, query: np.ndarray,
                             last_idx: Optional[int] = None, window_size: int = 50) -> int:
        """
        arc_length 기반 로컬 윈도우 탐색으로 nearest index를 찾음
        last_idx가 주어지면 그 주변만 탐색하여 성능 최적화
        🔄 Circular track: 트랙 끝에서 시작으로 wrap-around 지원
        """
        num_points = len(points)

        if last_idx is None or last_idx < 0 or last_idx >= num_points:
            # 초기화: 전체 탐색
            full_idx = self._nearest_index(points, query)
            self.get_logger().info(
                f"[INDEX] Full search: last_idx={last_idx}, found={full_idx}, total_points={num_points}",
                throttle_duration_sec=1.0
            )
            return full_idx

        # 🔄 Circular track: 윈도우가 경계를 넘어갈 수 있도록 인덱스 생성
        indices = []
        for i in range(-window_size, window_size + 1):
            idx = (last_idx + i) % num_points
            indices.append(idx)

        # 해당 인덱스들의 포인트만 추출
        local_points = points[indices]
        local_diffs = local_points - query
        local_dists = np.einsum("ij,ij->i", local_diffs, local_diffs)
        local_min_idx = int(np.argmin(local_dists))

        final_idx = indices[local_min_idx]

        # Circular distance 계산 (shortest path on circular track)
        index_diff = abs(final_idx - last_idx)
        index_jump = min(index_diff, num_points - index_diff)

        self.get_logger().info(
            f"[INDEX] Local search (circular): last={last_idx}, found={final_idx}, "
            f"jump={index_jump}, total_points={num_points}",
            throttle_duration_sec=1.0
        )

        return final_idx

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
