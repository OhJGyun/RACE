#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple LaserScan -> map visualization
- TF 조회는 항상 최신(latest) 사용
- Marker.header.stamp 는 기본적으로 LaserScan.header.stamp 사용(옵션)
"""

from __future__ import annotations
import math
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

class SimpleScanViz(Node):
    def __init__(self) -> None:
        super().__init__("simple_scan_viz")

        # ---- params ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_frame_id", "map")     # map / odom / base_link / laser 등
        self.declare_parameter("point_scale", 0.03)
        self.declare_parameter("tf_timeout", 0.2)
        # ⬇️ 추가: 마커 타임스탬프를 스캔 스탬프로 찍을지 여부
        self.declare_parameter("use_scan_stamp_for_markers", True)

        self.scan_topic   = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.marker_frame = self.get_parameter("marker_frame_id").get_parameter_value().string_value
        self.pt_scale     = float(self.get_parameter("point_scale").value)
        self.tf_timeout   = float(self.get_parameter("tf_timeout").value)
        self.use_scan_stamp = bool(self.get_parameter("use_scan_stamp_for_markers").value)

        # ---- TF ----
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- IO ----
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, "scan_viz/markers", 1)

        # ---- readiness flags ----
        self.clock_ready = False
        self.tf_ready    = False
        self._ready_timer = self.create_timer(0.1, self._check_ready)

        self.get_logger().info(
            f"[init] SimpleScanViz. scan={self.scan_topic}, frame={self.marker_frame} "
            f"(TF=latest, markers_stamp={'scan' if self.use_scan_stamp else 'now'})"
        )

    # wait for sim clock & TF
    def _check_ready(self):
        if not self.clock_ready:
            if self.get_clock().now().nanoseconds > 0:
                self.clock_ready = True
                self.get_logger().info("[ready] sim clock active")
            else:
                return
        if not self.tf_ready:
            if self.tf_buffer.can_transform(self.marker_frame, "laser", RclTime(),
                                            timeout=Duration(seconds=self.tf_timeout)):
                self.tf_ready = True
                self.get_logger().info(f"[ready] TF available: {self.marker_frame} <- laser")

    def _lookup_latest(self, source_frame: str):
        # 항상 최신으로만
        tf = self.tf_buffer.lookup_transform(self.marker_frame, source_frame, RclTime(),
                                             timeout=Duration(seconds=self.tf_timeout))
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.array([t.x, t.y, t.z], dtype=float)
        return R, T

    def _on_scan(self, scan: LaserScan):
        if not (self.clock_ready and self.tf_ready):
            return

        laser_frame = scan.header.frame_id or "laser"

        try:
            R_ml, T_ml = self._lookup_latest(laser_frame)
        except TransformException:
            return

        ang_min = float(scan.angle_min)
        ang_inc = float(scan.angle_increment)
        ranges  = scan.ranges
        if not ranges or ang_inc == 0.0:
            return

        pts_map: List[Point2D] = []
        rmin = float(scan.range_min); rmax = float(scan.range_max)

        for i, r in enumerate(ranges):
            r = float(r)
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            th = ang_min + i * ang_inc
            p_l = np.array([r * math.cos(th), r * math.sin(th), 0.0], dtype=float)
            p_m = R_ml @ p_l + T_ml
            pts_map.append((float(p_m[0]), float(p_m[1])))

        # 마커에 사용할 타임스탬프 선택
        stamp_for_markers: BuiltinTime = scan.header.stamp if self.use_scan_stamp else self.get_clock().now().to_msg()
        self._publish_points(pts_map, stamp_for_markers)

    def _publish_points(self, pts: Sequence[Point2D], stamp: BuiltinTime):
        arr = MarkerArray()

        m_clear = Marker()
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = stamp            # ⬅️ 스캔 스탬프(기본) 또는 now()
        m.ns = "scan"
        m.id = 0
        m.type = Marker.POINTS
        m.scale.x = self.pt_scale
        m.scale.y = self.pt_scale
        m.color.r = 0.2
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.95
        for x, y in pts:
            m.points.append(Point(x=x, y=y, z=0.0))
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
