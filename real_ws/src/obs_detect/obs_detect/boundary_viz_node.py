#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Boundary Visualization Node
Reads CSV boundary files and visualizes them in RViz as LINE_STRIP markers
"""

import os
import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class BoundaryVizNode(Node):
    def __init__(self):
        super().__init__('boundary_viz_node')

        # Parameters - use same defaults as ring_viz_optimized
        self.declare_parameter('inner_bound_csv', '/home/ircv7/RACE/bound/1102/1.0_1.0/inner_bound.csv')
        self.declare_parameter('outer_bound_csv', '/home/ircv7/RACE/bound/1102/1.0_1.0/outer_bound.csv')
        self.declare_parameter('marker_frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('line_width', 0.05)
        self.declare_parameter('z_height', 0.0)

        # Get parameters
        self.inner_csv = self.get_parameter('inner_bound_csv').get_parameter_value().string_value
        self.outer_csv = self.get_parameter('outer_bound_csv').get_parameter_value().string_value
        self.frame_id = self.get_parameter('marker_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.z_height = self.get_parameter('z_height').get_parameter_value().double_value

        # Load boundaries
        self.inner_points = self._load_csv(self.inner_csv)
        self.outer_points = self._load_csv(self.outer_csv)

        # Publisher
        self.pub = self.create_publisher(MarkerArray, 'boundary_markers', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_markers)

        self.get_logger().info(f'Boundary Viz Node initialized')
        self.get_logger().info(f'  Inner points: {len(self.inner_points)}')
        self.get_logger().info(f'  Outer points: {len(self.outer_points)}')
        self.get_logger().info(f'  Frame: {self.frame_id}')

    def _load_csv(self, csv_path: str) -> List[Tuple[float, float]]:
        """Load boundary points from CSV file"""
        points = []

        if not os.path.exists(csv_path):
            self.get_logger().error(f'CSV file not found: {csv_path}')
            return points

        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            points.append((x, y))
                        except ValueError:
                            continue

            self.get_logger().info(f'Loaded {len(points)} points from {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV {csv_path}: {e}')

        return points

    def _create_line_marker(
        self,
        marker_id: int,
        namespace: str,
        points: List[Tuple[float, float]],
        color: ColorRGBA
    ) -> Marker:
        """Create a LINE_STRIP marker from boundary points"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale.x = self.line_width  # Line width

        # Color
        marker.color = color

        # Lifetime (0 = forever)
        marker.lifetime = Duration(seconds=0).to_msg()

        # Points
        for x, y in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.z_height
            marker.points.append(p)

        # Close the loop by adding first point at the end
        if points:
            p = Point()
            p.x = points[0][0]
            p.y = points[0][1]
            p.z = self.z_height
            marker.points.append(p)

        return marker

    def _publish_markers(self):
        """Publish boundary markers"""
        marker_array = MarkerArray()

        # Inner boundary (red)
        if self.inner_points:
            inner_color = ColorRGBA()
            inner_color.r = 1.0
            inner_color.g = 0.0
            inner_color.b = 0.0
            inner_color.a = 0.8

            inner_marker = self._create_line_marker(
                marker_id=0,
                namespace='inner_boundary',
                points=self.inner_points,
                color=inner_color
            )
            marker_array.markers.append(inner_marker)

        # Outer boundary (blue)
        if self.outer_points:
            outer_color = ColorRGBA()
            outer_color.r = 0.0
            outer_color.g = 0.0
            outer_color.b = 1.0
            outer_color.a = 0.8

            outer_marker = self._create_line_marker(
                marker_id=1,
                namespace='outer_boundary',
                points=self.outer_points,
                color=outer_color
            )
            marker_array.markers.append(outer_marker)

        # Publish
        if marker_array.markers:
            self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryVizNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
