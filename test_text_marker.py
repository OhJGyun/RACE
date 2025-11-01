#!/usr/bin/env python3
"""
Simple test script to publish TEXT_VIEW_FACING markers to RViz2
This will help diagnose if text markers are visible in your RViz configuration
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class TestTextMarker(Node):
    def __init__(self):
        super().__init__('test_text_marker')

        # Publisher for test text markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/test/text_markers',
            10
        )

        # Timer to publish at 1Hz
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info("Test text marker node started")
        self.get_logger().info("Publishing to /test/text_markers")
        self.get_logger().info("In RViz2: Add -> MarkerArray -> Topic: /test/text_markers")

    def publish_markers(self):
        marker_array = MarkerArray()

        # Create a simple text marker at origin
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position at origin with some height
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        # Text properties
        marker.scale.z = 0.5  # Text height in meters
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = "TEST TEXT - Can you see me?"

        marker_array.markers.append(marker)

        # Add a second marker
        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "test_text"
        marker2.id = 1
        marker2.type = Marker.TEXT_VIEW_FACING
        marker2.action = Marker.ADD
        marker2.pose.position.x = 1.0
        marker2.pose.position.y = 1.0
        marker2.pose.position.z = 0.5
        marker2.pose.orientation.w = 1.0
        marker2.scale.z = 0.3
        marker2.color.r = 0.0
        marker2.color.g = 1.0
        marker2.color.b = 0.0
        marker2.color.a = 1.0
        marker2.text = "Second test marker"

        marker_array.markers.append(marker2)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published test markers", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TestTextMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
