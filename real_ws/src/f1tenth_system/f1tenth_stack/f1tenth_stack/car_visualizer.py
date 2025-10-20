#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CarVisualizer(Node):
    def __init__(self):
        super().__init__('car_visualizer')
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/car_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber to odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Car visualizer started. Waiting for odom data...')

    def odom_callback(self, msg):
        # Publish car marker
        self.publish_car_marker(msg)
        
        # Publish base_link transform
        self.publish_base_link_tf(msg)

    def publish_car_marker(self, odom_msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "car"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position (center of car)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        
        # Orientation (same as base_link)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale (0.5m length, 0.3m width, 0.1m height)
        marker.scale.x = 0.5
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        
        # Color (light blue)
        marker.color.r = 0.68
        marker.color.g = 0.85
        marker.color.b = 0.90
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)

    def publish_base_link_tf(self, odom_msg):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Copy position and orientation from odometry
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    car_visualizer = CarVisualizer()
    
    try:
        rclpy.spin(car_visualizer)
    except KeyboardInterrupt:
        pass
    
    car_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()