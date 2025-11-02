#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String 

class AckermannRelay(Node):
    def __init__(self):
        super().__init__('ackermann_relay_node')
        
        self.max_speed = 0.00
        
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.listener_callback,
            10)
        
        self.steer_pub = self.create_publisher(
            String, 
            '/viz/steer',
            10)
            
        self.speed_pub = self.create_publisher(
            String, 
            '/viz/speed',
            10)

    def listener_callback(self, msg: AckermannDriveStamped):
        steer_msg = String()
        speed_msg = String()
        
        current_speed = msg.drive.speed
        if current_speed > self.max_speed:
            self.max_speed = current_speed
        
        steer_msg.data = f"Steering: {msg.drive.steering_angle:.2f} rad"
        speed_msg.data = f"Max Speed: {self.max_speed:.2f} m/s"
        
        self.steer_pub.publish(steer_msg)
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Final Max Speed: {node.max_speed:.2f} m/s")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
