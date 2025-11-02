#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

class AckermannRelay(Node):
    def __init__(self):
        super().__init__('ackermann_relay_node')
        
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd', 
            self.listener_callback,
            10)
        
        
        self.steer_pub = self.create_publisher(
            String, 
            '/viz/steering_angle', 
            10)

    def listener_callback(self, msg: AckermannDriveStamped):
        # String Message
        steer_msg = String()
                
        # 값을 복사합니다.
        steer_msg.data = f"Steering: {msg.drive.steering_angle:.2f} rad"
        
        
        # 새로 만든 토픽으로 발행합니다.
        self.steer_pub.publish(steer_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = AckermannRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
