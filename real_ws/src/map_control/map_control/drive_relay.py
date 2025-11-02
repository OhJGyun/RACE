#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class AckermannRelay(Node):
    def __init__(self):
        super().__init__('ackermann_relay_node')
        
        # /ackermann_cmd 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',  # 구독할 토픽
            self.listener_callback,
            10)
        
        # RViz가 볼 수 있도록 단순한 Float64 타입으로 발행할 퍼블리셔
        self.steer_pub = self.create_publisher(
            Float64, 
            '/viz/steering_angle',  # RViz용 스티어링 토픽
            10)
            
        self.speed_pub = self.create_publisher(
            Float64, 
            '/viz/speed',           # RViz용 속도 토픽
            10)

    def listener_callback(self, msg: AckermannDriveStamped):
        # Float64 메시지를 새로 만듭니다.
        steer_msg = String()
        speed_msg = String()
        
        # 값을 복사합니다.
        steer_msg.data = msg.drive.steering_angle
        speed_msg.data = msg.drive.speed
        
        # 새로 만든 토픽으로 발행합니다.
        self.steer_pub.publish(steer_msg)
        self.speed_pub.publish(speed_msg)

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
