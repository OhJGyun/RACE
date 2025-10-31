#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class DriveRelay(Node):
    def __init__(self):
        super().__init__('drive_relay')
        
        # /drive 토픽 구독
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',  # controller_manager.py가 발행하는 토픽
            self.drive_callback,
            10)
        
        self.drive_drive_sub = self.create_subscription(
            
        )
        
        # RViz 오버레이가 사용할 토픽 발행
        self.speed_pub = self.create_publisher(Float32, '/monitor/current_speed', 10)
        self.steer_pub = self.create_publisher(Float32, '/monitor/steering_input', 10)
        
        self.get_logger().info('Drive Relay 노드가 시작되었습니다. /drive 토픽을 변환합니다.')

    def drive_callback(self, msg: AckermannDriveStamped):
        # 속도 값 추출 및 발행
        speed_msg = Float32()
        speed_msg.data = msg.drive.speed
        self.speed_pub.publish(speed_msg)
        
        # 조향각 값 추출 및 발행
        steer_msg = Float32()
        steer_msg.data = msg.drive.steering_angle
        self.steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    drive_relay = DriveRelay()
    rclpy.spin(drive_relay)
    drive_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()