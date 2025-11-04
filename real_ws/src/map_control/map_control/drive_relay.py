#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Float32, Int32
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
            '/viz/steering_angle',
            10)
            
        self.speed_pub = self.create_publisher(
            String, 
            '/viz/speed',
            10)
        
        self.lap_time_sub = self.create_subscription(
            Float32,
            '/lap_time',
            self.lap_time_callback,
            10)
        
        self.lap_time_text_pub = self.create_publisher(
            String,
            '/viz/lap_time',
            10)

        self.object_num_sub = self.create_subscription(
            Int32,
            '/total_obs',
            self.obs_num_callback,
            10)

        self.mode_sub = self.create_subscription(
            Int32,
            '/drive_mode',
            self.mode_callback,
            10)

        self.lane_sub = self.create_subscription(
            Int32,
            '/lane_selector/selected_lane',
            self.lane_num_callback,
            10)

        self.object_num_pub = self.create_publisher(
            String,
            '/viz/obs_num',
            10)

        self.mode_pub = self.create_publisher(
            String,
            '/viz/mode',
            10)

        self.lane_num_pub = self.create_publisher(
            String,
            '/viz/lane_num',
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

    def lap_time_callback(self, msg: Float32):
        lap_time_msg = String()
        lap_time_msg.data = f"Lap Time: {msg.data:.2f} s"
        self.lap_time_text_pub.publish(lap_time_msg)

    def obs_num_callback(self, msg:Int32):
        obs_num_msg = String()
        obs_num_msg.data = f"Obs: {msg.data}"
        self.object_num_pub.publish(obs_num_msg)

    def mode_callback(self, msg:Int32):
        mode_msg = String()

        if msg.data == 0:
            mode_text = "Normal"
        elif msg.data == 1:
            mode_text = "Avoid"
        elif msg.data == 2:
            mode_text = "SCC"
        else:
            mode_text = f"Unknown ({msg.data})"

        mode_msg.data = f"Mode: {mode_text}"
        self.mode_pub.publish(mode_msg)

    def lane_num_callback(self, msg:Int32):
        lane_num_msg = String()
        lane_num_msg.data = f"Lane: {msg.data}"
        self.lane_num_pub.publish(lane_num_msg)
        


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
