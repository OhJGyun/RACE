# f1tenth_stack/vesc_imu_bridge.py
import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescImuStamped
from sensor_msgs.msg import Imu

class VescImuBridge(Node):
    def __init__(self):
        super().__init__('vesc_imu_bridge')
        # frame_id 비어오면 대체용
        self.declare_parameter('frame_id', 'base_link')
        # 공분산 기본값(표준편차) 파라미터: 필요시 런치에서 조정
        self.declare_parameter('gyro_std',  0.02)   # rad/s
        self.declare_parameter('accel_std', 0.10)   # m/s^2

        # ===================== [ 수정 시작 ] =====================
        # carto_launch.py에서 설정한 부호(sign) 파라미터를 선언합니다.
        self.declare_parameter('roll_sign', 1.0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('accel_x_sign', 1.0)
        self.declare_parameter('accel_y_sign', 1.0)
        self.declare_parameter('accel_z_sign', 1.0)
        # ===================== [ 수정 끝 ] =====================

        self.pub = self.create_publisher(Imu, '/imu/vesc', 10)
        self.sub = self.create_subscription(VescImuStamped, '/sensors/imu', self.cb, 10)

    def cb(self, msg: VescImuStamped):
        out = Imu()

        # header
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or \
            self.get_parameter('frame_id').get_parameter_value().string_value

        imu = msg.imu

        # (1) orientation은 사용 안 함으로 표시
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = 0.0
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0    # ← Cartographer가 orientation 무시

        # ===================== [ 수정 시작 ] =====================
        # (2) 각속/가속 전달 시, 파라미터로 받은 부호를 곱해줍니다.
        # 파라미터 값 읽기
        roll_sign = self.get_parameter('roll_sign').get_parameter_value().double_value
        pitch_sign = self.get_parameter('pitch_sign').get_parameter_value().double_value
        yaw_sign = self.get_parameter('yaw_sign').get_parameter_value().double_value
        
        # 각속도(Angular Velocity)에 부호 적용
        out.angular_velocity.x = imu.angular_velocity.x * roll_sign
        out.angular_velocity.y = imu.angular_velocity.y * pitch_sign
        out.angular_velocity.z = imu.angular_velocity.z * yaw_sign

        # 선형 가속도(Linear Acceleration)에 부호 적용 (필요시 launch 파일에서 파라미터 추가)
        # 여기서는 기본값 1.0을 사용하거나, 각속도와 동일한 부호를 적용할 수 있습니다.
        out.linear_acceleration.x = imu.linear_acceleration.x * self.get_parameter('accel_x_sign').value
        out.linear_acceleration.y = imu.linear_acceleration.y * self.get_parameter('accel_y_sign').value
        out.linear_acceleration.z = imu.linear_acceleration.z * self.get_parameter('accel_z_sign').value
        # ===================== [ 수정 끝 ] =====================

        # (3) 각속/가속 공분산(대각) 넣기  ❗ -1 금지
        gstd = float(self.get_parameter('gyro_std').value)
        astd = float(self.get_parameter('accel_std').value)
        gv = gstd * gstd
        av = astd * astd
        out.angular_velocity_covariance = [0.0]*9
        out.linear_acceleration_covariance = [0.0]*9
        out.angular_velocity_covariance[0] = gv
        out.angular_velocity_covariance[4] = gv
        out.angular_velocity_covariance[8] = gv
        out.linear_acceleration_covariance[0] = av
        out.linear_acceleration_covariance[4] = av
        out.linear_acceleration_covariance[8] = av

        self.pub.publish(out)

# main 함수는 그대로 유지
def main():
    rclpy.init()
    node = VescImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
