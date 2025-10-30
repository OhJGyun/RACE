#!/usr/bin/env python3
"""
Steering Test Node - 조향각 응답성 테스트용 노드

다양한 파형의 조향각 명령을 생성하여 실제 drive 출력을 분석합니다.
- Step 입력: 급격한 변화 테스트
- Sine 파형: 주파수 응답 테스트
- Square 파형: 지연 시간 측정

Usage:
    ros2 run app steering_test_node --ros-args -p waveform:=step -p frequency:=0.5
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped


class SteeringTestNode(Node):
    def __init__(self):
        super().__init__('steering_test_node')

        # Parameters
        self.declare_parameter('waveform', 'step')  # 'step', 'sine', 'square', 'triangle'
        self.declare_parameter('frequency', 0.5)     # Hz
        self.declare_parameter('amplitude', 0.3)     # radians (±17 degrees)
        self.declare_parameter('speed', 2.0)         # m/s (constant speed)
        self.declare_parameter('publish_rate', 40.0) # Hz
        self.declare_parameter('test_duration', 30.0) # seconds (0 = infinite)

        self.waveform = self.get_parameter('waveform').value
        self.frequency = self.get_parameter('frequency').value
        self.amplitude = self.get_parameter('amplitude').value
        self.speed = self.get_parameter('speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.test_duration = self.get_parameter('test_duration').value

        # Publisher with BEST_EFFORT QoS to match ackermann_mux
        # Use /teleop topic (priority=100) instead of /drive (priority=10)
        # This ensures our test commands override any navigation commands
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/teleop',  # Same as joy_teleop, high priority
            qos
        )

        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State
        self.start_time = self.get_clock().now()
        self.last_step_time = self.start_time
        self.step_state = 1  # For step waveform

        self.get_logger().info('=' * 60)
        self.get_logger().info('Steering Test Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Waveform:      {self.waveform}')
        self.get_logger().info(f'  Frequency:     {self.frequency} Hz')
        self.get_logger().info(f'  Amplitude:     ±{self.amplitude} rad (±{math.degrees(self.amplitude):.1f}°)')
        self.get_logger().info(f'  Speed:         {self.speed} m/s')
        self.get_logger().info(f'  Publish Rate:  {self.publish_rate} Hz')
        self.get_logger().info(f'  Test Duration: {self.test_duration}s (0=infinite)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Recording this in PlotJuggler:')
        self.get_logger().info('  Input:  /teleop/drive/steering_angle (this node)')
        self.get_logger().info('  Output: /ackermann_cmd/drive/steering_angle (mux)')
        self.get_logger().info('  Servo:  /sensors/servo_position_command (VESC)')
        self.get_logger().info('')
        self.get_logger().info('NOTE: Make sure joy_teleop is NOT running!')
        self.get_logger().info('      Or this test will be ignored by mux.')
        self.get_logger().info('=' * 60)

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Check test duration
        if self.test_duration > 0 and elapsed > self.test_duration:
            self.get_logger().info(f'Test completed after {elapsed:.1f}s')
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
            return

        # Calculate steering angle based on waveform
        steering_angle = self.generate_waveform(elapsed)

        # Create message
        msg = AckermannDriveStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = steering_angle

        # Publish
        self.publisher.publish(msg)

        # Log every second
        if int(elapsed) != int(elapsed - (1.0 / self.publish_rate)):
            self.get_logger().info(
                f't={elapsed:.1f}s | steering={steering_angle:+.3f} rad ({math.degrees(steering_angle):+.1f}°) | speed={self.speed:.1f} m/s'
            )

    def generate_waveform(self, t):
        """Generate different waveforms for testing"""

        if self.waveform == 'step':
            # Step function: switches between -amplitude and +amplitude every period
            period = 1.0 / self.frequency if self.frequency > 0 else 2.0
            if (self.get_clock().now() - self.last_step_time).nanoseconds / 1e9 >= period:
                self.step_state *= -1
                self.last_step_time = self.get_clock().now()
            return self.amplitude * self.step_state

        elif self.waveform == 'sine':
            # Sine wave
            omega = 2 * math.pi * self.frequency
            return self.amplitude * math.sin(omega * t)

        elif self.waveform == 'square':
            # Square wave
            omega = 2 * math.pi * self.frequency
            return self.amplitude if math.sin(omega * t) >= 0 else -self.amplitude

        elif self.waveform == 'triangle':
            # Triangle wave
            period = 1.0 / self.frequency if self.frequency > 0 else 1.0
            phase = (t % period) / period  # 0 to 1
            if phase < 0.25:
                return self.amplitude * (4 * phase)
            elif phase < 0.75:
                return self.amplitude * (2 - 4 * phase)
            else:
                return self.amplitude * (4 * phase - 4)

        elif self.waveform == 'chirp':
            # Chirp: increasing frequency over time
            f_start = 0.1
            f_end = 2.0
            f_t = f_start + (f_end - f_start) * (t / self.test_duration)
            return self.amplitude * math.sin(2 * math.pi * f_t * t)

        else:
            self.get_logger().error(f'Unknown waveform: {self.waveform}')
            return 0.0


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SteeringTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
