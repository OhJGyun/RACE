#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <fstream>  // 로깅을 위해 추가
#include <iomanip>  // 시간 포맷을 위해 추가
#include <chrono>   // 시간 측정을 위해 추가

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 0.0);
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
  steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", 0.0);
  steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.0);

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // 로깅 추가
  std::ofstream log_file("/tmp/ackermann_to_vesc.log", std::ios::app);
  if (log_file.is_open()) {
    auto now_time = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now_time);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_time.time_since_epoch()) % 1000;
    
    log_file << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    log_file << "." << std::setfill('0') << std::setw(3) << ms.count();
    log_file << " ACKERMANN_CMD: speed=" << std::fixed << std::setprecision(3) 
             << cmd->drive.speed << " steering=" << cmd->drive.steering_angle;
    log_file << " | VESC_OUT: erpm=" << erpm_msg.data << " servo=" << servo_msg.data;
    log_file << std::endl;
    log_file.close();
  }

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)