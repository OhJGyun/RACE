#include "slam_nav/high_frequency_localizer.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace f1tenth_slam_nav
{

HighFrequencyLocalizer::HighFrequencyLocalizer(const rclcpp::NodeOptions & options)
: Node("high_frequency_localizer_node", options),
  amcl_received_(false),
  odom_received_(false),
  drift_x_(0.0),
  drift_y_(0.0),
  drift_yaw_(0.0),
  max_timestamps_(1000)
{
  // 파라미터 선언
  this->declare_parameter("high_freq_rate", 100.0);
  this->declare_parameter("use_imu", true);

  // 파라미터 가져오기
  high_freq_rate_ = this->get_parameter("high_freq_rate").as_double();
  use_imu_ = this->get_parameter("use_imu").as_bool();

  // 구독자 초기화
  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&HighFrequencyLocalizer::amclPoseCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&HighFrequencyLocalizer::odomCallback, this, std::placeholders::_1));

  if (use_imu_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&HighFrequencyLocalizer::imuCallback, this, std::placeholders::_1));
  }

  // 발행자 초기화
  fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fused_pose", 10);
  high_freq_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/high_freq_odom", 10);

  // 타이머 초기화
  auto high_freq_period = std::chrono::duration<double>(1.0 / high_freq_rate_);
  high_freq_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(high_freq_period),
    std::bind(&HighFrequencyLocalizer::publishHighFreqPose, this));

  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&HighFrequencyLocalizer::printPerformanceStats, this));

  RCLCPP_INFO(this->get_logger(),
    "고주파 로컬라이제이션 노드 시작 (%.1f Hz)", high_freq_rate_);
}

void HighFrequencyLocalizer::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 현재 오도메트리와 AMCL 포즈 차이 계산 (드리프트)
  if (odom_received_) {
    double amcl_x = msg->pose.pose.position.x;
    double amcl_y = msg->pose.pose.position.y;
    double amcl_yaw = quaternionToYaw(msg->pose.pose.orientation);

    double odom_x = last_odom_pose_.position.x;
    double odom_y = last_odom_pose_.position.y;
    double odom_yaw = quaternionToYaw(last_odom_pose_.orientation);

    // 드리프트 계산
    drift_x_ = amcl_x - odom_x;
    drift_y_ = amcl_y - odom_y;
    drift_yaw_ = normalizeAngle(amcl_yaw - odom_yaw);

    RCLCPP_INFO(this->get_logger(),
      "드리프트 보정: dx=%.3f, dy=%.3f, dyaw=%.1f°",
      drift_x_, drift_y_, drift_yaw_ * 180.0 / M_PI);
  }

  last_amcl_pose_ = msg->pose.pose;
  last_amcl_time_ = this->get_clock()->now();
  amcl_received_ = true;
}

void HighFrequencyLocalizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_pose_ = msg->pose.pose;
  last_odom_time_ = this->get_clock()->now();
  odom_received_ = true;
}

void HighFrequencyLocalizer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  last_imu_data_ = *msg;
}

void HighFrequencyLocalizer::publishHighFreqPose()
{
  if (!odom_received_) {
    return;
  }

  auto current_time = this->get_clock()->now();

  // 드리프트 보정된 오도메트리 계산
  double corrected_x = last_odom_pose_.position.x + drift_x_;
  double corrected_y = last_odom_pose_.position.y + drift_y_;

  double odom_yaw = quaternionToYaw(last_odom_pose_.orientation);
  double corrected_yaw = normalizeAngle(odom_yaw + drift_yaw_);

  // 고주파 포즈 메시지 생성
  auto fused_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
  fused_pose->header.stamp = current_time;
  fused_pose->header.frame_id = "map";

  fused_pose->pose.position.x = corrected_x;
  fused_pose->pose.position.y = corrected_y;
  fused_pose->pose.position.z = 0.0;

  fused_pose->pose.orientation = yawToQuaternion(corrected_yaw);

  // 발행
  fused_pose_pub_->publish(std::move(fused_pose));

  // 고주파 오도메트리 생성 및 발행
  auto high_freq_odom = std::make_unique<nav_msgs::msg::Odometry>();
  high_freq_odom->header.stamp = current_time;
  high_freq_odom->header.frame_id = "map";
  high_freq_odom->child_frame_id = "base_link";

  high_freq_odom->pose.pose.position.x = corrected_x;
  high_freq_odom->pose.pose.position.y = corrected_y;
  high_freq_odom->pose.pose.position.z = 0.0;
  high_freq_odom->pose.pose.orientation = yawToQuaternion(corrected_yaw);

  // 속도 정보는 원본 오도메트리에서 복사 (추후 개선 가능)
  // high_freq_odom->twist = last_odom_twist_;

  high_freq_odom_pub_->publish(std::move(high_freq_odom));

  // 성능 모니터링
  pose_timestamps_.push_back(current_time);
  if (pose_timestamps_.size() > max_timestamps_) {
    pose_timestamps_.pop_front();
  }
}

void HighFrequencyLocalizer::printPerformanceStats()
{
  if (pose_timestamps_.size() < 10) {
    return;
  }

  // 최근 1초간의 주파수 계산
  auto current_time = this->get_clock()->now();
  auto one_second_ago = current_time - rclcpp::Duration(1, 0); // 1초

  size_t recent_count = 0;
  for (const auto & timestamp : pose_timestamps_) {
    if (timestamp >= one_second_ago) {
      recent_count++;
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "고주파 로컬라이제이션 주파수: %zu Hz (목표: %.0f Hz)",
    recent_count, high_freq_rate_);

  // 드리프트 상태 출력
  if (amcl_received_) {
    RCLCPP_INFO(this->get_logger(),
      "현재 드리프트: dx=%.3f, dy=%.3f, dyaw=%.1f°",
      drift_x_, drift_y_, drift_yaw_ * 180.0 / M_PI);
  }
}

double HighFrequencyLocalizer::quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);

  tf2::Matrix3x3 matrix(tf_quaternion);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  return yaw;
}

geometry_msgs::msg::Quaternion HighFrequencyLocalizer::yawToQuaternion(double yaw)
{
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion quaternion;
  tf2::convert(tf_quaternion, quaternion);

  return quaternion;
}

double HighFrequencyLocalizer::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

} // namespace f1tenth_slam_nav

// main 함수
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<f1tenth_slam_nav::HighFrequencyLocalizer>();

  RCLCPP_INFO(node->get_logger(), "고주파 로컬라이제이션 C++ 노드 시작");

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "노드 실행 중 오류: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}