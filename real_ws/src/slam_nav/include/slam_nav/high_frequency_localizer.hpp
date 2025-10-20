#ifndef F1TENTH_SLAM_NAV__HIGH_FREQUENCY_LOCALIZER_HPP_
#define F1TENTH_SLAM_NAV__HIGH_FREQUENCY_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <deque>
#include <chrono>

namespace f1tenth_slam_nav
{

/**
 * @class HighFrequencyLocalizer
 * @brief 고주파 로컬라이제이션 노드
 *
 * AMCL의 저주파 정확한 글로벌 위치와 오도메트리의 고주파 로컬 정보를
 * 융합하여 100Hz 이상의 정밀한 위치 추정을 제공합니다.
 */
class HighFrequencyLocalizer : public rclcpp::Node
{
public:
  /**
   * @brief 생성자
   */
  explicit HighFrequencyLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief AMCL 포즈 콜백 - 드리프트 보정
   */
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief 오도메트리 콜백
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief IMU 콜백
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief 고주파 포즈 발행 타이머 콜백
   */
  void publishHighFreqPose();

  /**
   * @brief 성능 통계 출력 타이머 콜백
   */
  void printPerformanceStats();

  /**
   * @brief 쿼터니언을 yaw 각도로 변환
   */
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion);

  /**
   * @brief yaw 각도를 쿼터니언으로 변환
   */
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  /**
   * @brief 각도 정규화 (-π ~ π)
   */
  double normalizeAngle(double angle);

  // 구독자
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // 발행자
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr high_freq_odom_pub_;

  // 타이머
  rclcpp::TimerBase::SharedPtr high_freq_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // 상태 변수
  geometry_msgs::msg::Pose last_amcl_pose_;
  geometry_msgs::msg::Pose last_odom_pose_;
  sensor_msgs::msg::Imu last_imu_data_;

  rclcpp::Time last_amcl_time_;
  rclcpp::Time last_odom_time_;

  bool amcl_received_;
  bool odom_received_;

  // 드리프트 보정 변수
  double drift_x_;
  double drift_y_;
  double drift_yaw_;

  // 성능 모니터링
  std::deque<rclcpp::Time> pose_timestamps_;
  size_t max_timestamps_;

  // 파라미터
  double high_freq_rate_;  // Hz
  bool use_imu_;
};

} // namespace f1tenth_slam_nav

#endif // F1TENTH_SLAM_NAV__HIGH_FREQUENCY_LOCALIZER_HPP_