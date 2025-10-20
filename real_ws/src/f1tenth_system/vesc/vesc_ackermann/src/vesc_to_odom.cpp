// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using sensor_msgs::msg::Imu;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  publish_tf_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  yaw_initialized_(false),
  use_imu_yaw_(true),
  imu_yaw_(0.0),
  last_imu_time_(0, 0, RCL_ROS_TIME)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);

  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain").get<double>();
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset").get<double>();

  if (use_servo_cmd_) {
    steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain").get<double>();
    steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset").get<double>();
    wheelbase_ = declare_parameter("wheelbase").get<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);
  use_imu_yaw_ = declare_parameter("use_imu_yaw", use_imu_yaw_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  // subscribe to IMU for initial yaw
  imu_sub_ = create_subscription<Imu>(
    "sensors/imu/raw", 10, std::bind(&VescToOdom::imuCallback, this, _1));
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // convert to engineering units
  double current_speed = -(-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  // propigate odometry
  // Use IMU yaw if available and enabled, otherwise use wheel odometry yaw
  double current_yaw = yaw_;
  if (use_imu_yaw_ && yaw_initialized_) {
    current_yaw = imu_yaw_;
    yaw_ = imu_yaw_;  // Sync yaw with IMU
  } else if (use_servo_cmd_) {
    // Update yaw from wheel odometry only if not using IMU
    yaw_ += current_angular_velocity * dt.seconds();
    current_yaw = yaw_;
  }

  double x_dot = current_speed * cos(current_yaw);
  double y_dot = current_speed * sin(current_yaw);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  // Convert quaternion to yaw
  tf2::Quaternion q(
    imu->orientation.x,
    imu->orientation.y,
    imu->orientation.z,
    imu->orientation.w
  );

  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, imu_yaw_);

  last_imu_time_ = imu->header.stamp;

  if (!yaw_initialized_) {
    yaw_ = imu_yaw_;  // Initialize wheel odometry yaw with IMU
    yaw_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial yaw set from IMU to: %f rad (%f deg)", yaw_, yaw_ * 180.0 / M_PI);
  }

  // Log IMU updates periodically for debugging
  static int imu_count = 0;
  if (++imu_count % 50 == 0) {
    RCLCPP_DEBUG(this->get_logger(), "IMU yaw: %f rad (%f deg), use_imu_yaw: %s",
                 imu_yaw_, imu_yaw_ * 180.0 / M_PI, use_imu_yaw_ ? "true" : "false");
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
