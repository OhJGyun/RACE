#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

struct Waypoint {
    double x, y, target_speed;
    Waypoint(double x = 0.0, double y = 0.0, double speed = 1.0) : x(x), y(y), target_speed(speed) {}
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_integral = 10.0, double max_output = 100.0)
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), max_output_(max_output),
          previous_error_(0.0), integral_(0.0), previous_time_(0.0) {}

    double compute(double setpoint, double measured_value, double current_time) {
        double error = setpoint - measured_value;
        double dt = current_time - previous_time_;

        if (dt <= 0.0) {
            return 0.0;
        }

        double proportional = kp_ * error;

        integral_ += error * dt;
        integral_ = std::max(-max_integral_, std::min(integral_, max_integral_));
        double integral_term = ki_ * integral_;

        double derivative = kd_ * (error - previous_error_) / dt;

        double output = proportional + integral_term + derivative;
        output = std::max(-max_output_, std::min(output, max_output_));

        previous_error_ = error;
        previous_time_ = current_time;

        return output;
    }

    void reset() {
        previous_error_ = 0.0;
        integral_ = 0.0;
        previous_time_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double max_integral_, max_output_;
    double previous_error_, integral_, previous_time_;
};

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), 
                        tf_buffer_(this->get_clock()), 
                        tf_listener_(tf_buffer_)  // ✅ shared_from_this() 제거
    {
        // Parameters
        this->declare_parameter("csv_file_path", "/home/f1/f1tenth_ws/maps_racelines/raceline/fuck_jg_1_mintime_vmax3ms.csv");
        this->declare_parameter("lookahead_distance", 1.2);
        this->declare_parameter("max_speed", 8.0);
        this->declare_parameter("min_speed", 1.0);
        this->declare_parameter("max_steering_angle", 0.4189);
        this->declare_parameter("wheelbase", 0.33);

        // ✅ 새로운 파라미터: TF 사용 여부
        this->declare_parameter("use_tf_for_localization", true);
        this->declare_parameter("tf_timeout", 0.1);  // TF 조회 타임아웃 (초)
        this->declare_parameter("max_pose_age", 0.5);  // amcl_pose 최대 허용 나이 (초)

        // PID parameters
        this->declare_parameter("speed_kp", 1.0);
        this->declare_parameter("speed_ki", 0.1);
        this->declare_parameter("speed_kd", 0.05);
        this->declare_parameter("speed_max_integral", 2.0);
        this->declare_parameter("speed_max_output", 5.0);

        // Curvature control
        this->declare_parameter("enable_curvature_control", true);
        this->declare_parameter("max_curvature", 2.0);
        this->declare_parameter("min_curve_speed", 2.0);
        this->declare_parameter("curvature_speed_factor", 0.7);
        this->declare_parameter("curve_lookahead_points", 5);

        // Adaptive lookahead
        this->declare_parameter("enable_adaptive_lookahead", true);
        this->declare_parameter("min_lookahead_distance", 0.8);
        this->declare_parameter("max_lookahead_distance", 2.5);
        this->declare_parameter("lookahead_speed_factor", 0.3);
        this->declare_parameter("steering_smoothing_factor", 0.7);

        // Get parameters
        csv_file_path_ = this->get_parameter("csv_file_path").as_string();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // ✅ TF 관련 파라미터
        use_tf_for_localization_ = this->get_parameter("use_tf_for_localization").as_bool();
        tf_timeout_ = this->get_parameter("tf_timeout").as_double();
        max_pose_age_ = this->get_parameter("max_pose_age").as_double();

        // PID controller
        double speed_kp = this->get_parameter("speed_kp").as_double();
        double speed_ki = this->get_parameter("speed_ki").as_double();
        double speed_kd = this->get_parameter("speed_kd").as_double();
        double speed_max_integral = this->get_parameter("speed_max_integral").as_double();
        double speed_max_output = this->get_parameter("speed_max_output").as_double();
        speed_pid_ = std::make_unique<PIDController>(speed_kp, speed_ki, speed_kd, speed_max_integral, speed_max_output);

        // Curvature control
        enable_curvature_control_ = this->get_parameter("enable_curvature_control").as_bool();
        max_curvature_ = this->get_parameter("max_curvature").as_double();
        min_curve_speed_ = this->get_parameter("min_curve_speed").as_double();
        curvature_speed_factor_ = this->get_parameter("curvature_speed_factor").as_double();
        curve_lookahead_points_ = this->get_parameter("curve_lookahead_points").as_int();

        // Adaptive lookahead
        enable_adaptive_lookahead_ = this->get_parameter("enable_adaptive_lookahead").as_bool();
        min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
        max_lookahead_distance_ = this->get_parameter("max_lookahead_distance").as_double();
        lookahead_speed_factor_ = this->get_parameter("lookahead_speed_factor").as_double();
        steering_smoothing_factor_ = this->get_parameter("steering_smoothing_factor").as_double();

        previous_steering_angle_ = 0.0;

        // Load waypoints
        if (!loadWaypoints()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from CSV file: %s", csv_file_path_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from CSV file", waypoints_.size());

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pure_pursuit_path", 10);
        lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_point", 10);

        // Subscribers
        // ✅ amcl_pose는 백업용으로만 사용
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&PurePursuitNode::controlLoop, this));

        publishPath();

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node initialized successfully!");
        if (use_tf_for_localization_) {
            RCLCPP_INFO(this->get_logger(), "Using TF for real-time localization");
        } else {
            RCLCPP_INFO(this->get_logger(), "Using /amcl_pose topic for localization");
        }
    }

private:
    // ROS2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string csv_file_path_;
    double lookahead_distance_;
    double max_speed_;
    double min_speed_;
    double max_steering_angle_;
    double wheelbase_;

    // ✅ TF 관련 파라미터
    bool use_tf_for_localization_;
    double tf_timeout_;
    double max_pose_age_;

    // State
    std::vector<Waypoint> waypoints_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    bool pose_received_ = false;
    rclcpp::Time last_pose_time_;
    size_t closest_waypoint_index_ = 0;

    // Control
    std::unique_ptr<PIDController> speed_pid_;
    double current_speed_ = 0.0;

    // Curvature control
    bool enable_curvature_control_;
    double max_curvature_;
    double min_curve_speed_;
    double curvature_speed_factor_;
    int curve_lookahead_points_;

    // Adaptive lookahead
    bool enable_adaptive_lookahead_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double lookahead_speed_factor_;
    double steering_smoothing_factor_;
    double previous_steering_angle_;

    // ✅ Statistics
    size_t tf_success_count_ = 0;
    size_t tf_failure_count_ = 0;
    size_t amcl_fallback_count_ = 0;

    bool loadWaypoints() {
        waypoints_.clear();
        std::ifstream file(csv_file_path_);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", csv_file_path_.c_str());
            return false;
        }

        std::string line;
        bool first_line = true;

        while (std::getline(file, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            std::stringstream ss(line);
            std::string x_str, y_str, speed_str;

            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, speed_str, ',')) {
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    double speed = std::stod(speed_str);
                    waypoints_.emplace_back(x, y, speed);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error parsing line: %s", line.c_str());
                }
            } else if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    waypoints_.emplace_back(x, y, max_speed_);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error parsing line: %s", line.c_str());
                }
            }
        }

        file.close();
        return !waypoints_.empty();
    }

    // ✅ amcl_pose 콜백 (백업용)
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Frame ID 검증
        if (msg->header.frame_id != "map") {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Received pose in frame '%s', expected 'map'. Ignoring.",
                msg->header.frame_id.c_str()
            );
            return;
        }

        current_pose_ = *msg;
        pose_received_ = true;
        last_pose_time_ = msg->header.stamp;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_speed_ = sqrt(vx * vx + vy * vy);
    }

    // ✅ 핵심 함수: TF로 실시간 위치 획득
    bool getCurrentPoseFromTF(double& x, double& y, double& yaw) {
        try {
            // TF 조회: map → base_link
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map",              // target frame
                "base_link",        // source frame
                tf2::TimePointZero, // 최신 사용 가능한 변환
                tf2::durationFromSec(tf_timeout_)  // 타임아웃
            );

            // 위치 추출 ###
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;

            // Quaternion → Yaw 변환
            tf2::Quaternion quat(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            
            tf2::Matrix3x3 mat(quat);
            double roll, pitch;
            mat.getRPY(roll, pitch, yaw);

            tf_success_count_++;
            return true;

        } catch (tf2::TransformException& ex) {
            tf_failure_count_++;
            
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TF lookup failed: %s (Success: %zu, Fail: %zu)",
                ex.what(), tf_success_count_, tf_failure_count_
            );
            return false;
        }
    }

    // ✅ 백업: amcl_pose에서 위치 획득
    bool getCurrentPoseFromAMCL(double& x, double& y, double& yaw) {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "No amcl_pose received yet"
            );
            return false;
        }

        // 데이터 신선도 확인
        auto current_time = this->get_clock()->now();
        double pose_age = (current_time - last_pose_time_).seconds();

        if (pose_age > max_pose_age_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "amcl_pose data is %.2f seconds old (max: %.2f)",
                pose_age, max_pose_age_
            );
            return false;
        }

        // 위치 추출
        x = current_pose_.pose.pose.position.x;
        y = current_pose_.pose.pose.position.y;

        // Quaternion → Yaw
        tf2::Quaternion quat;
        tf2::fromMsg(current_pose_.pose.pose.orientation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, yaw);

        amcl_fallback_count_++;
        return true;
    }

    // ✅ 통합 함수: 위치 획득 (TF 우선, amcl_pose 백업)
    bool getCurrentPose(double& x, double& y, double& yaw) {
        if (use_tf_for_localization_) {
            // TF 우선 시도
            if (getCurrentPoseFromTF(x, y, yaw)) {
                return true;
            }

            // TF 실패 시 amcl_pose로 fallback
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "TF failed, falling back to amcl_pose"
            );

            if (getCurrentPoseFromAMCL(x, y, yaw)) {
                return true;
            }

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Both TF and amcl_pose failed!"
            );
            return false;

        } else {
            // amcl_pose만 사용
            return getCurrentPoseFromAMCL(x, y, yaw);
        }
    }

    void publishPath() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->get_clock()->now();

        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    double distance(const Waypoint& a, const Waypoint& b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    size_t findClosestWaypoint(double x, double y) {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            double dist = distance(Waypoint(x, y), waypoints_[i]);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    double getAdaptiveLookaheadDistance() {
        if (!enable_adaptive_lookahead_) {
            return lookahead_distance_;
        }

        double adaptive_distance = lookahead_distance_ + (current_speed_ * lookahead_speed_factor_);
        return std::max(min_lookahead_distance_, std::min(adaptive_distance, max_lookahead_distance_));
    }

    Waypoint findLookaheadPoint(double x, double y, double yaw) {
        size_t start_idx = findClosestWaypoint(x, y);
        closest_waypoint_index_ = start_idx;

        double current_lookahead = getAdaptiveLookaheadDistance();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            size_t idx = (start_idx + i) % waypoints_.size();
            double dist = distance(Waypoint(x, y), waypoints_[idx]);

            if (dist >= current_lookahead) {
                double dx = waypoints_[idx].x - x;
                double dy = waypoints_[idx].y - y;
                double angle_to_point = atan2(dy, dx);
                double angle_diff = angle_to_point - yaw;

                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                if (abs(angle_diff) < M_PI/2) {
                    return waypoints_[idx];
                }
            }
        }

        size_t ahead_idx = (start_idx + 10) % waypoints_.size();
        return waypoints_[ahead_idx];
    }

    double calculateSteeringAngle(double x, double y, double yaw, const Waypoint& lookahead_point) {
        double dx = lookahead_point.x - x;
        double dy = lookahead_point.y - y;

        double local_x = cos(yaw) * dx + sin(yaw) * dy;
        double local_y = -sin(yaw) * dx + cos(yaw) * dy;

        double lookahead_dist = sqrt(local_x * local_x + local_y * local_y);

        if (lookahead_dist < 0.1) {
            return previous_steering_angle_;
        }

        double curvature = 2.0 * local_y / (lookahead_dist * lookahead_dist);
        double raw_steering_angle = atan(wheelbase_ * curvature);

        double steering_angle = steering_smoothing_factor_ * previous_steering_angle_ +
                               (1.0 - steering_smoothing_factor_) * raw_steering_angle;

        steering_angle = std::max(-max_steering_angle_, std::min(steering_angle, max_steering_angle_));

        previous_steering_angle_ = steering_angle;

        return steering_angle;
    }

    double calculateCurvature(size_t waypoint_idx) {
        if (waypoints_.size() < 3 || waypoint_idx == 0 || waypoint_idx >= waypoints_.size() - 1) {
            return 0.0;
        }

        const auto& p1 = waypoints_[waypoint_idx - 1];
        const auto& p2 = waypoints_[waypoint_idx];
        const auto& p3 = waypoints_[waypoint_idx + 1];

        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;

        double angle1 = atan2(dy1, dx1);
        double angle2 = atan2(dy2, dx2);
        double angle_diff = angle2 - angle1;

        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        double dist = sqrt(dx1 * dx1 + dy1 * dy1) + sqrt(dx2 * dx2 + dy2 * dy2);

        if (dist < 0.01) {
            return 0.0;
        }

        return abs(angle_diff) / (dist * 0.5);
    }

    double getMaxCurvatureAhead(size_t start_idx) {
        double max_curvature = 0.0;

        for (int i = 0; i < curve_lookahead_points_ && (start_idx + i) < waypoints_.size(); ++i) {
            size_t idx = start_idx + i;
            double curvature = calculateCurvature(idx);
            max_curvature = std::max(max_curvature, curvature);
        }

        return max_curvature;
    }

    double getTargetSpeed() {
        if (waypoints_.empty()) {
            return min_speed_;
        }

        double target_speed = waypoints_[closest_waypoint_index_].target_speed;

        if (enable_curvature_control_) {
            double max_curve_ahead = getMaxCurvatureAhead(closest_waypoint_index_);

            if (max_curve_ahead > 0.1) {
                double curvature_ratio = std::min(max_curve_ahead / max_curvature_, 1.0);
                double speed_reduction = curvature_ratio * curvature_speed_factor_;

                target_speed = target_speed * (1.0 - speed_reduction);
                target_speed = std::max(target_speed, min_curve_speed_);

                if (max_curve_ahead > 0.5) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "High curvature detected: %.3f, reducing speed to: %.2f m/s",
                        max_curve_ahead, target_speed
                    );
                }
            }
        }

        return std::max(min_speed_, std::min(target_speed, max_speed_));
    }

    double calculateSpeedWithPID(double target_speed) {
        double current_time = this->get_clock()->now().seconds();
        double speed_adjustment = speed_pid_->compute(target_speed, current_speed_, current_time);
        double commanded_speed = target_speed + speed_adjustment;
        return std::max(min_speed_, std::min(commanded_speed, max_speed_));
    }

    // ✅ 메인 제어 루프
    void controlLoop() {
        if (waypoints_.empty()) {
            return;
        }

        // ✅ TF를 통해 실시간 위치 획득!
        double x, y, yaw;
        if (!getCurrentPose(x, y, yaw)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "No valid pose available, skipping control loop"
            );
            return;
        }

        // 나머지는 동일
        Waypoint lookahead_point = findLookaheadPoint(x, y, yaw);
        double steering_angle = calculateSteeringAngle(x, y, yaw, lookahead_point);
        double target_speed = getTargetSpeed();
        double speed = calculateSpeedWithPID(target_speed);

        // Publish drive command
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;

        drive_pub_->publish(drive_msg);

        // Publish lookahead point
        geometry_msgs::msg::PoseStamped lookahead_msg;
        lookahead_msg.header.frame_id = "map";
        lookahead_msg.header.stamp = this->get_clock()->now();
        lookahead_msg.pose.position.x = lookahead_point.x;
        lookahead_msg.pose.position.y = lookahead_point.y;
        lookahead_msg.pose.position.z = 0.0;
        lookahead_msg.pose.orientation.w = 1.0;

        lookahead_pub_->publish(lookahead_msg);

        // ✅ 향상된 로깅
        double current_curvature = 0.0;
        if (enable_curvature_control_) {
            current_curvature = getMaxCurvatureAhead(closest_waypoint_index_);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[%s] Pos: (%.2f, %.2f), Yaw: %.2f°, Steer: %.2f°, Speed: T=%.2f/C=%.2f/Cmd=%.2f, Curv: %.3f, WP: %zu | TF: %zu✓/%zu✗, AMCL: %zu",
            use_tf_for_localization_ ? "TF" : "AMCL",
            x, y, yaw * 180.0 / M_PI,
            steering_angle * 180.0 / M_PI,
            target_speed, current_speed_, speed,
            current_curvature, closest_waypoint_index_,
            tf_success_count_, tf_failure_count_, amcl_fallback_count_
        );
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
