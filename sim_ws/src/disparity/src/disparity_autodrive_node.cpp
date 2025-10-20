#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

class DisparityAutodrive : public rclcpp::Node {
public:
    DisparityAutodrive() : Node("disparity_autodrive") {
        // 퍼블리셔
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vehicle_path", 10);

        // 서브스크립션
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DisparityAutodrive::scan_callback, this, _1));


        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&DisparityAutodrive::odom_callback, this, _1));


        // 디버그용 퍼블리셔
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
        goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
        goal_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_line", 10);

        prev_steering_angle_ = 0.0;
        current_speed_ = 0.0;

        // Path 초기화
        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Disparity Autodrive with Pure Pursuit + Path visualization started!");
    }

private:
    // ================= ODOM 콜백 (Path 생성) =================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;

        path_msg_.poses.push_back(pose_stamped);

        // Path 퍼블리시
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
    }

    // ================= LIDAR 콜백 =================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int n = msg->ranges.size();
        if (n == 0) return;

        // 파라미터
        const float max_range = 15.0;
        const float disparity_threshold = 1.0;
        const float car_half_width = 0.15;
        const float margin = 0.3;
        const float wheel_base = 0.32;

        // ---------- Lidar 전처리 ----------
        std::vector<float> ranges = msg->ranges;
        for (int i = 0; i < n; i++) {
            float angle = msg->angle_min + i * msg->angle_increment;

            // 전방 180도만 사용
            if (angle < -M_PI / 2.0 || angle > M_PI / 2.0) {
                ranges[i] = 0.0;
                continue;
            }

            if (!std::isfinite(ranges[i])) ranges[i] = 0.0;
            else if (ranges[i] > max_range) ranges[i] = max_range;
        }

        // 필터링된 스캔 퍼블리시
        sensor_msgs::msg::LaserScan filtered_scan = *msg;
        filtered_scan.ranges = ranges;
        filtered_scan_pub_->publish(filtered_scan);

        // ---------- Gap 탐지 ----------
        int goal_idx = -1;
        bool is_curve = false;

        float max_disparity = -1.0;
        int boundary_idx = -1;
        float disparity_diff = 0.0;

        for (int i = 0; i < n - 1; i++) {
            if (ranges[i] <= 0.0 || ranges[i + 1] <= 0.0) continue;

            float diff = ranges[i + 1] - ranges[i];
            if (fabs(diff) > max_disparity) {
                max_disparity = fabs(diff);
                boundary_idx = (ranges[i + 1] < ranges[i]) ? i + 1 : i;
                disparity_diff = diff;
            }
        }

        is_curve = (max_disparity > disparity_threshold);

        // ---------- 목표 인덱스 ----------
        if (is_curve && boundary_idx != -1) {
            // 곡선 (Disparity 사용)
            float distance = std::min(ranges[boundary_idx], ranges[std::max(0, boundary_idx - 1)]);
            if (distance < 0.1) distance = 0.1;

            float offset_angle = atan((car_half_width + margin) / distance);
            float boundary_angle = msg->angle_min + boundary_idx * msg->angle_increment;

            float goal_angle;
            if (disparity_diff < 0) {
                goal_angle = boundary_angle - offset_angle;
            } else {
                goal_angle = boundary_angle + offset_angle;
            }

            goal_idx = std::clamp(int((goal_angle - msg->angle_min) / msg->angle_increment), 0, n - 1);

        } else {
            // 직선 (가장 먼 포인트 중앙 추종)
            std::vector<int> max_indices;
            for (int i = 0; i < n; i++) {
                if (ranges[i] >= max_range) {
                    max_indices.push_back(i);
                }
            }
            if (!max_indices.empty()) {
                goal_idx = (max_indices.front() + max_indices.back()) / 2;
            }
        }

        if (goal_idx == -1) return;

        // ---------- 목표 좌표 ----------
        float goal_angle = msg->angle_min + goal_idx * msg->angle_increment;
        float goal_dist = ranges[goal_idx];
        if (goal_dist <= 0.0) goal_dist = 1.0;

        float goal_x = goal_dist * cos(goal_angle);
        float goal_y = goal_dist * sin(goal_angle);

        // ---------- Pure Pursuit Lookahead ----------
        double Ld = std::sqrt(goal_x * goal_x + goal_y * goal_y);
        Ld = std::clamp(Ld + 0.3 * current_speed_, 0.5, 3.0);   // 모두 double

        float steering_angle = std::atan2(2.0 * wheel_base * goal_y, (Ld * Ld));
        steering_angle = std::clamp(steering_angle, -0.4f, 0.4f);

        // 스무딩
        float alpha = 0.7;
        steering_angle = alpha * prev_steering_angle_ + (1 - alpha) * steering_angle;
        prev_steering_angle_ = steering_angle;

        // ---------- 속도 제어 ----------
        float angle_error = fabs(steering_angle);
        float max_speed = 6.0;
        float min_speed = 4.7;
        float speed = max_speed - (angle_error / 0.4) * (max_speed - min_speed);
        speed = std::clamp(speed, min_speed, max_speed);

        // ---------- Ackermann 제어 명령 ----------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);

        // ---------- RViz 시각화 ----------
        // 목표점 마커
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = msg->header.frame_id;
        goal_marker.header.stamp = this->now();
        goal_marker.ns = "goal_point";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position.x = goal_x;
        goal_marker.pose.position.y = goal_y;
        goal_marker.pose.position.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.4;
        goal_marker.scale.y = 0.4;
        goal_marker.scale.z = 0.4;
        goal_marker.color.a = 1.0;
        goal_marker.color.r = 0.0;
        goal_marker.color.g = 1.0;
        goal_marker.color.b = 0.0;
        goal_marker_pub_->publish(goal_marker);

        // 차량에서 목표점까지 선
        visualization_msgs::msg::Marker goal_line;
        goal_line.header.frame_id = msg->header.frame_id;
        goal_line.header.stamp = this->now();
        goal_line.ns = "goal_line";
        goal_line.id = 1;
        goal_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        goal_line.action = visualization_msgs::msg::Marker::ADD;
        goal_line.scale.x = 0.05;
        goal_line.color.a = 1.0;

        if (is_curve) {
            goal_line.color.r = 1.0;
            goal_line.color.g = 0.0;
            goal_line.color.b = 0.0;
        } else {
            goal_line.color.r = 0.0;
            goal_line.color.g = 0.0;
            goal_line.color.b = 1.0;
        }

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = 0.0; p_start.y = 0.0; p_start.z = 0.0;
        p_end.x = goal_x; p_end.y = goal_y; p_end.z = 0.0;
        goal_line.points.push_back(p_start);
        goal_line.points.push_back(p_end);
        goal_line_pub_->publish(goal_line);

        RCLCPP_INFO(this->get_logger(), "[%s] steer=%.2f deg speed=%.2f Ld=%.2f",
                    is_curve ? "CURVE" : "STRAIGHT", steering_angle * 180.0 / M_PI, speed, Ld);
    }

    // ================= 멤버 변수 =================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_line_pub_;

    nav_msgs::msg::Path path_msg_;
    float prev_steering_angle_;
    float current_speed_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityAutodrive>());
    rclcpp::shutdown();
    return 0;
}
