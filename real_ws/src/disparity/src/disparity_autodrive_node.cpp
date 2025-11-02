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
        // Declare parameters
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("drive_topic", "/drive");
        this->declare_parameter("wheel_base", 0.32);
        this->declare_parameter("car_half_width", 0.15);
        this->declare_parameter("max_range", 15.0);
        this->declare_parameter("disparity_threshold", 1.0);
        this->declare_parameter("margin", 0.3);
        this->declare_parameter("max_speed", 6.0);
        this->declare_parameter("min_speed", 4.7);
        this->declare_parameter("max_steering_angle", 0.4);
        this->declare_parameter("ld_min", 0.5);
        this->declare_parameter("ld_max", 3.0);
        this->declare_parameter("m_ld", 0.4);
        this->declare_parameter("q_ld", 0.3);
        this->declare_parameter("steering_alpha", 0.7);
        this->declare_parameter("marker_frame_id", "ego_racecar/laser");
        this->declare_parameter("publish_markers", true);

        // Get parameters
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        car_half_width_ = this->get_parameter("car_half_width").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
        margin_ = this->get_parameter("margin").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        ld_min_ = this->get_parameter("ld_min").as_double();
        ld_max_ = this->get_parameter("ld_max").as_double();
        m_ld_ = this->get_parameter("m_ld").as_double();
        q_ld_ = this->get_parameter("q_ld").as_double();
        steering_alpha_ = this->get_parameter("steering_alpha").as_double();
        marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();
        publish_markers_ = this->get_parameter("publish_markers").as_bool();

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vehicle_path", 10);

        // Subscriptions
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&DisparityAutodrive::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&DisparityAutodrive::odom_callback, this, _1));

        // Debug publishers
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
        goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
        goal_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_line", 10);

        prev_steering_angle_ = 0.0;
        current_speed_ = 0.0;

        // Path initialization
        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Disparity Autodrive with Pure Pursuit + Path visualization started!");
        RCLCPP_INFO(this->get_logger(), "  Scan topic: %s", scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Odom topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Drive topic: %s", drive_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Max speed: %.1f m/s", max_speed_);
        RCLCPP_INFO(this->get_logger(), "  Min speed: %.1f m/s", min_speed_);
        RCLCPP_INFO(this->get_logger(), "  Lookahead: Ld = %.2f + %.2f * speed, clipped to [%.2f, %.2f]",
                    q_ld_, m_ld_, ld_min_, ld_max_);
    }

private:
    // ================= ODOM callback (Path generation) =================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;

        path_msg_.poses.push_back(pose_stamped);

        // Publish path
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
    }

    // ================= LIDAR callback =================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int n = msg->ranges.size();
        if (n == 0) return;

        // ---------- Lidar preprocessing ----------
        std::vector<float> ranges = msg->ranges;
        for (int i = 0; i < n; i++) {
            float angle = msg->angle_min + i * msg->angle_increment;

            // Use only front 180 degrees
            if (angle < -M_PI / 2.0 || angle > M_PI / 2.0) {
                ranges[i] = 0.0;
                continue;
            }

            if (!std::isfinite(ranges[i])) ranges[i] = 0.0;
            else if (ranges[i] > max_range_) ranges[i] = max_range_;
        }

        // Publish filtered scan
        sensor_msgs::msg::LaserScan filtered_scan = *msg;
        filtered_scan.ranges = ranges;
        filtered_scan_pub_->publish(filtered_scan);

        // ---------- Gap detection ----------
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

        is_curve = (max_disparity > disparity_threshold_);

        // ---------- Goal index ----------
        if (is_curve && boundary_idx != -1) {
            // Curve (use Disparity)
            float distance = std::min(ranges[boundary_idx], ranges[std::max(0, boundary_idx - 1)]);
            if (distance < 0.1) distance = 0.1;

            float offset_angle = atan((car_half_width_ + margin_) / distance);
            float boundary_angle = msg->angle_min + boundary_idx * msg->angle_increment;

            float goal_angle;
            if (disparity_diff < 0) {
                goal_angle = boundary_angle - offset_angle;
            } else {
                goal_angle = boundary_angle + offset_angle;
            }

            goal_idx = std::clamp(int((goal_angle - msg->angle_min) / msg->angle_increment), 0, n - 1);

        } else {
            // Straight (follow center of farthest points)
            std::vector<int> max_indices;
            for (int i = 0; i < n; i++) {
                if (ranges[i] >= max_range_) {
                    max_indices.push_back(i);
                }
            }
            if (!max_indices.empty()) {
                goal_idx = (max_indices.front() + max_indices.back()) / 2;
            }
        }

        if (goal_idx == -1) return;

        // ---------- Goal coordinates ----------
        float goal_angle = msg->angle_min + goal_idx * msg->angle_increment;
        float goal_dist = ranges[goal_idx];
        if (goal_dist <= 0.0) goal_dist = 1.0;

        float goal_x = goal_dist * cos(goal_angle);
        float goal_y = goal_dist * sin(goal_angle);

        // ---------- Pure Pursuit Lookahead ----------
        // Calculate lookahead distance using linear speed-based formula (like map_controller)
        // Ld = q_ld + m_ld * speed, then clip to [ld_min, ld_max]
        double Ld = q_ld_ + m_ld_ * current_speed_;
        Ld = std::clamp(Ld, ld_min_, ld_max_);

        float steering_angle = std::atan2(2.0 * wheel_base_ * goal_y, (Ld * Ld));
        steering_angle = std::clamp(steering_angle, -static_cast<float>(max_steering_angle_), static_cast<float>(max_steering_angle_));

        // Smoothing
        steering_angle = steering_alpha_ * prev_steering_angle_ + (1.0 - steering_alpha_) * steering_angle;
        prev_steering_angle_ = steering_angle;

        // ---------- Speed control ----------
        float angle_error = fabs(steering_angle);
        float speed = max_speed_ - (angle_error / max_steering_angle_) * (max_speed_ - min_speed_);
        speed = std::clamp(speed, static_cast<float>(min_speed_), static_cast<float>(max_speed_));

        // ---------- Ackermann control command ----------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);

        // ---------- RViz visualization ----------
        if (publish_markers_) {
            // Goal point marker
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = marker_frame_id_;
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

            // Line from vehicle to goal point
            visualization_msgs::msg::Marker goal_line;
            goal_line.header.frame_id = marker_frame_id_;
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
        }

        RCLCPP_INFO(this->get_logger(), "[%s] steer=%.2f deg speed=%.2f Ld=%.2f",
                    is_curve ? "CURVE" : "STRAIGHT", steering_angle * 180.0 / M_PI, speed, Ld);
    }

    // ================= Member variables =================
    // ROS communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_line_pub_;

    // Parameters
    double wheel_base_;
    double car_half_width_;
    double max_range_;
    double disparity_threshold_;
    double margin_;
    double max_speed_;
    double min_speed_;
    double max_steering_angle_;
    double ld_min_;
    double ld_max_;
    double m_ld_;
    double q_ld_;
    double steering_alpha_;
    std::string marker_frame_id_;
    bool publish_markers_;

    // State
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
