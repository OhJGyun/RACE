#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

class SlamNavManager : public rclcpp::Node
{
public:
    SlamNavManager() : Node("slam_nav_manager"), current_mode_("slam"), slam_process_pid_(-1), localization_process_pid_(-1)
    {
        // Service servers
        switch_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "switch_slam_mode",
            std::bind(&SlamNavManager::switchModeCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
            "save_current_map",
            std::bind(&SlamNavManager::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        get_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "get_current_mode",
            std::bind(&SlamNavManager::getModeCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Publisher for pose initialization
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "SLAM/Nav Manager initialized");
        RCLCPP_INFO(this->get_logger(), "Current mode: %s", current_mode_.c_str());
    }

    ~SlamNavManager()
    {
        stopCurrentProcesses();
    }

private:
    std::string current_mode_;
    pid_t slam_process_pid_;
    pid_t localization_process_pid_;
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_mode_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_mode_srv_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    void switchModeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::string target_mode = request->data ? "localization" : "slam";
        
        if (target_mode == current_mode_) {
            response->success = true;
            response->message = "Already in " + target_mode + " mode";
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Switching from %s to %s mode", 
                       current_mode_.c_str(), target_mode.c_str());
            
            // Stop current processes
            stopCurrentProcesses();
            
            // Start new mode
            bool success = false;
            if (target_mode == "slam") {
                success = startSlamMode();
            } else {
                success = startLocalizationMode();
            }
            
            if (success) {
                current_mode_ = target_mode;
                response->success = true;
                response->message = "Successfully switched to " + target_mode + " mode";
                RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", target_mode.c_str());
            } else {
                response->success = false;
                response->message = "Failed to switch to " + target_mode + " mode";
                RCLCPP_ERROR(this->get_logger(), "Failed to switch to %s mode", target_mode.c_str());
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Error switching modes: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Error switching modes: %s", e.what());
        }
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        if (current_mode_ != "slam") {
            RCLCPP_WARN(this->get_logger(), "Map saving only available in SLAM mode");
            return;
        }
        
        try {
            // Get current timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto tm = *std::localtime(&time_t);
            
            char timestamp[20];
            std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);
            
            std::string map_name = "f1tenth_map_" + std::string(timestamp);
            std::string map_dir = "/home/f1/f1tenth_ws/maps";
            
            // Create maps directory if it doesn't exist
            std::string mkdir_cmd = "mkdir -p " + map_dir;
            std::system(mkdir_cmd.c_str());
            
            // Save map using map_saver
            std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f " + 
                                 map_dir + "/" + map_name + " > /dev/null 2>&1 &";
            
            int result = std::system(save_cmd.c_str());
            
            if (result == 0) {
                RCLCPP_INFO(this->get_logger(), "Map saved successfully: %s", map_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save map");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving map: %s", e.what());
        }
    }

    void getModeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        response->success = true;
        response->message = current_mode_;
    }

    void stopCurrentProcesses()
    {
        try {
            // Kill SLAM and localization related processes
            std::vector<std::string> processes_to_kill = {
                "sync_slam_toolbox_node",
                "async_slam_toolbox_node", 
                "cartographer_node",
                "amcl",
                "map_server",
                "lifecycle_manager"
            };
            
            for (const auto& process : processes_to_kill) {
                std::string kill_cmd = "pkill -f " + process + " > /dev/null 2>&1";
                std::system(kill_cmd.c_str());
            }
            
            // Give processes time to terminate
            std::this_thread::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(this->get_logger(), "Stopped current processes");
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error stopping processes: %s", e.what());
        }
    }

    bool startSlamMode()
    {
        try {
            std::string launch_cmd = "ros2 launch slam_nav slam_launch.py "
                                   "slam_backend:=slam_toolbox mode:=sync > /dev/null 2>&1 &";
            
            slam_process_pid_ = fork();
            if (slam_process_pid_ == 0) {
                // Child process
                execl("/bin/sh", "sh", "-c", launch_cmd.c_str(), (char*)NULL);
                exit(1);
            } else if (slam_process_pid_ > 0) {
                // Parent process
                std::this_thread::sleep_for(std::chrono::seconds(3));
                
                // Check if process is still running
                int status;
                pid_t result = waitpid(slam_process_pid_, &status, WNOHANG);
                return (result == 0); // Process still running
            }
            
            return false;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error starting SLAM: %s", e.what());
            return false;
        }
    }

    bool startLocalizationMode()
    {
        try {
            std::string launch_cmd = "ros2 launch slam_nav localization_launch.py > /dev/null 2>&1 &";
            
            localization_process_pid_ = fork();
            if (localization_process_pid_ == 0) {
                // Child process
                execl("/bin/sh", "sh", "-c", launch_cmd.c_str(), (char*)NULL);
                exit(1);
            } else if (localization_process_pid_ > 0) {
                // Parent process
                std::this_thread::sleep_for(std::chrono::seconds(3));
                
                // Check if process is still running
                int status;
                pid_t result = waitpid(localization_process_pid_, &status, WNOHANG);
                return (result == 0); // Process still running
            }
            
            return false;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error starting localization: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto slam_nav_manager = std::make_shared<SlamNavManager>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(slam_nav_manager);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(slam_nav_manager->get_logger(), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}