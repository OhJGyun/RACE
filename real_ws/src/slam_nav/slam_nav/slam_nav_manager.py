#!/usr/bin/env python3

"""
F1TENTH SLAM/Navigation Manager Node

This node provides services to dynamically switch between SLAM and localization modes,
save maps, and manage the navigation stack state.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import subprocess
import os
import signal
import time


class SlamNavManager(Node):
    def __init__(self):
        super().__init__('slam_nav_manager')
        
        self.current_mode = "slam"  # Current mode: "slam" or "localization"
        self.slam_process = None
        self.localization_process = None
        
        # Service servers
        self.switch_mode_srv = self.create_service(
            SetBool, 'switch_slam_mode', self.switch_mode_callback
        )
        
        self.save_map_srv = self.create_service(
            Empty, 'save_current_map', self.save_map_callback
        )
        
        self.get_mode_srv = self.create_service(
            SetBool, 'get_current_mode', self.get_mode_callback  
        )
        
        # Publishers/Subscribers for pose initialization
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        
        self.get_logger().info("SLAM/Nav Manager initialized")
        self.get_logger().info(f"Current mode: {self.current_mode}")

    def switch_mode_callback(self, request, response):
        """
        Switch between SLAM and localization modes
        request.data: True for localization, False for SLAM
        """
        target_mode = "localization" if request.data else "slam"
        
        if target_mode == self.current_mode:
            response.success = True
            response.message = f"Already in {target_mode} mode"
            return response
            
        try:
            self.get_logger().info(f"Switching from {self.current_mode} to {target_mode} mode")
            
            # Stop current processes
            self._stop_current_processes()
            
            # Start new mode
            if target_mode == "slam":
                success = self._start_slam_mode()
            else:
                success = self._start_localization_mode()
                
            if success:
                self.current_mode = target_mode
                response.success = True
                response.message = f"Successfully switched to {target_mode} mode"
                self.get_logger().info(f"Mode switched to: {target_mode}")
            else:
                response.success = False
                response.message = f"Failed to switch to {target_mode} mode"
                self.get_logger().error(f"Failed to switch to {target_mode} mode")
                
        except Exception as e:
            response.success = False
            response.message = f"Error switching modes: {str(e)}"
            self.get_logger().error(f"Error switching modes: {str(e)}")
            
        return response

    def save_map_callback(self, request, response):
        """Save current map when in SLAM mode"""
        if self.current_mode != "slam":
            self.get_logger().warn("Map saving only available in SLAM mode")
            return response
            
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            map_name = f"f1tenth_map_{timestamp}"
            map_dir = "/home/f1/f1tenth_ws/maps"
            
            # Create maps directory if it doesn't exist
            os.makedirs(map_dir, exist_ok=True)
            
            # Save map using map_saver
            cmd = [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", os.path.join(map_dir, map_name)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                self.get_logger().info(f"Map saved successfully: {map_name}")
            else:
                self.get_logger().error(f"Failed to save map: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"Error saving map: {str(e)}")
            
        return response

    def get_mode_callback(self, request, response):
        """Get current operating mode"""
        response.success = True
        response.message = self.current_mode
        return response

    def _stop_current_processes(self):
        """Stop current SLAM or localization processes"""
        try:
            # Use killall to stop slam_toolbox and cartographer nodes
            subprocess.run(["killall", "-9", "sync_slam_toolbox_node"], 
                          capture_output=True, timeout=5)
            subprocess.run(["killall", "-9", "async_slam_toolbox_node"], 
                          capture_output=True, timeout=5)
            subprocess.run(["killall", "-9", "cartographer_node"], 
                          capture_output=True, timeout=5)
            subprocess.run(["killall", "-9", "amcl"], 
                          capture_output=True, timeout=5)
            subprocess.run(["killall", "-9", "map_server"], 
                          capture_output=True, timeout=5)
            subprocess.run(["killall", "-9", "lifecycle_manager"], 
                          capture_output=True, timeout=5)
            
            time.sleep(2)  # Give processes time to terminate
            self.get_logger().info("Stopped current processes")
            
        except Exception as e:
            self.get_logger().warn(f"Error stopping processes: {str(e)}")

    def _start_slam_mode(self):
        """Start SLAM mode"""
        try:
            cmd = [
                "ros2", "launch", "slam_nav", "slam_launch.py",
                "slam_backend:=slam_toolbox", "mode:=sync"
            ]
            
            self.slam_process = subprocess.Popen(cmd)
            time.sleep(3)  # Give process time to start
            
            return self.slam_process.poll() is None
            
        except Exception as e:
            self.get_logger().error(f"Error starting SLAM: {str(e)}")
            return False

    def _start_localization_mode(self):
        """Start localization mode"""
        try:
            cmd = [
                "ros2", "launch", "slam_nav", "localization_launch.py"
            ]
            
            self.localization_process = subprocess.Popen(cmd)
            time.sleep(3)  # Give process time to start
            
            return self.localization_process.poll() is None
            
        except Exception as e:
            self.get_logger().error(f"Error starting localization: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    slam_nav_manager = SlamNavManager()
    
    executor = MultiThreadedExecutor()
    executor.add_node(slam_nav_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        slam_nav_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()