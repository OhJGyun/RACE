#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PAL Robotics SL. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors:
#   * Siegfried-A. Gevatter
#   * Jeremie Deray (artivis)

# New Keyboard Teleop with ackermann_msgs.msg, pynput
# Fixed_by Jisang_Yun
# Modified: Added S key for Twist/Ackermann toggle

import curses
import os
import signal
import time
import threading
from collections import defaultdict

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    print("pynput not available. Install with: pip install pynput")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class TextWindow():
    """Curses-based text window for clean GUI display"""
    
    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)  # Hide cursor
        self._num_lines = lines
        
        # Initialize colors if available
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
            curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message, color_pair=0):
        """Write a line of text to the screen"""
        if lineno < 0 or lineno >= self._num_lines:
            return
        
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 2
        
        for i, text in enumerate(message.split('\n')):
            if y + i < height - 1:  # Ensure we don't write past screen bounds
                text = text.ljust(min(len(text), width - x - 1))
                try:
                    if color_pair > 0:
                        self._screen.addstr(y + i, x, text, curses.color_pair(color_pair))
                    else:
                        self._screen.addstr(y + i, x, text)
                except curses.error:
                    pass  # Ignore errors from writing to screen edges

    def draw_title(self, title="F1TENTH Keyboard Teleop"):
        """Draw title without border"""
        height, width = self._screen.getmaxyx()
        title_x = max(2, (width - len(title)) // 2)
        self._screen.addstr(0, title_x, title, curses.color_pair(4))

    def refresh(self):
        self._screen.refresh()


class PynputCursesKeyTeleop(Node):
    def __init__(self, interface):
        super().__init__('pynput_curses_key_teleop')
        
        if not PYNPUT_AVAILABLE:
            self.get_logger().error("pynput library is required. Install with: pip install pynput")
            return
        
        self._interface = interface
        self._hz = 40.0
        self._running = True
        
        # Speed profiles (number key -> max speed)
        self._speed_profiles = {
            '1': 0.6, '2': 0.8, '3': 1.2, '4': 1.8, '5': 2.4,
            '6': 2.8, '7': 3.0, '8': 3.2, '9': 3.5, '0': 4.0
        }
        self._current_speed_profile = '2'  # Default to profile 2 (0.8 m/s)
        self._max_forward_rate = self._speed_profiles[self._current_speed_profile]
        self._max_backward_rate = self._max_forward_rate * 0.7  # Backward is 70% of forward
        self._max_rotation_rate = 1.0
        
        # Progressive acceleration settings
        self._acceleration_time = 0.5  # Time to reach max speed (seconds) - only for linear
        self._deceleration_time = 1.0  # Time to stop when key released (seconds) - only for linear
        self._steering_acceleration_time = 0.35  # Fast steering response (seconds)
        self._steering_deceleration_time = 0.3  # Quick steering return to center (seconds)
        self._initial_linear_speed = 0.4  # Initial speed when key is first pressed (m/s)
        
        # Message type toggle (True for Ackermann, False for Twist)
        self._use_ackermann = True
        
        # Thread-safe key state tracking with timestamps
        self._key_states = defaultdict(bool)
        self._key_press_times = defaultdict(float)
        self._state_lock = threading.Lock()
        
        # Key mappings to movement (linear, angular) - Only arrow keys
        self._key_mappings = {
            keyboard.Key.up: (1.0, 0.0),      # forward
            keyboard.Key.down: (-1.0, 0.0),   # backward
            keyboard.Key.left: (0.0, 0.45),    # left turn
            keyboard.Key.right: (0.0, -0.45),  # right turn
        }
        
        # Current velocity values
        self._linear = 0.0
        self._angular = 0.0
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._active_keys = []
        
        # ROS publishers
        self._pub_ackermann = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', qos_profile_system_default)
        self._pub_twist = self.create_publisher(
            Twist, '/cmd_vel', qos_profile_system_default)
        
        # Start keyboard listener
        self._start_keyboard_listener()

    def _start_keyboard_listener(self):
        """Start the keyboard listener in a separate thread"""
        def on_press(key):
            # Handle quit command
            if key == keyboard.KeyCode.from_char('q') or key == keyboard.KeyCode.from_char('Q'):
                self.get_logger().info("Quit key pressed. Shutting down...")
                self._running = False
                return False
            
            # Handle message type toggle
            if key == keyboard.KeyCode.from_char('s') or key == keyboard.KeyCode.from_char('S'):
                self._use_ackermann = not self._use_ackermann
                msg_type = "Ackermann" if self._use_ackermann else "Twist"
                self.get_logger().info(f"Switched to {msg_type} mode")
                return
            
            # Handle speed profile selection
            if hasattr(key, 'char') and key.char and key.char in self._speed_profiles:
                self._current_speed_profile = key.char
                self._max_forward_rate = self._speed_profiles[key.char]
                self._max_backward_rate = self._max_forward_rate * 0.7
                self.get_logger().info(f"Speed profile {key.char}: {self._max_forward_rate:.1f} m/s")
                return
            
            # Handle movement keys
            if key in self._key_mappings:
                with self._state_lock:
                    if not self._key_states[key]:  # Key just pressed
                        self._key_press_times[key] = time.time()
                    self._key_states[key] = True

        def on_release(key):
            if key in self._key_mappings:
                with self._state_lock:
                    self._key_states[key] = False

        # Start listener WITHOUT suppress - allows normal keyboard usage elsewhere
        self._listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release
        )
        self._listener.start()

    def run(self):
        """Main run loop"""
        while self._running:
            self._calculate_velocity()
            self._publish()
            time.sleep(1.0 / self._hz)

    def _calculate_velocity(self):
        """Calculate velocity based on currently pressed keys with progressive acceleration"""
        target_linear = 0.0
        target_angular = 0.0
        active_keys = []
        current_time = time.time()
        
        with self._state_lock:
            current_states = dict(self._key_states)
            current_press_times = dict(self._key_press_times)
        
        # Calculate target velocities based on pressed keys
        for key, is_pressed in current_states.items():
            if is_pressed and key in self._key_mappings:
                l, a = self._key_mappings[key]
                
                if l != 0:  # Linear movement (forward/backward) - progressive acceleration
                    # Calculate how long the key has been pressed
                    press_duration = current_time - current_press_times.get(key, current_time)
                    
                    # Progressive acceleration: 0 to 1 over acceleration_time
                    progress = min(press_duration / self._acceleration_time, 1.0)
                    
                    # Apply easing for smoother acceleration (quadratic ease-in)
                    progress = progress * progress
                    
                    # Scale by maximum rates - starts from initial speed
                    if l > 0:  # Forward
                        speed_range = self._max_forward_rate - self._initial_linear_speed
                        target_linear += l * (self._initial_linear_speed + progress * speed_range)
                    else:  # Backward
                        speed_range = self._max_backward_rate - self._initial_linear_speed
                        target_linear += l * (self._initial_linear_speed + progress * speed_range)
                    
                    # Add to active keys for display with progress
                    key_name = self._get_key_name(key)
                    active_keys.append(f"{key_name}({progress:.1%})")
                
                if a != 0:  # Angular movement (steering) - fast but progressive
                    # Calculate how long the steering key has been pressed
                    press_duration = current_time - current_press_times.get(key, current_time)
                    
                    # Fast progressive acceleration for steering: 0 to 1 over steering_acceleration_time
                    steering_progress = min(press_duration / self._steering_acceleration_time, 1.0)
                    
                    # Apply slight easing for smooth steering (less aggressive than linear)
                    steering_progress = steering_progress * (2 - steering_progress)  # Ease-out
                    
                    target_angular += a * steering_progress * self._max_rotation_rate
                    
                    # Add to active keys for display with progress
                    key_name = self._get_key_name(key)
                    if key_name not in [k.split('(')[0] for k in active_keys]:
                        active_keys.append(f"{key_name}({steering_progress:.1%})")
        
        # Store target values
        self._target_linear = target_linear
        self._target_angular = target_angular
        
        # Smooth transition to target velocity
        dt = 1.0 / self._hz
        
        # Linear velocity with progressive acceleration/deceleration
        if abs(target_linear) < 0.01:  # No linear input
            # Decelerate to zero
            decel_rate = self._max_forward_rate / self._deceleration_time
            if self._linear > 0:
                self._linear = max(0.0, self._linear - decel_rate * dt)
            elif self._linear < 0:
                self._linear = min(0.0, self._linear + decel_rate * dt)
        else:
            # Move towards target linear velocity
            accel_rate = max(self._max_forward_rate, self._max_backward_rate) / self._acceleration_time
            diff = target_linear - self._linear
            if abs(diff) > accel_rate * dt:
                self._linear += accel_rate * dt * (1 if diff > 0 else -1)
            else:
                self._linear = target_linear
        
        # Angular velocity with fast progressive response
        if abs(target_angular) < 0.01:  # No angular input
            # Quick deceleration to zero for steering
            decel_rate = self._max_rotation_rate / self._steering_deceleration_time
            if self._angular > 0:
                self._angular = max(0.0, self._angular - decel_rate * dt)
            elif self._angular < 0:
                self._angular = min(0.0, self._angular + decel_rate * dt)
        else:
            # Fast response for steering
            accel_rate = self._max_rotation_rate / self._steering_acceleration_time
            diff = target_angular - self._angular
            if abs(diff) > accel_rate * dt:
                self._angular += accel_rate * dt * (1 if diff > 0 else -1)
            else:
                self._angular = target_angular
        
        # Clamp values to reasonable ranges
        self._linear = max(-3.0, min(3.0, self._linear))
        self._angular = max(-1.5, min(1.5, self._angular))
        
        self._active_keys = active_keys

    def _get_key_name(self, key):
        """Convert key to readable string"""
        key_names = {
            keyboard.Key.up: '↑',
            keyboard.Key.down: '↓', 
            keyboard.Key.left: '←',
            keyboard.Key.right: '→'
        }
        
        if key in key_names:
            return key_names[key]
        elif hasattr(key, 'char') and key.char:
            return key.char.upper()
        else:
            return str(key)

    def _make_ackermann_msg(self, speed, steering_angle):
        """Create Ackermann message"""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        return msg

    def _make_twist_msg(self, linear_vel, angular_vel):
        """Create Twist message"""
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        return msg

    def _publish(self):
        """Publish control commands and update GUI"""
        # Clear and update interface
        self._interface.clear()
        self._interface.draw_title()
        
        # Display current status
        active_str = ' '.join(self._active_keys) if self._active_keys else 'None'
        
        # Message type display
        msg_type = "Ackermann" if self._use_ackermann else "Twist"
        msg_color = 1 if self._use_ackermann else 2
        self._interface.write_line(2, f"Mode: {msg_type}", msg_color)
        
        # Speed profile display
        profile_color = 1
        self._interface.write_line(3, f"Speed Profile: {self._current_speed_profile} ({self._max_forward_rate:.1f}m/s)", profile_color)
        
        # Speed display with color coding
        speed_color = 1 if self._linear > 0 else (3 if self._linear < 0 else 0)
        self._interface.write_line(4, f"Speed:    {self._linear:+6.2f} m/s", speed_color)
        
        # Steering display with color coding  
        steer_color = 2 if abs(self._angular) > 0 else 0
        self._interface.write_line(5, f"Steering: {self._angular:+6.2f} rad/s", steer_color)
        
        # Active keys display
        self._interface.write_line(6, f"Keys:     {active_str}")
        
        # Controls display
        self._interface.write_line(8, "Movement Controls:")
        self._interface.write_line(9, "↑: Forward      ↓: Backward")
        self._interface.write_line(10, "←: Left         →: Right")
        
        # Speed profile controls
        self._interface.write_line(11, "Speed Profiles (1-0):")
        self._interface.write_line(12, "1:0.6  2:0.8  3:1.2  4:1.8  5:2.4")
        self._interface.write_line(13, "6:2.8  7:3.0  8:3.2  9:3.5  0:4.0")
        
        # Other controls
        self._interface.write_line(14, "S: Toggle Mode  Q: Quit")
        
        # Progressive acceleration info
        self._interface.write_line(15, f"Linear: {self._acceleration_time:.1f}s | Steering: {self._steering_acceleration_time:.1f}s")
        
        # Additional info
        if self._active_keys:
            self._interface.write_line(17, "● DRIVING", 1)
        else:
            self._interface.write_line(17, "○ STOPPED", 2)
            
        self._interface.write_line(18, f"Rate: {self._hz} Hz | You can type in other apps!")
        
        # Topic info
        topic = "/ackermann_cmd" if self._use_ackermann else "/cmd_vel"
        self._interface.write_line(19, f"Publishing to: {topic}")
        
        self._interface.refresh()
        
        # Publish appropriate message type
        if self._use_ackermann:
            ackermann_msg = self._make_ackermann_msg(self._linear, self._angular)
            self._pub_ackermann.publish(ackermann_msg)
        else:
            twist_msg = self._make_twist_msg(self._linear, self._angular)
            self._pub_twist.publish(twist_msg)

    def stop(self):
        """Clean shutdown"""
        self._running = False
        if hasattr(self, '_listener'):
            self._listener.stop()


def execute(stdscr):
    """Main execution function for curses"""
    if not PYNPUT_AVAILABLE:
        stdscr.addstr(0, 0, "Error: pynput library is required")
        stdscr.addstr(1, 0, "Install with: pip install pynput")
        stdscr.addstr(2, 0, "Press any key to exit...")
        stdscr.refresh()
        stdscr.getch()
        return
    
    rclpy.init()
    
    try:
        app = PynputCursesKeyTeleop(TextWindow(stdscr, lines=20))
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'app' in locals():
            app.stop()
        rclpy.shutdown()


def main():
    """Main entry point"""
    if not PYNPUT_AVAILABLE:
        print("Error: pynput library is required")
        print("Install with: pip install pynput")
        return
    
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        print("\nShutdown completed.")


if __name__ == '__main__':
    main()
