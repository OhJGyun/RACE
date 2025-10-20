cmake_minimum_required(VERSION 3.8)
project(slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files  
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Include directories
include_directories(include)

# Add C++ executables
add_executable(slam_nav_manager src/slam_nav_manager.cpp)
ament_target_dependencies(slam_nav_manager
  rclcpp
  std_srvs
  geometry_msgs
  nav_msgs
  lifecycle_msgs
)

add_executable(high_frequency_localizer_node src/high_frequency_localizer.cpp)
ament_target_dependencies(high_frequency_localizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# Install C++ executables
install(TARGETS
  slam_nav_manager
  high_frequency_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/localization_performance_test.py
  scripts/high_frequency_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()