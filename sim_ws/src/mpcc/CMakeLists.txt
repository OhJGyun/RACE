cmake_minimum_required(VERSION 3.8)
project(mpcc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python modules
ament_python_install_package(mpcc)

# Install Python executable scripts
install(PROGRAMS
  scripts/mpcc_node.py
  scripts/mpcc.py
  DESTINATION lib/${PROJECT_NAME}
)

# Note: Waypoint data is now stored in /RACE/path/ and shared between sim_ws and real_ws
# No need to install waypoint directories into the package

# Install params
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
