cmake_minimum_required(VERSION 3.8)
project(nonlinear_mpc_casadi)

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
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/Nonlinear_MPC_node.py
  scripts/Nonlinear_MPC.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install waypoint directories
install(DIRECTORY
  scripts/porto/
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  scripts/f1tenth/
  DESTINATION lib/${PROJECT_NAME}/f1tenth
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install params
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
