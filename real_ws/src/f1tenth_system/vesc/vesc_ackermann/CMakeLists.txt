cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

# Setting C++ standard to 17 (ROS2 Humble recommendation)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# nodes library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom.cpp
)

# register nodes as components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::AckermannToVesc
  EXECUTABLE ackermann_to_vesc_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::VescToOdom
  EXECUTABLE vesc_to_odom_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
