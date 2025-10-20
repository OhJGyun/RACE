cmake_minimum_required(VERSION 3.8)
project(disparity_autodrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ===== 실행 파일 추가 =====
add_executable(disparity_autodrive_node src/disparity_autodrive_node.cpp)

ament_target_dependencies(disparity_autodrive_node
  rclcpp
  sensor_msgs
  ackermann_msgs
  nav_msgs
  visualization_msgs
)

# 실행 파일 설치 (빌드 후 실행 가능하게)
install(TARGETS
  disparity_autodrive_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

