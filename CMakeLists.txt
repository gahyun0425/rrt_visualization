cmake_minimum_required(VERSION 3.5)
project(rrt_visualization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable target name 수정
add_executable(rrt_node src/rrt_node.cpp)

# Target name을 rrt_visualization으로 변경
ament_target_dependencies(rrt_node rclcpp visualization_msgs geometry_msgs)

install(TARGETS
  rrt_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

