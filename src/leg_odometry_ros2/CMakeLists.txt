cmake_minimum_required(VERSION 3.5)
project(leg_odometry_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(leg_odometry_node
  src/leg_odometry_node.cpp
  src/leg_kinematics.cpp
)
ament_target_dependencies(leg_odometry_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs unitree_go)
target_link_libraries(leg_odometry_node Eigen3::Eigen)

install(TARGETS
  leg_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
