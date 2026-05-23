cmake_minimum_required(VERSION 3.8)
project(rov_brain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

include_directories(include)

add_executable(brain_node src/brain_node.cpp)
ament_target_dependencies(brain_node rclcpp std_msgs sensor_msgs) 

install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()