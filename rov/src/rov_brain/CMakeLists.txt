cmake_minimum_required(VERSION 3.8)
project(robot_brain)

# Standard C++ 17 requis pour ROS 2 et OpenCV moderne
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Trouver les packages nécessaires
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Définir l'exécutable
# On ne liste que brain_node.cpp puisqu'il contient maintenant le main()
add_executable(brain_node src/brain_node.cpp)

# 3. Lier les dépendances
# On ajoute sensor_msgs, cv_bridge et OpenCV
target_link_libraries(brain_node ${OpenCV_LIBRARIES})
ament_target_dependencies(brain_node 
  rclcpp 
  std_msgs 
)

# 4. Installation
install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()