cmake_minimum_required(VERSION 3.8)
project(rov_hardware)

# 1. Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Inclusion du dossier des headers (.hpp)
include_directories(include)

# 3. Configuration des nœuds C++

# Nœuds standards
add_executable(temp_node src/temp_node.cpp)
ament_target_dependencies(temp_node rclcpp std_msgs)

add_executable(pression_node src/pression_node.cpp)
ament_target_dependencies(pression_node rclcpp std_msgs sensor_msgs)

add_executable(cpu_temp_node src/cpu_temp_node.cpp)
ament_target_dependencies(cpu_temp_node rclcpp std_msgs)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp geometry_msgs)

add_executable(thruster_controller_node src/thruster_controller_node.cpp)
ament_target_dependencies(thruster_controller_node rclcpp geometry_msgs)

# Nœud Servo Caméra (Utilisant lgpio)
add_executable(camera_servo_node src/camera_servo_node.cpp)
ament_target_dependencies(camera_servo_node rclcpp std_msgs)

# Lien avec la bibliothèque lgpio trouvée sur le système
find_library(LGPIO_LIB lgpio)
target_link_libraries(camera_servo_node ${LGPIO_LIB})

# 4. Installation
install(TARGETS
  temp_node
  pression_node 
  cpu_temp_node 
  imu_node
  thruster_controller_node
  camera_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/camera_bridge_node.py
  src/battery_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()