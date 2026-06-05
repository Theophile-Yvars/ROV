cmake_minimum_required(VERSION 3.8)
project(rov_hardware)

# 1. Trouver les dépendances (Python ET C++)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Inclusion du dossier des headers (.hpp)
include_directories(include)

# 3. Configuration des nœuds C++

# --- Nœud Température interne (TMP102) ---
add_executable(temp_node src/temp_node.cpp)
ament_target_dependencies(temp_node rclcpp std_msgs)

# --- Nœud Pression / Profondeur externe (MS5837) ---
add_executable(pression_node src/pression_node.cpp)
ament_target_dependencies(pression_node rclcpp std_msgs sensor_msgs)

# --- Nœud Température CPU ---
add_executable(cpu_temp_node src/cpu_temp_node.cpp)
ament_target_dependencies(cpu_temp_node rclcpp std_msgs)

# --- Nœud Centrale Inertielle (IMU BNO055) ---
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp geometry_msgs)

# --- NOUVEAU : Nœud de contrôle des moteurs (4 Moteurs via Sysfs PWM) ---
add_executable(thruster_controller_node src/thruster_controller_node.cpp)
ament_target_dependencies(thruster_controller_node rclcpp geometry_msgs)
# Note : Plus besoin de target_link_libraries car Sysfs utilise l'écriture de fichiers standard C++


# 4. Installation de TOUS les exécutables (C++)
install(TARGETS
  temp_node
  pression_node 
  cpu_temp_node 
  imu_node
  thruster_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Installation des scripts (Python)
install(PROGRAMS
  src/camera_bridge_node.py
  src/battery_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Fin du fichier ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()