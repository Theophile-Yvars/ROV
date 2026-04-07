cmake_minimum_required(VERSION 3.8)
project(rov_hardware)

# 1. Trouver les dépendances (Python ET C++)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)      # Ajouté pour le C++
find_package(std_msgs REQUIRED)    # Ajouté pour le message de température

# 2. Inclusion du dossier des headers (.hpp)
include_directories(include)

# 3. Configuration du nœud C++ (TMP102)
add_executable(temp_node src/temp_node.cpp)
ament_target_dependencies(temp_node rclcpp std_msgs)

# 4. Installation de TOUS les exécutables (Python et C++)
install(TARGETS
  temp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/camera_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Fin du fichier ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()