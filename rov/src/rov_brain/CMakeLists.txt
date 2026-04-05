cmake_minimum_required(VERSION 3.8)
project(rov_brain)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Note: Si tu utilises OpenCV plus tard, n'oublie pas le find_package(OpenCV REQUIRED)

# 2. Définir l'exécutable
add_executable(brain_node src/brain_node.cpp)

# --- AJOUT ICI : Dossier d'inclusion ---
target_include_directories(brain_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Lier les dépendances
ament_target_dependencies(brain_node 
  rclcpp 
  std_msgs 
)

# 4. Installation
install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installer aussi le dossier include pour les autres packages
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()