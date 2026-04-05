cmake_minimum_required(VERSION 3.8)
project(rov_hardware)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Installer le script Python pour qu'il soit exécutable par ROS
install(PROGRAMS
  src/camera_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Optionnel : si tu veux organiser ça en module plus tard
# ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()