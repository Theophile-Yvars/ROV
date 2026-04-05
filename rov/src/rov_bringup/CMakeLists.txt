cmake_minimum_required(VERSION 3.8)
project(rov_bringup)

find_package(ament_cmake REQUIRED)

# On installe le dossier 'launch' pour qu'il soit accessible après le build
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()