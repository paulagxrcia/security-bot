cmake_minimum_required(VERSION 3.10.2)
project(rosbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf rviz launch hooks config DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/rosbot_description.sh.in")
ament_package()
