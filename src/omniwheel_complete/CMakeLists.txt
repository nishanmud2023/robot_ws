cmake_minimum_required(VERSION 3.8)

project(omniwheel_complete)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
    rviz
    gazebo_models
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()