cmake_minimum_required(VERSION 3.8)

project(omniwheel_complete)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models")
  install(DIRECTORY gazebo_models DESTINATION share/${PROJECT_NAME})
endif()
ament_package()
