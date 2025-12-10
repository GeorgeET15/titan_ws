cmake_minimum_required(VERSION 3.8)
project(titan_v1)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
