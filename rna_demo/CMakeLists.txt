cmake_minimum_required(VERSION 3.5)

project(rna_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
