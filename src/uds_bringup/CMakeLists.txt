cmake_minimum_required(VERSION 3.16)
project(uds_bringup)

find_package(ament_cmake REQUIRED)

# install launch files
install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
