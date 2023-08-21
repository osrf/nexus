cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_cmake)
project(${PROJECT_NAME} VERSION 0.0.1)

find_package(ament_cmake)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake-extras.cmake)
