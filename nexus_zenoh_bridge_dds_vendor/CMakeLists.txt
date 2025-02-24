cmake_minimum_required(VERSION 3.16)
project(nexus_zenoh_bridge_dds_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

set(ZENOH_VERSION "1.0.3")
set(BINARY_URL "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip")
set(BINARY_DIR "${CMAKE_BINARY_DIR}/download")
file(DOWNLOAD ${BINARY_URL} ${BINARY_DIR}/zenoh-plugin-ros2dds.zip)

find_program(UNZIP_EXECUTABLE unzip)
if(NOT UNZIP_EXECUTABLE)
  message(FATAL_ERROR "unzip command not found. Please install it.")
endif()

execute_process(
  COMMAND ${UNZIP_EXECUTABLE} ${BINARY_DIR}/zenoh-plugin-ros2dds.zip
  WORKING_DIRECTORY ${BINARY_DIR}
)

install(
  FILES ${BINARY_DIR}/zenoh-bridge-ros2dds
  DESTINATION "lib/${PROJECT_NAME}"
  PERMISSIONS OWNER_EXECUTE
)

ament_package()
