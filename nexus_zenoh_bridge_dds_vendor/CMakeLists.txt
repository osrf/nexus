cmake_minimum_required(VERSION 3.16)
project(nexus_zenoh_bridge_dds_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(zeno_bridge_dds_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
  VCS_VERSION 1.0.4
)

# TODO(sloretz) make a nice way to get this path from ament_vendor
set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/zeno_bridge_dds_vendor-prefix/install")
install(
  DIRECTORY "${INSTALL_DIR}/lib/zenoh_bridge_ros2dds/"
  DESTINATION "lib/${PROJECT_NAME}"
  USE_SOURCE_PERMISSIONS
)

ament_package()
