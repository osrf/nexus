cmake_minimum_required(VERSION 3.16)
project(nexus_zenoh_bridge_dds_vendor VERSION 0.6.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(zeno_bridge_dds_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-plugin-dds.git
  VCS_VERSION 0.7.2-rc
)

# TODO(sloretz) make a nice way to get this path from ament_vendor
set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/zeno_bridge_dds_vendor-prefix/install")
install(
  DIRECTORY "${INSTALL_DIR}/lib/zenoh_bridge_dds/"
  DESTINATION "lib/${PROJECT_NAME}"
  USE_SOURCE_PERMISSIONS
)

ament_package()
