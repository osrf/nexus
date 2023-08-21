cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_zenoh_bridge_dds_vendor)
project(${PROJECT_NAME} VERSION 0.6.0)

include(ExternalProject)

find_package(ament_cmake REQUIRED)

ExternalProject_Add(zenoh_bridge_dds
  URL https://github.com/koonpeng/zenoh-plugin-dds/releases/download/0.6.0-beta.1/zenoh_bridge_dds-0.6.0-beta.1.tar.gz
  PREFIX "zenoh_bridge_dds-bin"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/zenoh_bridge_dds-bin/src/zenoh_bridge_dds/lib/zenoh_bridge_dds"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  USE_SOURCE_PERMISSIONS
)

ament_package()
