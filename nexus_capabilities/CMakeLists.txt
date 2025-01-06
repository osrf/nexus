cmake_minimum_required(VERSION 3.16)
project(nexus_capabilities)

include(GenerateExportHeader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(dep_pkgs
  behaviortree_cpp_v3
  geometry_msgs
  nexus_common
  nexus_endpoints
  nexus_orchestrator_msgs
  nlohmann_json
  rclcpp
  rclcpp_lifecycle
  rmf_dispenser_msgs
  rmf_task_msgs
  yaml_cpp_vendor
  yaml-cpp
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

set(plugin_dep_pkgs
  pluginlib
  tf2
  tf2_ros
)
foreach(pkg ${plugin_dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(rmf_utils 1.4.0 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_catch2 REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  # TODO(luca) Add uncrustify back after formatting
  #ament_uncrustify(
  #  ARGN include src
  #  CONFIG_FILE ${uncrustify_config_file}
  #  MAX_LINE_LENGTH 80
  #  LANGUAGE CPP
  #)
endif()

include(GNUInstallDirs)

#===============================================================================
# nexus_capabilities library

add_library(${PROJECT_NAME} SHARED
  src/conversions/pose_stamped.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${behaviortree_cpp_v3_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nexus_orchestrator_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    nexus_common::nexus_common
    nexus_endpoints::nexus_endpoints
    ${geometry_msgs_TARGETS}
    ${nexus_orchestrator_msgs_TARGETS}
    BT::behaviortree_cpp_v3
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp::yaml-cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

GENERATE_EXPORT_HEADER(${PROJECT_NAME}
  EXPORT_FILE_NAME "include/nexus_capabilities/export.hpp"
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dep_pkgs})

#===============================================================================
# installs
install(
  DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)

#===============================================================================
# builtin capabilities
#===============================================================================
add_library(nexus_builtin_capabilities SHARED
  src/capabilities/detection.cpp
  src/capabilities/detection_capability.cpp
  src/capabilities/dispense_item_capability.cpp
  src/capabilities/dispense_item.cpp
  src/capabilities/execute_trajectory_capability.cpp
  src/capabilities/execute_trajectory.cpp
  src/capabilities/gripper_capability.cpp
  src/capabilities/gripper_control.cpp
  src/capabilities/plan_motion_capability.cpp
  src/capabilities/plan_motion.cpp
  src/capabilities/rmf_request.cpp
  src/capabilities/rmf_request_capability.cpp
)

target_compile_options(nexus_builtin_capabilities PUBLIC INTERFACE cxx_std_17)

target_link_libraries(nexus_builtin_capabilities
  PUBLIC
    ${PROJECT_NAME}
    nexus_common::nexus_common
    nexus_endpoints::nexus_endpoints
    nlohmann_json::nlohmann_json
    pluginlib::pluginlib
    tf2::tf2
    tf2_ros::tf2_ros
    ${rmf_dispenser_msgs_TARGETS}
    ${rmf_task_msgs_TARGETS}
    yaml-cpp
)

install(
  TARGETS
    nexus_builtin_capabilities
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nexus_capabilities src/capabilities/plugins.xml)

#===============================================================================
# builtin task checkers
#===============================================================================
add_library(nexus_builtin_task_checkers SHARED
  src/task_checkers/filepath_checker.cpp
)

target_compile_options(nexus_builtin_task_checkers PUBLIC INTERFACE cxx_std_17)

target_link_libraries(nexus_builtin_task_checkers
  PUBLIC
    ${PROJECT_NAME}
    nexus_common::nexus_common
    nexus_endpoints::nexus_endpoints
    pluginlib::pluginlib
    rclcpp::rclcpp
)

install(
  TARGETS
  nexus_builtin_task_checkers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nexus_capabilities src/task_checkers/plugins.xml)

ament_package()
