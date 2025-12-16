cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_system_orchestrator)
project(${PROJECT_NAME} VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nexus_cmake REQUIRED)
find_package(nexus_common REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(nexus_lifecycle_manager REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

add_library(${PROJECT_NAME}_plugin SHARED
  src/add_task_assignments_to_metadata.cpp
  src/bid_transporter.cpp
  src/create_transporter_task.cpp
  src/execute_task.cpp
  src/for_each_task.cpp
  src/send_signal.cpp
  src/system_orchestrator.cpp
  src/transporter_request.cpp
  src/workcell_request.cpp
  src/context.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  nexus_common::nexus_common
  nexus_endpoints::nexus_endpoints
  nexus_lifecycle_manager::nexus_lifecycle_manager_core
  BT::behaviortree_cpp_v3
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  yaml-cpp::yaml-cpp
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE "COMPOSITION_BUILDING_DLL")
target_compile_features(${PROJECT_NAME}_plugin PRIVATE cxx_std_17)
rclcpp_components_register_node(${PROJECT_NAME}_plugin
  PLUGIN "nexus::system_orchestrator::SystemOrchestrator"
  EXECUTABLE nexus_system_orchestrator
)
install(TARGETS ${PROJECT_NAME}_plugin DESTINATION lib)

if(BUILD_TESTING)
  find_package(rmf_utils REQUIRED)
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

  # Adds a ament catch2 test with some common libraries.
  # Usage: nexus_add_test(<target> <source>...)
  function(nexus_add_test target)
    ament_add_catch2(${target} ${ARGN})
    target_link_libraries(${target} PRIVATE
      ${PROJECT_NAME}_plugin
      nexus_common::nexus_common_test
    )
  endfunction()
endif()

ament_package()
