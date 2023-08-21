cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_workcell_orchestrator)
project(${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nexus_cmake REQUIRED)
find_package(nexus_capabilities REQUIRED)
find_package(nexus_common REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(nexus_lifecycle_manager REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED VERSION 0.7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#===============================================================================
add_library(${PROJECT_NAME}_plugin SHARED
  src/get_result.cpp
  src/get_joint_constraints.cpp
  src/make_transform.cpp
  src/serialization.cpp
  src/set_result.cpp
  src/signals.cpp
  src/task_parser.cpp
  src/transform_pose.cpp
  src/workcell_orchestrator.cpp
)
target_include_directories(${PROJECT_NAME}_plugin
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${vision_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  nexus_capabilities::nexus_capabilities
  nexus_common::nexus_common
  nexus_endpoints::nexus_endpoints
  nexus_lifecycle_manager::nexus_lifecycle_manager_core
  BT::behaviortree_cpp_v3
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  yaml-cpp
  ${vision_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_plugin PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_plugin PUBLIC "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_node(${PROJECT_NAME}_plugin
  PLUGIN "nexus::workcell_orchestrator::WorkcellOrchestrator"
  EXECUTABLE nexus_workcell_orchestrator
)
install(TARGETS ${PROJECT_NAME}_plugin DESTINATION lib)

#===============================================================================
if(BUILD_TESTING)
  find_package(rmf_utils 1.4.0 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_catch2 REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
    LANGUAGE CPP
  )

  # Adds a ament catch2 test with some common libraries.
  # Usage: nexus_add_test(<target> <source>...)
  function(nexus_add_test target)
    ament_add_catch2(${target} ${ARGN})
    target_link_libraries(${target} PRIVATE
      ${PROJECT_NAME}_plugin
      nexus_common::nexus_common_test
    )
  endfunction()

  nexus_add_test(serialization_tests
    src/main_test.cpp
    src/serialization_test.cpp
  )
  nexus_add_test(task_results_tests
    src/main_test.cpp
    src/task_results_test.cpp
  )
  nexus_add_test(transform_pose_test src/transform_pose_test.cpp)
endif()

ament_package()
