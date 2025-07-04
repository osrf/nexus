cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_common)
project(${PROJECT_NAME} VERSION 0.0.1)

include(GenerateExportHeader)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nexus_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

add_library(${PROJECT_NAME} SHARED
  src/bt_store.cpp
  src/logging.cpp
  src/node_thread.cpp
  src/node_utils.cpp
  src/pausable_sequence.cpp
  src/task_remapper.cpp
)
GENERATE_EXPORT_HEADER(${PROJECT_NAME}
  EXPORT_FILE_NAME "include/nexus_common_export.hpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/nexus_common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}>
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${vision_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  BT::behaviortree_cpp_v3
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  yaml-cpp
  ${lifecycle_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/nexus_common DESTINATION include/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/ DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(rmf_utils REQUIRED)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)

  add_library(${PROJECT_NAME}_test STATIC
    src/test_utils.cpp
  )
  target_compile_features(${PROJECT_NAME}_test PUBLIC cxx_std_17)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}_test>
  )
  target_link_libraries(${PROJECT_NAME}_test PUBLIC
    BT::behaviortree_cpp_v3
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
  install(TARGETS ${PROJECT_NAME}_test
    EXPORT ${PROJECT_NAME}_test
    INCLUDES DESTINATION include/${PROJECT_NAME}_test
  )
  install(DIRECTORY include/nexus_common_test DESTINATION include/${PROJECT_NAME}_test)
  ament_export_targets(${PROJECT_NAME}_test)

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
    target_include_directories(${target} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include/nexus_common_test
      ${example_interfaces_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
    )
    target_link_libraries(${target} PRIVATE
      ${PROJECT_NAME}
      ${PROJECT_NAME}_test
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      BT::behaviortree_cpp_v3
      rmf_utils::rmf_utils
      ${example_interfaces_TARGETS}
      ${std_msgs_TARGETS}
    )
  endfunction()

  nexus_add_test(action_client_bt_node_test src/action_client_bt_node_test.cpp)
  nexus_add_test(pausable_sequence_test src/pausable_sequence_test.cpp)
  nexus_add_test(service_client_bt_node_test src/service_client_bt_node_test.cpp)
  nexus_add_test(models_test
    src/main_test.cpp
    src/models/item_test.cpp
    src/models/work_order_test.cpp
  )
  nexus_add_test(ros_utils_test
    src/main_test.cpp
    src/batch_service_call_test.cpp
  )
  nexus_add_test(logging_test src/logging_test.cpp)
  nexus_add_test(sync_ros_client_test src/sync_ros_client_test.cpp)
  nexus_add_test(task_remapper_test src/task_remapper_test.cpp)
endif()

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(behaviortree_cpp_v3 geometry_msgs lifecycle_msgs rclcpp rclcpp_action rclcpp_lifecycle tf2 vision_msgs)
ament_package()
