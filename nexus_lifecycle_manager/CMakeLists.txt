cmake_minimum_required(VERSION 3.16)
project(nexus_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nexus_common REQUIRED)
find_package(nexus_lifecycle_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager_client.cpp
)

set(dependencies
  nexus_common
  nexus_lifecycle_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rmf_utils
  std_srvs
  diagnostic_updater
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}>
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  ${dependencies}
)

install(TARGETS
  ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  # TODO(luca) Add uncrustify back after formatting
  #ament_uncrustify(
  #  ARGN include src test
  #  CONFIG_FILE ${uncrustify_config_file}
  #  MAX_LINE_LENGTH 80
  #  LANGUAGE CPP
  #)

  find_package(ament_cmake_catch2 QUIET)
  if (ament_cmake_catch2_FOUND)
    ament_add_catch2(
      test_nexus_lifecycle_manager
      test/main.cpp
      test/test_lifecycle_manager.cpp
      TIMEOUT 30)
    target_link_libraries(test_nexus_lifecycle_manager
      rclcpp::rclcpp
      rclcpp_lifecycle::rclcpp_lifecycle
      ${PROJECT_NAME}_core
    )
  endif()
endif()

ament_export_targets(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
