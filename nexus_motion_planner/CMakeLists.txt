cmake_minimum_required(VERSION 3.8)
project(nexus_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(warehouse_ros_sqlite REQUIRED)

include_directories(include)

set (motion_planner_server_dependencies
  nexus_endpoints
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  warehouse_ros
  warehouse_ros_sqlite
  tf2
  tf2_ros
)

set (test_request_dependencies
  moveit_msgs
  nexus_endpoints
  geometry_msgs
  rclcpp
  trajectory_msgs
)

#===============================================================================
set(LIBRARY_NAME motion_planner_server_core)
set(EXECUTABLE_NAME motion_planner_server)

# Server executable
add_executable(${EXECUTABLE_NAME} src/main.cpp src/motion_planner_server.cpp src/motion_plan_cache.cpp)
ament_target_dependencies(${EXECUTABLE_NAME} ${motion_planner_server_dependencies})

# Test executable
add_executable(test_request src/test_request.cpp)
ament_target_dependencies(test_request ${test_request_dependencies})

#===============================================================================
install(TARGETS
    ${EXECUTABLE_NAME}
    test_request
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmf_utils REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
    LANGUAGE CPP
  )

  # TODO(YV): Fix these tests.
  # add_launch_test(
  #   test/test_request.test.py
  #   TIMEOUT 60
  # )
endif()

ament_export_dependencies(${motion_planner_server_dependencies})
ament_package()
