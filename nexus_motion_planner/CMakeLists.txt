cmake_minimum_required(VERSION 3.8)
project(nexus_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
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
  tf2
  tf2_ros
)

set (motion_plan_cache_dependencies
  moveit_ros_planning_interface
  rclcpp
  warehouse_ros
  warehouse_ros_sqlite
)

set (test_request_dependencies
  moveit_msgs
  nexus_endpoints
  geometry_msgs
  rclcpp
  trajectory_msgs
)

#===============================================================================
set(MOTION_PLAN_CACHE_LIBRARY_NAME motion_planner_server_core)
set(EXECUTABLE_NAME motion_planner_server)

# Motion plan cache library
add_library(${MOTION_PLAN_CACHE_LIBRARY_NAME} src/motion_plan_cache.cpp)
ament_target_dependencies(${MOTION_PLAN_CACHE_LIBRARY_NAME} ${motion_plan_cache_dependencies})

# Server executable
add_executable(${EXECUTABLE_NAME} src/main.cpp src/motion_planner_server.cpp)
ament_target_dependencies(${EXECUTABLE_NAME} ${motion_planner_server_dependencies})
target_link_libraries(${EXECUTABLE_NAME} ${MOTION_PLAN_CACHE_LIBRARY_NAME})

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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmf_utils REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  add_executable(test_motion_plan_cache src/test_motion_plan_cache.cpp)
  target_link_libraries(test_motion_plan_cache ${MOTION_PLAN_CACHE_LIBRARY_NAME})

  install(TARGETS
      test_motion_plan_cache
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  # TODO(luca) Add uncrustify back after formatting
  #ament_uncrustify(
  #  ARGN include src test
  #  CONFIG_FILE ${uncrustify_config_file}
  #  MAX_LINE_LENGTH 80
  #  LANGUAGE CPP
  #)

  # TODO(luca) reintroduce this test, it was failing since the beginning so unclear
  # what it it supposed to do
  #ament_add_pytest_test(test_motion_plan_cache_py "test/test_motion_plan_cache.py"
  # WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  #)

  # Motion planner server test with cache mode unset
  add_launch_test(
    test/test_request.test.py
    TIMEOUT 60
  )

  # Motion planner server test with cache mode set (so it uses the cache)
  add_launch_test(
    test/test_request_with_cache.test.py
    TIMEOUT 60
  )
endif()

ament_export_dependencies(${motion_planner_server_dependencies} ${motion_plan_cache_dependencies})
ament_package()
