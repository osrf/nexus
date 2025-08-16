cmake_minimum_required(VERSION 3.8)
project(nexus_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(include)

set (dependencies
  nexus_endpoints
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
  warehouse_ros
  warehouse_ros_sqlite
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

#===============================================================================
set(MOTION_PLAN_CACHE_LIBRARY_NAME motion_planner_server_core)
set(EXECUTABLE_NAME motion_planner_server)

# Motion plan cache library
add_library(${MOTION_PLAN_CACHE_LIBRARY_NAME} src/motion_plan_cache.cpp)
target_link_libraries(${MOTION_PLAN_CACHE_LIBRARY_NAME} PUBLIC
  ${moveit_ros_planning_interface_TARGETS}
  rclcpp::rclcpp
  warehouse_ros::warehouse_ros
  warehouse_ros_sqlite::warehouse_ros_sqlite
)

# Server executable
add_executable(${EXECUTABLE_NAME} src/main.cpp src/motion_planner_server.cpp)
target_link_libraries(${EXECUTABLE_NAME}
  nexus_endpoints::nexus_endpoints
  ${MOTION_PLAN_CACHE_LIBRARY_NAME}
)

# Component
add_library(motion_planner_server_component SHARED
  src/motion_planner_server_component.cpp
  src/motion_planner_server.cpp
)

target_link_libraries(motion_planner_server_component PRIVATE
  nexus_endpoints::nexus_endpoints
  ${MOTION_PLAN_CACHE_LIBRARY_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
)

target_compile_features(motion_planner_server_component INTERFACE cxx_std_17)

rclcpp_components_register_node(motion_planner_server_component
  PLUGIN "nexus::motion_planner::MotionPlannerServerComponent"
  EXECUTABLE motion_planner_server_component_node
  EXECUTOR SingleThreadedExecutor
)

# Test executable
add_executable(test_request src/test_request.cpp)
target_link_libraries(test_request
  ${moveit_msgs_TARGETS}
  nexus_endpoints::nexus_endpoints
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${trajectory_msgs_TARGETS}
)

#===============================================================================
install(TARGETS
    ${MOTION_PLAN_CACHE_LIBRARY_NAME}
    motion_planner_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_dependencies(${dependencies})
ament_package()
