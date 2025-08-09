cmake_minimum_required(VERSION 3.5)
project(nexus_robot_controller)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(include)

set(library_name robot_controller_server_core)
set(executable_name robot_controller_server)

set(dependencies
  controller_manager
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# Library
add_library(${library_name} src/robot_controller_server.cpp)
target_link_libraries(${library_name} PUBLIC
  controller_manager::controller_manager
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS}
)

# Executable
add_executable(${executable_name} src/robot_controller_server_node.cpp)
target_link_libraries(${executable_name} ${library_name})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(rmf_utils QUIET)
  find_package(ament_cmake_uncrustify QUIET)

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

  find_package(launch_testing_ament_cmake)
  install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
  add_launch_test(test/robot_controller_abb_irb910sc.test.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
