cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_integration_tests)
project(${PROJECT_NAME} VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(nexus_transporter REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

###############################################################################

add_executable(mock_gripper src/mock_gripper.cpp)
target_link_libraries(mock_gripper PRIVATE
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
)
install(TARGETS mock_gripper RUNTIME DESTINATION lib/${PROJECT_NAME})

###############################################################################

add_executable(mock_detection src/mock_detection.cpp)
target_link_libraries(mock_detection PRIVATE
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${yaml_cpp_vendor_TARGETS}
  ${tf2_ros_TARGETS}
)
target_include_directories(mock_detection PRIVATE
  ${tf2_ros_INCLUDE_DIRS}
)
install(TARGETS mock_detection RUNTIME DESTINATION lib/${PROJECT_NAME})

###############################################################################

add_library(mock_emergency_alarm_component SHARED src/mock_emergency_alarm.cpp)
target_link_libraries(mock_emergency_alarm_component
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
)
ament_target_dependencies(mock_emergency_alarm_component
  std_srvs
  rclcpp_components
)
rclcpp_components_register_node(mock_emergency_alarm_component
  PLUGIN "MockEmergencyAlarm"
  EXECUTABLE mock_emergency_alarm
  EXECUTOR SingleThreadedExecutor)
install(TARGETS mock_emergency_alarm_component RUNTIME DESTINATION lib/${PROJECT_NAME})

###############################################################################

add_library(mock_transporter SHARED src/mock_transporter.cpp)
target_include_directories(mock_transporter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlub_INCLUDE_DIRS}
)
target_link_libraries(mock_transporter
  PUBLIC
    nexus_transporter::nexus_transporter
    pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nexus_transporter mock_plugins.xml)

ament_export_libraries(mock_transporter)
ament_export_targets(export_${PROJECT_NAME})

###############################################################################
add_library(mock_printer SHARED src/mock_printer.cpp)
target_include_directories(mock_printer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nexus_endpoins_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(mock_printer PRIVATE
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rclcpp_components_TARGETS}
)
target_compile_features(mock_printer INTERFACE cxx_std_17)
rclcpp_components_register_node(mock_printer
  PLUGIN "MockPrinter"
  EXECUTABLE mock_printer_node
  EXECUTOR SingleThreadedExecutor)

###############################################################################
add_test(NAME all_tests COMMAND python3 -m unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

###############################################################################
# Generate zenoh bridge configuration files from a NEXUS network configuration file
set(nexus_network_cfg_path ${CMAKE_CURRENT_SOURCE_DIR}/config/zenoh/nexus_network_config.yaml)
set(zenoh_cfg_output_dir ${CMAKE_CURRENT_BINARY_DIR}/config/zenoh)

# Invoke the nexus_network_configuration script and generate config json within the build directory.
add_custom_target(generate_zenoh_bridge_configs ALL
  COMMAND ros2 run nexus_network_configuration nexus_network_configuration -n ${nexus_network_cfg_path} -o ${zenoh_cfg_output_dir}
)

install(DIRECTORY launch config rviz scripts DESTINATION share/${PROJECT_NAME})
# Install the zenoh config directory containing generated configs.
message("zenoh_cfg_output_dir: " ${zenoh_cfg_output_dir})
install(DIRECTORY ${zenoh_cfg_output_dir} DESTINATION share/${PROJECT_NAME}/config/)

install(
  TARGETS
    mock_transporter
    mock_printer
    EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(rmf_utils REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_catch2 QUIET)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
    LANGUAGE C++
  )

  if (ament_cmake_catch2_FOUND)
    find_package(ament_index_cpp REQUIRED)
    find_package(rcpputils REQUIRED)

    # To run the test use:
    # ros2 run nexus_integration_tests test_mocks
    add_executable(test_mocks
      test/main.cpp
      src/test_mock_detection.cpp
      src/test_mock_gripper.cpp
    )
    target_link_libraries(test_mocks
      ament_index_cpp::ament_index_cpp
      nexus_endpoints::nexus_endpoints
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      rclcpp_lifecycle::rclcpp_lifecycle
      rcpputils::rcpputils
      ${yaml_cpp_vendor_TARGETS}
      rmf_utils::rmf_utils
      ${tf2_ros_TARGETS}
    )
    target_include_directories(test_mocks PUBLIC
      ${yaml_cpp_vendor_INCLUDE_DIRS}
      ${tf2_ros_INCLUDE_DIRS}
    )
    install(TARGETS test_mocks RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

ament_package()
