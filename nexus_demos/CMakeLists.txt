cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_demos)
project(${PROJECT_NAME} VERSION 0.0.1)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(nexus_transporter REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

###############################################################################

add_library(mock_gripper_component SHARED src/mock_gripper.cpp)
target_link_libraries(mock_gripper_component PRIVATE
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_compile_features(mock_gripper_component INTERFACE cxx_std_17)
rclcpp_components_register_node(mock_gripper_component
  PLUGIN "nexus_demos::MockGripper"
  EXECUTABLE mock_gripper
  EXECUTOR SingleThreadedExecutor)
install(
  TARGETS mock_gripper_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

###############################################################################

add_library(mock_detector_component SHARED src/mock_detector.cpp)
target_link_libraries(mock_detector_component PRIVATE
  nexus_endpoints::nexus_endpoints
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  yaml-cpp::yaml-cpp
  ${tf2_ros_TARGETS}
)
target_include_directories(mock_detector_component PRIVATE
  ${tf2_ros_INCLUDE_DIRS}
)
target_compile_features(mock_detector_component INTERFACE cxx_std_17)
rclcpp_components_register_node(mock_detector_component
  PLUGIN "nexus_demos::MockDetector"
  EXECUTABLE mock_detector
  EXECUTOR SingleThreadedExecutor)
install(
  TARGETS mock_detector_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
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
install(
  TARGETS mock_emergency_alarm_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
###############################################################################
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

install(
  TARGETS
    mock_printer
    EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

if(BUILD_TESTING)
  find_package(rmf_utils REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_catch2 QUIET)

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

  if (ament_cmake_catch2_FOUND)
    find_package(ament_index_cpp REQUIRED)
    find_package(rcpputils REQUIRED)

    ament_add_catch2(test_mocks
      test/main.cpp
      src/test_mock_detector.cpp
      src/test_mock_gripper.cpp
    )
    target_link_libraries(test_mocks
      ament_index_cpp::ament_index_cpp
      mock_detector_component
      mock_gripper_component
      nexus_endpoints::nexus_endpoints
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      rclcpp_components::component
      rclcpp_lifecycle::rclcpp_lifecycle
      rcpputils::rcpputils
      yaml-cpp::yaml-cpp
      rmf_utils::rmf_utils
      ${tf2_ros_TARGETS}
    )
    target_include_directories(test_mocks PUBLIC
      ${tf2_ros_INCLUDE_DIRS}
    )
  endif()
endif()

# Modified from rmf_demos_maps
# Get absolute path of the building map
set(building_map "config/rmf/maps/depot/depot.building.yaml")
get_filename_component(building_map_path ${building_map} REALPATH)

# Get the output world name
string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${building_map_path})
string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

set(output_world_name ${world_name})
set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/config/rmf/maps/${output_world_name})
set(output_world_path ${output_dir}/${output_world_name}.world)
set(output_model_dir ${output_dir}/models)

##############################################################################
# Generate Gz world, download Models, change robot to use Fuel model
##############################################################################

message("BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${building_map_path} ${output_world_path} ${output_model_dir}")
message("REPLACING ROBOT MODELS WITH THOSE FROM FUEL")
if (NO_DOWNLOAD_MODELS)
  add_custom_command(
    DEPENDS ${building_map_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${building_map_path} ${output_world_path} ${output_model_dir} --TEMPLATE_WORLD_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/rmf/maps/depot/depot_template.sdf --SKIP_CAMERA_POSE
    COMMAND sed -i 's|<uri>model:\/\/DeliveryRobot<\/uri>|<uri>https:\/\/fuel.gazebosim.org\/1.0\/Open-RMF\/models\/DeliveryRobotWithConveyor<\/uri>|g' ${output_world_path}
    OUTPUT ${output_world_path}
  )
else()
  message("DOWNLOADING MODELS WITH COMMAND: ros2 run rmf_building_map_tools building_map_model_downloader ${building_map_path}")
  add_custom_command(
    DEPENDS ${building_map_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${building_map_path} ${output_world_path} ${output_model_dir} --TEMPLATE_WORLD_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/rmf/maps/depot/depot_template.sdf --SKIP_CAMERA_POSE
    COMMAND sed -i 's|<uri>model:\/\/DeliveryRobot<\/uri>|<uri>https:\/\/fuel.gazebosim.org\/1.0\/Open-RMF\/models\/DeliveryRobotWithConveyor<\/uri>|g' ${output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${building_map_path} -e ~/.gazebo/models
    OUTPUT ${output_world_path}
  )
endif()

##############################################################################
# Generate the nav graphs
##############################################################################

set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
add_custom_command(
  OUTPUT ${output_nav_graphs_phony}
  COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${building_map_path} ${output_nav_graphs_dir}
  DEPENDS ${building_map_path} ${output_world_path}
)

add_custom_target(generate_${output_world_name}_nav_graphs ALL
  DEPENDS ${output_nav_graphs_phony}
)

install(
  DIRECTORY ${output_dir}
  DESTINATION share/${PROJECT_NAME}/config/rmf/maps
)

ament_package()
