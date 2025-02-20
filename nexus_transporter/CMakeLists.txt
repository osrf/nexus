cmake_minimum_required(VERSION 3.8)
project(nexus_transporter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
set(dep_pkgs
  nexus_endpoints
  nexus_transporter_msgs
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  rmf_utils
  tf2_ros
  yaml_cpp_vendor
  yaml-cpp
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
add_library(nexus_transporter SHARED
  src/nexus_transporter/Itinerary.cpp
)

target_link_libraries(nexus_transporter
  PUBLIC
    ${nexus_transporter_msgs_TARGETS}
    rclcpp::rclcpp
    ${rclcpp_action_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    rmf_utils::rmf_utils
    yaml-cpp::yaml-cpp
)

target_include_directories(nexus_transporter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${nexus_transporter_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${rmf_utils_INCLUDE_DIRS}
)

target_compile_features(nexus_transporter INTERFACE cxx_std_17)

ament_export_targets(nexus_transporter HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle nexus_transporter_msgs rmf_utils)
#===============================================================================
add_library(nexus_transporter_component SHARED src/TransporterNode.cpp)

target_link_libraries(nexus_transporter_component
  PUBLIC
    nexus_endpoints::nexus_endpoints
    nexus_transporter
    ${nexus_transporter_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${rclcpp_components_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${tf2_ros_TARGETS}
    yaml-cpp::yaml-cpp
)

target_include_directories(nexus_transporter_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${nexus_endpoints_INCLUDE_DIRS}
    ${nexus_transporter_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

target_compile_features(nexus_transporter_component INTERFACE cxx_std_17)

rclcpp_components_register_node(nexus_transporter_component
  PLUGIN "nexus_transporter::TransporterNode"
  EXECUTABLE nexus_transporter_node
  EXECUTOR SingleThreadedExecutor)


#===============================================================================
# Mock transporter plugin.
add_library(mock_transporter SHARED src/transporters/mock_transporter.cpp)
target_include_directories(mock_transporter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlub_INCLUDE_DIRS}
)
target_link_libraries(mock_transporter
  PUBLIC
    nexus_transporter
    pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nexus_transporter src/transporters/plugins.xml)

ament_export_libraries(mock_transporter)

#===============================================================================
find_package(ament_cmake_uncrustify QUIET)
find_package(ament_cmake_catch2 QUIET)

if(BUILD_TESTING AND ament_cmake_uncrustify_FOUND AND ament_cmake_catch2_FOUND)
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

  ament_add_catch2(test_nexus_transporter
    test/main.cpp
    test/test_Itinerary.cpp
    TIMEOUT 300)
  target_link_libraries(test_nexus_transporter
    nexus_transporter
  )

  target_include_directories(test_nexus_transporter
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
endif()

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS
    mock_transporter
    nexus_transporter
    nexus_transporter_component
  EXPORT nexus_transporter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
