cmake_minimum_required(VERSION 3.8)
project(nexus_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rclcpp_components
  nexus_endpoints
  nexus_orchestrator_msgs
  rmf_building_map_msgs
  rmf_visualization_msgs
  geometry_msgs
  visualization_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
add_library(workcell_state_visualizer SHARED src/workcell_state_visualizer.cpp)

target_link_libraries(workcell_state_visualizer
  PUBLIC
    rclcpp::rclcpp
    ${rclcpp_components_TARGETS}
    nexus_endpoints::nexus_endpoints
    ${nexus_orchestrator_msgs_TARGETS}
    ${rmf_building_map_msgs_TARGETS}
    ${rmf_visualization_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)

target_include_directories(workcell_state_visualizer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(workcell_state_visualizer INTERFACE cxx_std_17)

rclcpp_components_register_node(workcell_state_visualizer
  PLUGIN "workcell_state_visualizer"
  EXECUTABLE workcell_state_visualizer_node)

#===============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")


  # TODO(luca) reintroduce uncrustify
  #ament_uncrustify(
  #  ARGN src
  #  CONFIG_FILE ${uncrustify_config_file}
  #  MAX_LINE_LENGTH 80
  #)

endif()

#===============================================================================
install(
  TARGETS workcell_state_visualizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
