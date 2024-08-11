cmake_minimum_required(VERSION 3.8)
project(nexus_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(dep_pkgs
  nexus_endpoints
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2
  VRPN
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
add_library(nexus_calibration_component SHARED src/calibration_node.cpp)

ament_target_dependencies(nexus_calibration_component ${dep_pkgs})

target_compile_features(nexus_calibration_component INTERFACE cxx_std_17)

rclcpp_components_register_node(nexus_calibration_component
  PLUGIN "nexus::CalibrationNode"
  EXECUTABLE nexus_calibration_node
  EXECUTOR SingleThreadedExecutor)


#===============================================================================
if(BUILD_TESTING)

endif()

#===============================================================================
install(
  TARGETS
    nexus_calibration_component
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
