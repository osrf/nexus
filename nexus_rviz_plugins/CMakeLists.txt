cmake_minimum_required(VERSION 3.8)
project(nexus_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nexus_endpoints REQUIRED)
find_package(nexus_lifecycle_manager REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)


# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(nexus_rviz_plugins_headers_to_moc
  include/nexus_rviz_plugins/nexus_panel.hpp
  include/nexus_rviz_plugins/ros_action_qevent.hpp
)

add_library(${library_name} SHARED
  src/nexus_panel.cpp
  ${nexus_rviz_plugins_headers_to_moc}
)

ament_target_dependencies(${library_name}
  nexus_endpoints
  nexus_lifecycle_manager
  lifecycle_msgs
  rclcpp
  rviz_common
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
)

if(BUILD_TESTING)
  find_package(rmf_utils REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
    LANGUAGE C++
  )
endif()

ament_package()
