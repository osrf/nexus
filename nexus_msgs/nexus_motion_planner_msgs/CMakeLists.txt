cmake_minimum_required(VERSION 3.8)
project(nexus_motion_planner_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set (dependencies
  geometry_msgs
  moveit_msgs
)

set(srv_files
  "srv/GetMotionPlan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
    ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
