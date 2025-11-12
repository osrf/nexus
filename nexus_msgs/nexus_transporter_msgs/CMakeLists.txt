cmake_minimum_required(VERSION 3.8)
project(nexus_transporter_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nexus_orchestrator_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Destination.msg"
  "msg/TransportationRequest.msg"
  "msg/TransporterState.msg"
)

set(srv_files
  "srv/IsTransporterAvailable.srv"
)

set(action_files
  "action/Transport.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    nexus_orchestrator_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
