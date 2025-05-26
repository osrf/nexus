cmake_minimum_required(VERSION 3.8)
project(nexus_orchestrator_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nexus_transporter_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/WorkcellTask.action"
  "action/ExecuteWorkOrder.action"
)

set(msg_files
  "msg/TaskProgress.msg"
  "msg/TaskState.msg"
  "msg/WorkcellDescription.msg"
  "msg/WorkcellState.msg"
  "msg/WorkcellTask.msg"
  "msg/WorkOrder.msg"
  "msg/WorkOrderState.msg"
)

set(srv_files
  "srv/BidTransporter.srv"
  "srv/GetWorkOrderState.srv"
  "srv/IsTaskDoable.srv"
  "srv/ListTransporters.srv"
  "srv/ListWorkcells.srv"
  "srv/PauseSystem.srv"
  "srv/PauseWorkcell.srv"
  "srv/QueueWorkcellTask.srv"
  "srv/RegisterTransporter.srv"
  "srv/RegisterWorkcell.srv"
  "srv/RemovePendingTask.srv"
  "srv/SignalWorkcell.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    nexus_transporter_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
