cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(nexus_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(VRPN REQUIRED)

#===============================================================================
add_library(${PROJECT_NAME} SHARED
  src/MotionCaptureSystem.cc
  src/MotionCaptureRigidBody.cc
)

target_link_libraries(${PROJECT_NAME}
  gz-sim::core
  ${VRPN_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}>
  ${VRPN_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  gz-sim
  VRPN)


#===============================================================================
if(BUILD_TESTING)

endif()

#===============================================================================
install(DIRECTORY models/motion_capture_system DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY models/motion_capture_rigid_body DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY include/nexus_gazebo DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

ament_environment_hooks("nexus_gazebo_gazebo_paths.dsv.in")

ament_package()
