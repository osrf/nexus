cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME nexus_demos)
project(${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})

# Refactored from rmf_demos_maps, we just need to download models
file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")
foreach(building_map_path ${traffic_editor_paths})
  ##############################################################################
  # Downloads Models
  ##############################################################################
  if (NOT NO_DOWNLOAD_MODELS)
    message("DOWNLOADING MODELS WITH COMMAND: ros2 run rmf_building_map_tools building_map_model_downloader ${building_map_path}")
    execute_process(
      COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${building_map_path} -e ~/.gazebo/models
    )
  endif()
endforeach()

ament_package()
