/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef SRC__WORKCELLSTATEVISUALIZER_HPP
#define SRC__WORKCELLSTATEVISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_state.hpp>

#include <rmf_building_map_msgs/msg/building_map.hpp>
#include <rmf_visualization_msgs/msg/rviz_param.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>
#include <unordered_set>

//==============================================================================
class WorkcellStateVisualizer : public rclcpp::Node
{
public:
  using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;
  using RvizParam = rmf_visualization_msgs::msg::RvizParam;
  using WorkcellStateMsg = nexus_orchestrator_msgs::msg::WorkcellState;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Color = std_msgs::msg::ColorRGBA;

/// Constructor
  WorkcellStateVisualizer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  struct WorkcellState {
    double x;
    double y;
    std::string map_name;

    // Follows enum in WorkcellState.msg
    // 0 indefinite, 1 idle, 2 busy
    uint8_t status = 0;
    std::string task_id = "<None>";
    std::string message = "<None>";
  };

  void initialize_state_subscriptions();

  void timer_cb();

  std::string _current_level;

  rclcpp::TimerBase::SharedPtr _publish_timer;
  rclcpp::Subscription<BuildingMap>::SharedPtr _map_sub;
  rclcpp::Subscription<RvizParam>::SharedPtr _param_sub;
  std::vector<rclcpp::Subscription<nexus::endpoints::WorkcellStateTopic::MessageType>::SharedPtr> _state_subs;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_pub;

  // Map workcell name to a unique marker id
  std::unordered_map<std::string, std::size_t> _ids;
  std::size_t _next_available_id;

  std::unordered_map<std::string, WorkcellState> _workcell_states;
};


#endif // SRC__WORKCELLSTATEVISUALIZER_HPP
