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

#include <chrono>

#include "workcell_state_visualizer.hpp"

using namespace std::chrono_literals;

namespace nexus::visualization {

//==============================================================================
WorkcellStateVisualizer::WorkcellStateVisualizer(const rclcpp::NodeOptions& options)
: Node("workcell_state_visualizer", options),
  _next_available_id(0)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring workcell_state_visualizer..."
  );

  _current_level = this->declare_parameter("initial_map_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter initial_map_name to %s", _current_level.c_str()
  );

  auto map_qos = rclcpp::QoS(10).transient_local();
  _map_sub = this->create_subscription<BuildingMap>(
    "/map",
    map_qos,
    [=](BuildingMap::ConstSharedPtr msg)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Received map"
      );
      // Note that this implementation depends on vertices being in the navgraphs
      // We should probably parse the map but it could be in pixel units which is tricky
      for (const auto& level : msg->levels)
      {
        for (const auto& graph : level.nav_graphs)
        {
          for (const auto& vertex : graph.vertices)
          {
            for (const auto& param : vertex.params)
            {
              if (param.name == "pickup_dispenser")
              {
                RCLCPP_INFO(
                  this->get_logger(),
                  "Found dispenser [%s]", param.value_string.c_str()
                );
                WorkcellState workcell_state;
                workcell_state.x = vertex.x;
                workcell_state.y = vertex.y;
                workcell_state.map_name = level.name;
                _workcell_states.insert({vertex.name, workcell_state});
                _ids.insert({vertex.name, _next_available_id});
                ++_next_available_id;
              }
            }
          }
        }
      }
      initialize_state_subscriptions();
      _publish_timer = this->create_wall_timer(1s, std::bind(&WorkcellStateVisualizer::timer_cb, this));
    });

  _param_sub = this->create_subscription<RvizParam>(
    "rmf_visualization/parameters",
    rclcpp::SystemDefaultsQoS(),
    [=](RvizParam::ConstSharedPtr msg)
    {
      if (msg->map_name.empty() || msg->map_name == _current_level)
        return;

      _current_level = msg->map_name;
      timer_cb();
    });

  _marker_pub = this->create_publisher<MarkerArray>("/workcell_markers", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Running workcell_state_visualizer..."
  );

}

void WorkcellStateVisualizer::initialize_state_subscriptions()
{
  for (const auto& [name, _] : _workcell_states)
  {
    _state_subs.emplace_back(nexus::endpoints::WorkcellStateTopic::create_subscription(this, name,
      [this](nexus::endpoints::WorkcellStateTopic::MessageType::ConstSharedPtr msg)
      {
        auto it = _workcell_states.find(msg->workcell_id);
        if (it == _workcell_states.end())
        {
          RCLCPP_WARN(this->get_logger(), "Received state for workcell [%s] but it was not found in the map", msg->workcell_id.c_str());
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Received state for workcell [%s]", msg->workcell_id.c_str());
        auto& workcell_state = it->second;
        workcell_state.status = msg->status;
        workcell_state.message = msg->message.empty() ? "<None>" : msg->message;
        workcell_state.task_id = msg->task_id.empty() ? "<None>" : msg->task_id;
      }));
  }
}

void WorkcellStateVisualizer::timer_cb()
{
  auto set_body_pose =
  [](const WorkcellState& state, Marker& marker)
  {
    marker.pose.position.x = state.x;
    marker.pose.position.y = state.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
  };

  auto set_text_pose =
  [](const WorkcellState& state, const double x_offset, Marker& marker)
  {
    marker.pose.position.x = state.x + x_offset;
    marker.pose.position.y = state.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
  };

  auto fill_markers =
  [&](
    const std::string& name,
    const WorkcellState& state,
    const double radius,
    const std::size_t id,
    MarkerArray& marker_array)
  {
    Marker body_marker;
    body_marker.header.frame_id = "map";
    body_marker.header.stamp = this->get_clock()->now();
    body_marker.ns = "body";
    body_marker.id = id;
    body_marker.type = body_marker.CUBE;
    body_marker.action = body_marker.MODIFY;
    set_body_pose(state, body_marker);
    body_marker.scale.x = radius;
    body_marker.scale.y = radius;
    body_marker.scale.z = radius;
    // TODO(luca) color to match state
    Color color;
    std::string status;
    color.a = 1.0;
    if (state.status == 0)
    {
      // Indeterminate, grey
      color.r = 0.3;
      color.g = 0.3;
      color.b = 0.3;
      status = "UNKNOWN";
    } else if (state.status == WorkcellStateMsg::STATUS_IDLE) {
      // Idle, green
      color.r = 0.0;
      color.g = 0.8;
      color.b = 0.0;
      status = "IDLE";
    } else if (state.status == WorkcellStateMsg::STATUS_BUSY) {
      // Busy, yellow 
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.0;
      status = "BUSY";
    }
    body_marker.color = color;
    body_marker.lifetime = rclcpp::Duration(std::chrono::seconds(1));

    auto text_marker = body_marker;
    text_marker.ns = "name";
    text_marker.type = text_marker.TEXT_VIEW_FACING;
    set_text_pose(state, radius * 2.0, text_marker);
    text_marker.text = name + "\n" + "STATUS:" + status + "\n" + "TASK:" + state.task_id + "\n" + "MESSAGE:" + state.message;
    text_marker.scale.z = 0.3;

    marker_array.markers.push_back(std::move(body_marker));
    marker_array.markers.push_back(std::move(text_marker));
  };

  auto marker_array = std::make_unique<MarkerArray>();
  for (const auto& [name, state] : _workcell_states)
  {
    if (state.map_name != _current_level)
      continue;

    double radius = 1.0;
    fill_markers(
      name,
      state,
      radius,
      _ids[name],
      *marker_array
    );
  }
  if (!marker_array->markers.empty())
    _marker_pub->publish(std::move(marker_array));
}
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(nexus::visualization::WorkcellStateVisualizer)
