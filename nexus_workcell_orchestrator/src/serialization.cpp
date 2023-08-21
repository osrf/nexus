/*
 * Copyright (C) 2023 Johnson & Johnson
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

#include "serialization.hpp"

#include <rclcpp/logging.hpp>

#include <yaml-cpp/yaml.h>

namespace nexus::workcell_orchestrator {

BT::PortsList SerializeDetections::providedPorts()
{
  return { BT::InputPort<vision_msgs::msg::Detection3DArray>("detections"),
    BT::OutputPort<std::string>("result") };
}

BT::NodeStatus SerializeDetections::tick()
{
  auto detections = this->getInput<vision_msgs::msg::Detection3DArray>(
    "detections");
  if (!detections)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "%s: port [detections] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->setOutput("result", vision_msgs::msg::to_yaml(*detections));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList DeserializeDetections::providedPorts()
{
  return { BT::InputPort<std::string>("yaml"),
    BT::OutputPort<vision_msgs::msg::Detection3DArray>("result") };
}

BT::NodeStatus DeserializeDetections::tick()
{
  auto maybe_yaml = this->getInput<std::string>("yaml");
  if (!maybe_yaml || maybe_yaml->empty())
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "[%s]: [yaml] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    auto pose =
      YAML::Load(*maybe_yaml).as<vision_msgs::msg::Detection3DArray>();
    this->setOutput("result", pose);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "[%s]: Failed to deserialize detections [%s]",
      this->name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}
