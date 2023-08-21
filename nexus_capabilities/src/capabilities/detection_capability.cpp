/*
 * Copyright (C) 2022 Johnson & Johnson
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

#include "detection_capability.hpp"

#include "detection.hpp"

#include <memory>
#include <string>

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

//==============================================================================
void DetectionCapability::declare_params(
  rclcpp_lifecycle::LifecycleNode& node)
{
  ParameterDescriptor desc;
  desc.read_only = true;
  desc.description = "List of detectors available";
  node.declare_parameter("detectors", std::vector<std::string>{}, desc);
}

//==============================================================================
void DetectionCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> ctx_mgr,
  BT::BehaviorTreeFactory& bt_factory)
{
  const auto detectors_param = node->get_parameter("detectors");
  const auto& detectors = detectors_param.as_string_array();
  if (detectors.empty())
  {
    RCLCPP_WARN(node->get_logger(), "No detectors defined");
  }
  for (const auto& detector : detectors)
  {
    this->_clients.emplace(detector,
      node->create_client<endpoints::DetectorService::ServiceType>(endpoints::
      DetectorService::service_name(detector)));
  }

  bt_factory.registerBuilder<DetectOffset>(
    "detection.DetectOffset",
    [this, ctx_mgr, w_node = std::weak_ptr{node}](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }

      return std::make_unique<DetectOffset>(name, config,
      node->get_logger(), ctx_mgr, [this, node](const std::string& detector)
      {
        return this->_clients.at(detector);
      });
    });

  bt_factory.registerBuilder<DetectAllItems>(
    "detection.DetectAllItems",
    [this, ctx_mgr, w_node = std::weak_ptr{node}](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }

      return std::make_unique<DetectAllItems>(name, config, node, ctx_mgr,
      [this](const std::string& detector)
      {
        return this->_clients.at(detector);
      });
    });

  bt_factory.registerBuilder<GetDetection>(
    "detection.GetDetection",
    [ctx_mgr](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetDetection>(name, config, ctx_mgr);
    });

  bt_factory.registerBuilder<GetDetectionPose>(
    "detection.GetDetectionPose",
    [ctx_mgr](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetDetectionPose>(name, config, ctx_mgr);
    });
}

//==============================================================================
void DetectionCapability::activate()
{
  // Do nothing.
}

//==============================================================================
void DetectionCapability::deactivate()
{
  // Do nothing.
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::DetectionCapability,
  nexus::Capability
)
