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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__SERIALIZATION_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__SERIALIZATION_HPP

// Required for YAML deserialization into Detection3DArray msg.
#include <nexus_common/models/vision.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::workcell_orchestrator {

/**
 * Serialize a `vision_msgs::msg::Detection3DArray` into yaml.
 * Input Ports:
 *   detections |vision_msgs::msg::Detection3DArray|
 * Output Ports:
 *   result |std::string|
 */
class SerializeDetections : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts();

public: SerializeDetections(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node)
  : BT::SyncActionNode(name, config), _node(node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode& _node;
};

/**
 * Deserialize a yaml into `vision_msgs::msg::Detection3DArray`.
 * Input Ports:
 *   yaml |std::string|
 * Output Ports:
 *   result |vision_msgs::msg::Detection3DArray|
 */
class DeserializeDetections : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts();

public: DeserializeDetections(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node)
  : BT::SyncActionNode{name, config}, _node{node} {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode& _node;
};

}

#endif // NEXUS_WORKCELL_ORCHESTRATOR__SERIALIZATION_HPP
