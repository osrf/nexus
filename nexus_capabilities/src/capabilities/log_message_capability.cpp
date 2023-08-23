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

#include "log_message_capability.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

void LogMessageCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  bt_factory.registerSimpleAction("log_message",
    [this, w_node = std::weak_ptr{node}](BT::TreeNode& bt)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }
      return print_str(bt, *node);
    },
    {BT::InputPort<std::string>("msg")}
  );

  bt_factory.registerSimpleAction("log_message.Pose",
    [this, w_node = std::weak_ptr{node}](BT::TreeNode& bt)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }
      return print_msg<geometry_msgs::msg::Pose>(bt, *node);
    },
    {BT::InputPort<geometry_msgs::msg::Pose>("msg")}
  );

  bt_factory.registerSimpleAction("log_message.PoseStamped",
    [this, w_node = std::weak_ptr{node}](BT::TreeNode& bt)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }
      return print_msg<geometry_msgs::msg::PoseStamped>(bt, *node);
    },
    {BT::InputPort<geometry_msgs::msg::PoseStamped>("msg")}
  );

  bt_factory.registerSimpleAction("log_message.Transform",
    [this, w_node = std::weak_ptr{node}](BT::TreeNode& bt)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }
      return print_msg<geometry_msgs::msg::Transform>(bt, *node);
    },
    {BT::InputPort<geometry_msgs::msg::Transform>("msg")}
  );

  bt_factory.registerSimpleAction("log_message.TransformStamped",
    [this, w_node = std::weak_ptr{node}](BT::TreeNode& bt)
    {
      auto node = w_node.lock();
      if (!node)
      {
        std::cerr << "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES!!!" << std::endl;
        std::terminate();
      }
      return print_msg<geometry_msgs::msg::TransformStamped>(bt, *node);
    },
    {BT::InputPort<geometry_msgs::msg::TransformStamped>("msg")}
  );
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::LogMessageCapability,
  nexus::Capability
)
