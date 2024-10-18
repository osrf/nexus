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

#include "plan_motion_capability.hpp"

#include "plan_motion.hpp"

#include <nexus_capabilities/exceptions.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace nexus::capabilities {

void PlanMotionCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  this->_client =
    node->create_client<endpoints::GetMotionPlanService::ServiceType>(
    endpoints::GetMotionPlanService::service_name());
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);
  bt_factory.registerBuilder<PlanMotion>("plan_motion.PlanMotion",
    [this, node, tf_broadcaster](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<PlanMotion>(name, config, *node, this->_client,
      tf_broadcaster);
    });
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::PlanMotionCapability,
  nexus::Capability
)
