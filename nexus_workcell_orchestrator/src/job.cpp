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

#include "job.hpp"

#include "get_result.hpp"
#include "make_transform.hpp"
#include "serialization.hpp"
#include "get_joint_constraints.hpp"
#include "set_result.hpp"
#include "signals.hpp"
#include "transform_pose.hpp"

#include <nexus_common/pausable_sequence.hpp>

namespace nexus::workcell_orchestrator {

Job::Job(rclcpp_lifecycle::LifecycleNode::WeakPtr node,
  tf2_ros::Buffer::SharedPtr tf2_buffer)
: _node(std::move(node)), _tf2_buffer(std::move(tf2_buffer))
{
  // create IsPauseTriggered BT node, it is a simple action node with output port "paused".
  this->_bt_factory.registerSimpleAction("IsPauseTriggered",
    [this](BT::TreeNode& node)
    {
      node.setOutput("paused", this->_paused);
      return BT::NodeStatus::SUCCESS;
    }, { BT::OutputPort<bool>("paused") });

  // register PausableSequence
  this->_bt_factory.registerNodeType<common::PausableSequence>(
    "PausableSequence");

  this->_bt_factory.registerBuilder<SetResult>("SetResult",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetResult>(name, config, this->_ctx);
    });
  this->_bt_factory.registerBuilder<GetResult>("GetResult",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetResult>(name, config, this->_ctx,
      *this->_node.lock());
    });

  this->_bt_factory.registerNodeType<MakeTransform>("MakeTransform");
  this->_bt_factory.registerBuilder<ApplyPoseOffset>(
    "ApplyPoseOffset",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<ApplyPoseOffset>(name, config,
      *this->_node.lock(),
      this->_tf2_buffer);
    });

  this->_bt_factory.registerBuilder<SerializeDetections>(
    "SerializeDetections",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SerializeDetections>(name, config,
      *this->_node.lock());
    });
  this->_bt_factory.registerBuilder<DeserializeDetections>(
    "DeserializeDetections",
    [this](
      const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<DeserializeDetections>(name, config,
      *this->_node.lock());
    });

  this->_bt_factory.registerBuilder<WaitForSignal>("WaitForSignal",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<WaitForSignal>(name, config, this->_ctx);
    });

  this->_bt_factory.registerBuilder<SetSignal>("SetSignal",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetSignal>(name, config, this->_ctx);
    });

  this->_bt_factory.registerBuilder<GetJointConstraints>(
    "GetJointConstraints",
    [this](
      const std::string& name,
      const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetJointConstraints>(
        name,
        config,
        this->_node
      );
    });
}

void Job::load_capability(const Capability& cap)
{
  cap
}

}
