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


#ifndef NEXUS_WORKCELL_ORCHESTRATOR__GET_RESULT_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__GET_RESULT_HPP

#include <nexus_capabilities/context_manager.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::workcell_orchestrator {

/**
 * Set a ROS message as the result of a task. Fails if no such key exists.
 * Input Ports:
 *   key |std::string|
 * Output Ports:
 *   result |std::string|
 */
class GetResult : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("key"), BT::OutputPort<std::string>(
        "result") };
  }

public: GetResult(const std::string& name, const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr)
  : BT::SyncActionNode{name, config}, _ctx_mgr{ctx_mgr} {}

public: BT::NodeStatus tick() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

}

#endif
