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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__EXECUTE_TASK_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__EXECUTE_TASK_HPP

#include "context.hpp"

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <filesystem>

namespace nexus::system_orchestrator {

/**
 * Searches for a behavior tree to execute a task and run it as a sub tree.
 *
 * The behavior tree used is based on the task type after remapping and the
 * filename of the behavior tree without extension.
 *
 * Input Ports:
 *   task |nexus_orchestrator_msgs::msg::WorkcellTask| The task to execute.
 *   workcell |std::string| Workcell to execute on.
 */
class ExecuteTask : public BT::StatefulActionNode
{
public: using Task = nexus_orchestrator_msgs::msg::WorkcellTask;

public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<Task>("task"),
      BT::InputPort<std::string>("workcell"),
      BT::InputPort<Task>("transport_task") };
  }

public: ExecuteTask(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<Context> ctx,
    std::filesystem::path bt_path,
    std::shared_ptr<BT::BehaviorTreeFactory> bt_factory)
  : BT::StatefulActionNode(name, config), _ctx(std::move(ctx)), _bt_path(std::move(
        bt_path)),
    _bt_factory(std::move(bt_factory)) {}

protected: BT::NodeStatus onStart() override;

protected: BT::NodeStatus onRunning() override;

protected: void onHalted() override;

private: std::shared_ptr<Context> _ctx;
private: std::filesystem::path _bt_path;
private: std::shared_ptr<BT::BehaviorTreeFactory> _bt_factory;
private: std::unique_ptr<BT::Tree> _bt;
};

}

#endif
