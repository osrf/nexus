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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__SEND_SIGNAL_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__SEND_SIGNAL_HPP

#include "context.hpp"

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <behaviortree_cpp_v3/action_node.h>

namespace nexus::system_orchestrator {

/**
 * Sends a signal to a workcell.
 *
 * Input Ports:
 *   task |nexus_orchestrator_msgs::msg::WorkcellTask| The task the signal is tied to.
 *   signal |std::string| Signal to send.
 */
class SendSignal : public BT::SyncActionNode
{
public: using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<nexus_orchestrator_msgs::msg::WorkcellTask>("task",
        "The task the signal is tied to."),
      BT::InputPort<std::string>("transporter", "The transporter to signal"),
      BT::InputPort<std::string>(
        "signal", "Signal to send.") };
  }

public: SendSignal(const std::string& name, const BT::NodeConfiguration& config,
    std::shared_ptr<Context> ctx)
  : BT::SyncActionNode(name, config), _ctx(std::move(ctx)) {}

public: BT::NodeStatus tick() override;

private: BT::NodeStatus signal_task(const WorkcellTask& task, const std::string& signal);
private: BT::NodeStatus signal_transporter(const std::string& transporter, const std::string& signal);

private: std::shared_ptr<Context> _ctx;
};

}

#endif
