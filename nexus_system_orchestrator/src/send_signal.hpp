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
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<nexus_orchestrator_msgs::msg::WorkcellTask>("task",
        "The task the signal is tied to."),
      BT::InputPort<std::string>(
        "signal", "Signal to send.") };
  }

public: SendSignal(const std::string& name, const BT::NodeConfiguration& config,
    std::shared_ptr<Context> ctx)
  : BT::SyncActionNode(name, config), _ctx(std::move(ctx)) {}

public: BT::NodeStatus tick() override;

private: std::shared_ptr<Context> _ctx;
};

/**
 * Sends a signal to a transporter.
 *
 * Input Ports:
 *   transporter |std::string| The transporter the signal is tied to,
 *   transporter_task_id |std::string| The transporter task id the signal is tied to,
 *   workcell |std::string| The workcell sending the signal,
 *   signal |std::string| Signal to send.
 */
class SignalTransporter : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("transporter",
        "The transporter the signal is tied to."),
      BT::InputPort<std::string>("transporter_task_id",
        "The transporter task id the signal is tied to."),
      BT::InputPort<std::string>(
        "signal", "Signal to send.") };
  }

public: SignalTransporter(const std::string& name, const BT::NodeConfiguration& config,
    std::shared_ptr<Context> ctx)
  : BT::SyncActionNode(name, config), _ctx(std::move(ctx)) {}

public: BT::NodeStatus tick() override;

private: std::shared_ptr<Context> _ctx;
};

}

#endif
