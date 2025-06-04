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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__SIGNALS_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__SIGNALS_HPP

#include <nexus_capabilities/context_manager.hpp>

#include <behaviortree_cpp_v3/action_node.h>

namespace nexus::workcell_orchestrator {

/**
 * Returns RUNNING until a signal is received from system orchestrator.
 *
 * Input Ports:
 *   signal |std::string| Signal to wait for.
 *   clear |bool| Set this to true to clear the signal when this node is finished.
 */
class WaitForSignal : public BT::StatefulActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("signal", "Signal to wait for."),
      BT::InputPort<std::string>("clear",
        "Set this to true to clear the signal when this node is finished.") };
  }

public: WaitForSignal(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr)
  : BT::StatefulActionNode(name, config), _ctx_mgr(std::move(ctx_mgr)) {}

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override {};

private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

/**
 * Set a signal.
 */
class SetSignal : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("signal", "Signal to set.") };
  }

public: SetSignal(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr)
  : BT::SyncActionNode(name, config), _ctx_mgr(std::move(ctx_mgr)) {}

protected: BT::NodeStatus tick() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
};


}

#endif
