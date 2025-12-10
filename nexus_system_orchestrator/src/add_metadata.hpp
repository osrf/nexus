/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__ADD_METADATA__HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__ADD_METADATA__HPP

#include "context.hpp"

#include <behaviortree_cpp_v3/action_node.h>

namespace nexus::system_orchestrator {

/**
 * Adds metadata to all assigned tasks.
 */
class AddMetadata : public BT::SyncActionNode
{

public: AddMetadata(const std::string& name,
    const BT::NodeConfiguration& config, std::shared_ptr<Context> ctx)
  : BT::SyncActionNode(name, config), _ctx(std::move(ctx)) {}

public: BT::NodeStatus tick() override;

private: std::shared_ptr<Context> _ctx;
};

}  // namespace nexus::system_orchestrator

#endif  // NEXUS_SYSTEM_ORCHESTRATOR__ADD_METADATA__HPP
