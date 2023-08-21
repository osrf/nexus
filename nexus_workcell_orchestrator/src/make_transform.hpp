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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__MAKE_TRANSFORM_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__MAKE_TRANSFORM_HPP

#include <behaviortree_cpp_v3/action_node.h>

namespace nexus::workcell_orchestrator {

/**
 * Creates a pose and write it to an output port
 *
 * Input Ports:
 *   x |double|
 *   y |double|
 *   z |double|
 *   qx |double|
 *   qy |double|
 *   qz |double|
 *   qw |double|
 * Output Ports:
 *   result |geometry_msgs::msg::Transform|
 */
class MakeTransform : public BT::SyncActionNode
{
public: inline MakeTransform(const std::string& name,
    const BT::NodeConfiguration& config)
  : BT::SyncActionNode{name, config} {}

public: static BT::PortsList providedPorts();

public: BT::NodeStatus tick() override;
};

}

#endif
