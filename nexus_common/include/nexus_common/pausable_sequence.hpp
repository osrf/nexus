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

#ifndef NEXUS_COMMON__PAUSABLE_SEQUENCE__HPP
#define NEXUS_COMMON__PAUSABLE_SEQUENCE__HPP

#include "nexus_common_export.hpp"

#include <behaviortree_cpp_v3/control_node.h>

namespace nexus::common {

/**
 * Ticks each children until pause is triggered. Unlike sequence,
 *
 * InputPorts:
 *   pause |bool| Indicates if execution should be paused.
 */
class NEXUS_COMMON_EXPORT PausableSequence : public BT::ControlNode
{
  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts();

  /**
   * @param name The name of the node.
   * @param config The BT config.
   */
public: inline PausableSequence(const std::string& name,
    const BT::NodeConfiguration& config)
  : BT::ControlNode{name, config} {}

public: BT::NodeStatus tick() override;

private: size_t _current_child_idx = 0;
};

}

#endif
