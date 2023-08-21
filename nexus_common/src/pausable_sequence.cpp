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

#include "pausable_sequence.hpp"

#include <iostream>

namespace nexus::common {

BT::PortsList PausableSequence::providedPorts()
{
  return { BT::InputPort<bool>("pause") };
}

BT::NodeStatus PausableSequence::tick()
{
  const size_t children_count = this->children_nodes_.size();

  this->setStatus(BT::NodeStatus::RUNNING);

  auto paused = this->getInput<bool>("pause");
  if (!paused.has_value())
  {
    // ideally we should use RCLCPP_ERROR but we don't have access to a ros node.
    std::cerr << this->name() << ": port [pause] is required" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (*paused)
  {
    return BT::NodeStatus::RUNNING;
  }

  TreeNode* current_child_node =
    this->children_nodes_[this->_current_child_idx];
  const BT::NodeStatus child_status = current_child_node->executeTick();

  switch (child_status)
  {
    case BT::NodeStatus::RUNNING:
    {
      return child_status;
    }
    case BT::NodeStatus::FAILURE:
    {
      // Reset on failure
      haltChildren();
      this->_current_child_idx = 0;
      return child_status;
    }
    case BT::NodeStatus::SUCCESS:
    {
      this->_current_child_idx++;
      if (this->_current_child_idx >= children_count)
      {
        haltChildren();
        this->_current_child_idx = 0;
        return BT::NodeStatus::SUCCESS;
      }
      // unlike Sequence, we need to return RUNNING to give other nodes a chance
      // to update the pause status.
      return BT::NodeStatus::RUNNING;
    }
    break;
    case BT::NodeStatus::IDLE:
    {
      throw BT::LogicError("A child node must never return IDLE");
    }
  }

  // should not ever reach here.
  return BT::NodeStatus::FAILURE;
}

}
