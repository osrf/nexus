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

#include "for_each_task_pair.hpp"

#include <rclcpp/rclcpp.hpp>

namespace nexus::system_orchestrator {

using Task = nexus_orchestrator_msgs::msg::WorkcellTask;

BT::PortsList ForEachTaskPair::providedPorts()
{
  return {
    BT::OutputPort<Task>("place_task"),
    BT::OutputPort<Task>("pick_task"),
    BT::OutputPort<std::string>("source"),
    BT::OutputPort<std::string>("destination")
  };
}

BT::NodeStatus ForEachTaskPair::tick()
{
  if (this->_first_tick)
  {
    this->_first_tick = false;
    RCLCPP_DEBUG(this->_logger, "Looping through %lu tasks",
      this->_ctx->tasks.size());
  }
  if (this->_ctx->tasks.size() == 0)
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (this->_ctx->tasks.size() < 2)
  {
    RCLCPP_ERROR(this->_logger, "ForEachTaskPair node requires at least 2 nodes");
    return BT::NodeStatus::FAILURE;
  }

  this->setStatus(BT::NodeStatus::RUNNING);

  // Iterate pairwise, a transporter will be requested to go from source to destination
  while (this->_current_idx < this->_ctx->tasks.size() - 1)
  {
    auto place_task = this->_ctx->tasks.at(this->_current_idx);
    auto pick_task = this->_ctx->tasks.at(this->_current_idx + 1);
    this->setOutput("place_task", place_task);
    this->setOutput("pick_task", pick_task);
    try
    {
      this->setOutput("source",
        this->_ctx->workcell_task_assignments.at(place_task.id));
    }
    catch (const std::out_of_range&)
    {
      RCLCPP_ERROR(this->_logger, "task [%s] not assigned to any workcell",
        place_task.id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    try
    {
      this->setOutput("destination",
        this->_ctx->workcell_task_assignments.at(pick_task.id));
    }
    catch (const std::out_of_range&)
    {
      RCLCPP_ERROR(this->_logger, "task [%s] not assigned to any workcell",
        pick_task.id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    this->setStatus(BT::NodeStatus::RUNNING);
    auto child_state = this->child_node_->executeTick();
    switch (child_state)
    {
      case BT::NodeStatus::SUCCESS:
        this->haltChild();
        this->_current_idx += 2;
        break;
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::FAILURE:
        this->haltChild();
        this->_current_idx = 0;
        return BT::NodeStatus::FAILURE;
      case BT::NodeStatus::IDLE:
        this->_current_idx = 0;
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}
