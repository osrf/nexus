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

#include "for_each_task.hpp"

#include <rclcpp/rclcpp.hpp>

namespace nexus::system_orchestrator {

using Task = nexus_orchestrator_msgs::msg::WorkcellTask;

BT::PortsList ForEachTask::providedPorts()
{
  return { BT::OutputPort<Task>("task"), BT::OutputPort<std::string>(
      "workcell") };
}

BT::NodeStatus ForEachTask::tick()
{
  if (this->_first_tick)
  {
    this->_first_tick = false;
    RCLCPP_DEBUG(this->_logger, "Looping through %lu tasks",
      this->_ctx->get_task_number());
  }
  if (this->_ctx->get_task_number() == 0)
  {
    return BT::NodeStatus::SUCCESS;
  }

  this->setStatus(BT::NodeStatus::RUNNING);

  const auto tasks = this->_ctx->get_tasks();
  while (this->_current_idx < tasks.size())
  {
    auto current_task = tasks.at(this->_current_idx);
    this->setOutput("task", current_task);
    try
    {
      this->setOutput("workcell",
        this->_ctx->get_workcell_task_assignments().at(current_task.task_id));
    }
    catch (const std::out_of_range&)
    {
      RCLCPP_ERROR(this->_logger, "task [%s] not assigned to any workcell",
        current_task.task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    this->setStatus(BT::NodeStatus::RUNNING);
    auto child_state = this->child_node_->executeTick();
    switch (child_state)
    {
      case BT::NodeStatus::SUCCESS:
        this->haltChild();
        ++this->_current_idx;
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
