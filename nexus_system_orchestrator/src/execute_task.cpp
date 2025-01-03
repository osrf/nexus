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

#include "execute_task.hpp"

namespace nexus::system_orchestrator {

BT::NodeStatus ExecuteTask::onStart()
{
  auto task = this->getInput<Task>("task");
  if (!task)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [task] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto workcell = this->getInput<std::string>("workcell");
  if (!workcell)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [workcell] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Remap the BT filename to load if one is provided.
  std::string bt_name = task->type;
  auto it = _ctx->task_remaps->find(task->type);
  if (it != _ctx->task_remaps->end())
  {
    RCLCPP_DEBUG(
      _ctx->node.get_logger(),
      "[ExecuteTask] Loading remapped BT [%s] for original task type [%s]",
      it->second.c_str(),
      task->type.c_str()
    );
    bt_name = it->second;
  }
  std::filesystem::path task_bt_path(this->_bt_path / (bt_name + ".xml"));
  if (!std::filesystem::is_regular_file(task_bt_path))
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: no behavior tree to execute task type [%s]",
      this->name().c_str(), task->type.c_str());
    return BT::NodeStatus::FAILURE;
  }

  this->_bt = std::make_unique<BT::Tree>(this->_bt_factory->createTreeFromFile(
        task_bt_path));
  this->_bt->rootBlackboard()->set("task", *task);
  this->_bt->rootBlackboard()->set("workcell", *workcell);
  auto transport_task = this->getInput<Task>("transport_task");
  if (transport_task)
  {
    this->_bt->rootBlackboard()->set("transport_task", *transport_task);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteTask::onRunning()
{
  return this->_bt->tickRoot();
}

void ExecuteTask::onHalted()
{
  this->_bt->haltTree();
}

}
