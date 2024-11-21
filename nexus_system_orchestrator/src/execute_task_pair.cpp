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

#include "execute_task_pair.hpp"

namespace nexus::system_orchestrator {

BT::NodeStatus ExecuteTaskPair::onStart()
{
  auto place_task = this->getInput<Task>("place_task");
  if (!place_task)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [place_task] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto pick_task = this->getInput<Task>("pick_task");
  if (!pick_task)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [pick_task] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto source = this->getInput<std::string>("source");
  if (!source)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [source] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto destination = this->getInput<std::string>("destination");
  if (!destination)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [destination] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Remap the BT filename to load if one is provided.
  // TODO(luca) look into task remapping
  std::string bt_name = place_task->type;
  auto it = _ctx->task_remaps->find(place_task->type);
  if (it != _ctx->task_remaps->end())
  {
    RCLCPP_DEBUG(
      _ctx->node.get_logger(),
      "[ExecuteTaskPair] Loading remapped BT [%s] for original task type [%s]",
      it->second.c_str(),
      place_task->type.c_str()
    );
    bt_name = it->second;
  }
  std::filesystem::path task_bt_path(this->_bt_path / (bt_name + ".xml"));
  if (!std::filesystem::is_regular_file(task_bt_path))
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(),
      "%s: no behavior tree to execute task type [%s]",
      this->name().c_str(), place_task->type.c_str());
    return BT::NodeStatus::FAILURE;
  }

  this->_bt = std::make_unique<BT::Tree>(this->_bt_factory->createTreeFromFile(
        task_bt_path));
  this->_bt->rootBlackboard()->set("place_task", *place_task);
  this->_bt->rootBlackboard()->set("pick_task", *pick_task);
  this->_bt->rootBlackboard()->set("source", *source);
  this->_bt->rootBlackboard()->set("destination", *destination);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteTaskPair::onRunning()
{
  return this->_bt->tickRoot();
}

void ExecuteTaskPair::onHalted()
{
  this->_bt->haltTree();
}

}
