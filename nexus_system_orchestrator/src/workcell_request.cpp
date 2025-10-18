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

#include "workcell_request.hpp"

#include <nexus_orchestrator_msgs/msg/task_progress.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

namespace nexus::system_orchestrator {

using TaskProgress = nexus_orchestrator_msgs::msg::TaskProgress;
using TaskState = nexus_orchestrator_msgs::msg::TaskState;

BT::PortsList WorkcellRequest::providedPorts()
{
  return {
    BT::InputPort<Task>("task", "The task to execute"),
    BT::InputPort<std::string>("workcell", "Workcell to use")
  };
}

BT::NodeStatus WorkcellRequest::onStart()
{
  auto maybe_task = this->getInput<Task>("task");
  if (!maybe_task)
  {
    RCLCPP_ERROR(this->_node->get_logger(), "[task] param is required");
    return BT::NodeStatus::FAILURE;
  }
  this->_task = maybe_task.value();

  auto maybe_workcell = this->getInput<std::string>("workcell");
  if (!maybe_workcell)
  {
    RCLCPP_ERROR(this->_node->get_logger(), "%s: [workcell] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_workcell_id = maybe_workcell.value();

  this->_session = this->_ctx->get_workcell_session(this->_workcell_id);
  if (!this->_session)
  {
    RCLCPP_ERROR(this->_node->get_logger(), "%s: workcell [%s] not found",
      this->name().c_str(),
      this->_workcell_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
      endpoints::WorkcellRequestAction::ActionType>::onStart();
}

std::string WorkcellRequest::get_action_name() const
{
  return endpoints::WorkcellRequestAction::action_name(this->_workcell_id);
}

std::optional<endpoints::WorkcellRequestAction::ActionType::Goal>
WorkcellRequest::make_goal()
{
  endpoints::WorkcellRequestAction::ActionType::Goal goal;
  goal.task = this->_task;
  const auto signals = this->_ctx->get_task_queued_signals(this->_task.task_id);
  if (!signals.has_value())
  {
    RCLCPP_DEBUG(
      this->_ctx->get_node().get_logger(), "%s: No queued signals for task [%s]",
      this->name().c_str(), this->_task.task_id.c_str());
    // ignore
  }
  goal.start_signals = *signals;
  this->_ctx->set_task_queued_signals(this->_task.task_id, {});

  goal.task.previous_results = this->_ctx->get_task_results();
  RCLCPP_INFO_STREAM(
    this->_ctx->get_node().get_logger(), "%s: Sending workcell request:" << std::endl << nexus_orchestrator_msgs::action::to_yaml(
        goal));
  return goal;
}

void WorkcellRequest::on_feedback(
  endpoints::WorkcellRequestAction::ActionType::Feedback::ConstSharedPtr msg)
{
  this->_ctx->set_task_state(this->_task.task_id, msg->state);
  this->_on_task_progress(this->_ctx->get_task_states());
}

bool WorkcellRequest::on_result(
  const rclcpp_action::ClientGoalHandle<endpoints::WorkcellRequestAction::ActionType>::WrappedResult& result)
{
  if (!result.result->success)
  {
    this->_ctx->add_error(result.result->message);
    return false;
  }

  auto task = this->getInput<Task>("task");
  if (!task)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [task] is required",
      this->name().c_str());
    return false;
  }

  this->_ctx->set_task_results(result.result->result); // -.-
  // TODO(luca) When output stations are available, set the location of the SKU
  // to the output station here and the workcell itself when WorkcellRequest is called.
  if (!this->_task.output_item_to_station_map.empty())
  {
    this->_ctx->set_sku_location(this->_task);
  }

  const auto task_state = this->_ctx->get_task_state(this->_task.task_id);
  if (!task_state.has_value())
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: task state not available",
      this->name().c_str());
    return false;
  }

  auto updated_state = *task_state;
  updated_state.status = TaskState::STATUS_FINISHED;
  this->_ctx->set_task_state(this->_task.task_id, updated_state);
  this->_on_task_progress(this->_ctx->get_task_states());
  return true;
}

}
