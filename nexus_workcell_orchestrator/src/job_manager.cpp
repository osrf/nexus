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

#include "job_manager.hpp"

namespace nexus::workcell_orchestrator {

using TaskState = nexus_orchestrator_msgs::msg::TaskState;
using WorkcellRequest = endpoints::WorkcellRequestAction::ActionType;

/**
 * Tick the behavior tree of a job and update the task status according to the result.
 */
void JobManager::_tick_bt(Job& job)
{
  this->_ctx_mgr->set_active_context(job.ctx);
  const auto bt_status = job.bt->tickRoot();
  ++job.tick_count;
  this->_ctx_mgr->set_active_context(nullptr);

  // update task status according to bt status
  switch (bt_status)
  {
    case BT::NodeStatus::RUNNING: {
      job.task_state.status = TaskState::STATUS_RUNNING;
      break;
    }
    case BT::NodeStatus::FAILURE: {
      RCLCPP_INFO(
        job.ctx->node->get_logger(), "Task [%s] failed",
        job.ctx->task.id.c_str());
      // Publish feedback.
      job.task_state.status = TaskState::STATUS_FAILED;
      auto fb = std::make_shared<WorkcellRequest::Feedback>();
      fb->state = job.task_state;
      job.goal_handle->publish_feedback(std::move(fb));

      // Abort the action request.
      auto result =
        std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
      result->success = false;
      result->message = "Task failed";
      job.goal_handle->abort(result);
      break;
    }
    case BT::NodeStatus::SUCCESS: {
      // Publish feedback.
      job.task_state.status = TaskState::STATUS_FINISHED;
      auto fb = std::make_shared<WorkcellRequest::Feedback>();
      fb->state = job.task_state;
      job.goal_handle->publish_feedback(std::move(fb));

      auto result =
        std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
      result->success = true;
      result->result = YAML::Dump(job.ctx->task.previous_results);
      auto report = job.bt_logging->generate_report();
      result->message = common::ReportConverter::to_string(report);
      RCLCPP_INFO(job.ctx->node->get_logger(), "%s", result->message.c_str());
      job.goal_handle->succeed(result);
      break;
    }
    default: {
      std::ostringstream oss;
      oss << "Error ticking task [" << job.ctx->task.id <<
        "]: Behavior tree returned invalid or unknown status [" <<
        bt_status << "]";
      RCLCPP_ERROR_STREAM(job.ctx->node->get_logger(), oss.str());
      throw std::runtime_error(oss.str());
    }
  }
}

uint8_t JobManager::_tick_job(Job& job)
{
  auto& task_status = job.task_state.status;
  switch (task_status)
  {
    case TaskState::STATUS_ASSIGNED:
      break;
    case TaskState::STATUS_QUEUED:
    case TaskState::STATUS_RUNNING: {
      this->_tick_bt(job);
      break;
    }
    case TaskState::STATUS_FAILED: {
      RCLCPP_WARN(
        job.ctx->node->get_logger(), "Cannot tick task [%s] that has already failed",
        job.ctx->task.id.c_str());
      break;
    }
    case TaskState::STATUS_FINISHED: {
      RCLCPP_WARN(
        job.ctx->node->get_logger(), "Cannot tick task [%s] that has already finished",
        job.ctx->task.id.c_str());
      break;
    }
    default: {
      RCLCPP_ERROR(
        job.ctx->node->get_logger(), "Error ticking task [%s]: Unknown status [%i]",
        job.ctx->task.id.c_str(),
        task_status);
      break;
    }
  }
  return task_status;
}

Job& JobManager::assign_task(
  const std::string& task_id)
{
  const auto it =
    std::find_if(this->_jobs.begin(), this->_jobs.end(), [&task_id](
        const Job& j)
      {
        return j.task_state.task_id == task_id;
      });
  if (it != this->_jobs.end())
  {
    throw JobError("Another task with the same id already exist");
  }

  auto& j = this->_jobs.emplace_back(Job{nullptr, std::nullopt, nullptr,
        nullptr});
  j.task_state.task_id = task_id;
  j.task_state.workcell_id = this->_node->get_name();
  j.task_state.status = TaskState::STATUS_ASSIGNED;
  RCLCPP_INFO(
    this->_node->get_logger(),
    "Task [%s] is pending, call \"%s\" to start this task",
    j.task_state.task_id.c_str(),
    endpoints::WorkcellRequestAction::action_name(
      this->_node->get_name()).c_str());

  RCLCPP_INFO(
    this->_node->get_logger(), "Assigned task [%s]",
    task_id.c_str());
  return j;
}

Job& JobManager::queue_task(const GoalHandlePtr& goal_handle,
  const std::shared_ptr<Context>& ctx, BT::Tree&& bt)
{
  const auto& job_id = goal_handle->get_goal()->task.id;
  const auto it =
    std::find_if(this->_jobs.begin(), this->_jobs.end(), [&job_id](const Job& j)
      {
        return j.task_state.task_id == job_id;
      });
  if (it == this->_jobs.end())
  {
    // abort the goal and throw error
    const auto result = std::make_shared<WorkcellRequest::Result>();
    result->success = false;
    result->message = "Task not found";
    goal_handle->abort(result);
    throw JobError("Task not found");
  }

  it->ctx = ctx;
  it->bt = std::move(bt);
  it->bt_logging =
    std::make_unique<common::BtLogging>(*it->bt, this->_node);
  it->goal_handle = goal_handle;
  it->task_state.status = TaskState::STATUS_QUEUED;
  RCLCPP_INFO(
    it->ctx->node->get_logger(), "Task [%s] is now queued",
    it->task_state.task_id.c_str());
  return *it;
}

void JobManager::remove_assigned_task(const std::string& task_id)
{
  const auto it = std::find_if(this->_jobs.begin(),
      this->_jobs.end(), [&task_id](const Job& j)
      {
        return j.ctx->task.id == task_id;
      });
  if (it == this->_jobs.end())
  {
    return;
  }
  if (it->task_state.status != TaskState::STATUS_ASSIGNED)
  {
    throw JobError("Task is not assigned");
  }
  this->_jobs.erase(it);
}

void JobManager::halt_all_jobs()
{
  RCLCPP_INFO(this->_node->get_logger(), "Halting all tasks");
  for (auto& j : this->_jobs)
  {
    // halt bts that are running
    if (j.task_state.status == TaskState::STATUS_RUNNING)
    {
      this->_ctx_mgr->set_active_context(j.ctx);
      j.bt->haltTree();
      this->_ctx_mgr->set_active_context(nullptr);

      RCLCPP_INFO(
        j.ctx->node->get_logger(), "Task [%s] halted", j.ctx->task.id.c_str());
      // Publish feedback.
      j.task_state.status = TaskState::STATUS_FAILED;
      auto fb = std::make_shared<WorkcellRequest::Feedback>();
      fb->state = j.task_state;
      j.goal_handle->publish_feedback(std::move(fb));

      // Abort the action request.
      auto result =
        std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
      result->success = false;
      result->message = "Task halted";
      j.goal_handle->abort(result);
      break;
    }
  }

  this->_jobs.clear();
}

void JobManager::tick()
{
  size_t i = 0;
  auto it = this->_jobs.begin();
  for (; i < this->_max_concurrent && it != this->_jobs.end(); ++i)
  {
    const auto task_status = this->_tick_job(*it);
    if (task_status == TaskState::STATUS_FAILED ||
      task_status == TaskState::STATUS_FINISHED)
    {
      it = this->_jobs.erase(it);
      continue;
    }
    else
    {
      ++it;
    }
  }
}

}
