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

#include "nexus_capabilities/context.hpp"

#include <exception>

namespace nexus {

common::Result<void> Context::publish_feedback(
  const nexus_orchestrator_msgs::msg::TaskProgress& progress,
  const std::string& msg)
{
  endpoints::WorkcellRequestAction::ActionType::Feedback::SharedPtr feedback_msg;
  feedback_msg->state.task_id = this->task.id;
  feedback_msg->state.workcell_id = this->node->get_name();
  feedback_msg->state.status =
    nexus_orchestrator_msgs::msg::TaskState::STATUS_RUNNING;
  feedback_msg->state.progress = progress;
  feedback_msg->state.message = msg;
  try
  {
    this->_goal_handle->publish_feedback(std::move(feedback_msg));
    return common::Result<void>();
  }
  catch (const std::runtime_error& e)
  {
    return e;
  }
}

}
