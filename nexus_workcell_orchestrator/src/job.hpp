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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__JOB_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__JOB_HPP

#include <nexus_capabilities/context.hpp>
#include <nexus_common/logging.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <filesystem>

namespace nexus::workcell_orchestrator {

struct Job
{
  using TaskState = nexus_orchestrator_msgs::msg::TaskState;
  using GoalHandlePtr =
    std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>;

  std::shared_ptr<Context> ctx;
  std::optional<BT::Tree> bt;
  GoalHandlePtr goal_handle;
  std::optional<common::BtLogging> bt_logging;
  TaskState task_state = TaskState();
  uint64_t tick_count = 0;
};

}

#endif
