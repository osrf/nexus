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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__SESSION_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__SESSION_HPP

#include <nexus_common/sync_service_client.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nexus_orchestrator_msgs/msg/workcell_description.hpp>

namespace nexus::system_orchestrator {

/**
 * NOTE: This is shared across jobs, DO NOT put per job data here, put it in context instead!
 */
struct WorkcellSession
{
  nexus_orchestrator_msgs::msg::WorkcellDescription description;
  endpoints::WorkcellStateTopic::MessageType state;
  rclcpp::Client<endpoints::IsTaskDoableService::ServiceType>::SharedPtr
    task_doable_client;
  rclcpp::Client<endpoints::PauseWorkcellService::ServiceType>::SharedPtr
    pause_client;
  std::unique_ptr<common::SyncServiceClient<endpoints::SignalWorkcellService::ServiceType>>
  signal_wc_client;
  std::unique_ptr<common::SyncServiceClient<endpoints::QueueWorkcellTaskService::ServiceType>>
  queue_task_client;
  std::unique_ptr<common::SyncServiceClient<endpoints::RemovePendingTaskService::ServiceType>>
  remove_pending_task_client;
};

struct TransporterSession
{
  nexus_orchestrator_msgs::msg::WorkcellDescription description;
  rclcpp::Client<endpoints::IsTransporterAvailableService::ServiceType>::
  SharedPtr available_client;
  std::unique_ptr<common::SyncServiceClient<endpoints::SignalTransporterService::ServiceType>>
  signal_client;
};

}

#endif
