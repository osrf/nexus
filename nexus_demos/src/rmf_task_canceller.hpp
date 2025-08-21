// Copyright 2025 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEXUS_DEMOS__SRC__RMF_TASK_CANCELLER_HPP
#define NEXUS_DEMOS__SRC__RMF_TASK_CANCELLER_HPP

#include <nexus_orchestrator_msgs/msg/workcell_state.hpp>
#include <nexus_orchestrator_msgs/msg/work_order_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <unordered_map>

namespace nexus_demos {
using RMFApiRequest = rmf_task_msgs::msg::ApiRequest;
using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;
using WorkcellState = nexus_orchestrator_msgs::msg::WorkcellState;
using WorkOrderState = nexus_orchestrator_msgs::msg::WorkOrderState;

//==============================================================================
class RMFTaskCanceller : public rclcpp::Node
{
public:
  RMFTaskCanceller(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::shared_ptr<rclcpp::Publisher<RMFApiRequest>> rmf_api_request_pub_;
  std::shared_ptr<rclcpp::Publisher<DispenserResult>> dispenser_result_pub_;
  std::shared_ptr<rclcpp::Subscription<DispenserRequest>> dispenser_request_sub_;
  std::shared_ptr<rclcpp::Publisher<IngestorResult>> ingestor_result_pub_;
  std::shared_ptr<rclcpp::Subscription<IngestorRequest>> ingestor_request_sub_;
  std::shared_ptr<rclcpp::Subscription<WorkcellState>> rmf_state_sub_;
  std::shared_ptr<rclcpp::Subscription<WorkOrderState>> wo_state_sub_;

  // Map the work_order_id to the WorkOrderState msg.
  std::unordered_map<std::string, WorkOrderState::ConstSharedPtr> cancelled_wo_;
  // The state of the RMF workcell.
  WorkcellState::ConstSharedPtr rmf_workcell_state_;
};
}  // namespace nexus_demos

#endif  // NEXUS_DEMOS__SRC__RMF_TASK_CANCELLER_HPP
