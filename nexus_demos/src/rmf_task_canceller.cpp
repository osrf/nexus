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

#include "rmf_task_canceller.hpp"

#include <nexus_orchestrator_msgs/msg/task_state.hpp>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

namespace nexus_demos {

//==============================================================================
RMFTaskCanceller::RMFTaskCanceller(const rclcpp::NodeOptions & options)
: Node("rmf_workcell_task_canceller"),
  rmf_workcell_state_(nullptr)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Starting rmf_task_canceller..."
  );

  const std::string rmf_workcell_name =
    this->declare_parameter("rmf_workcell_name", "rmf_nexus_transporter");
  const auto reliable_qos =
    rclcpp::SystemDefaultsQoS().keep_last(10).reliable();
  const auto transient_qos =
    rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();

  rmf_api_request_pub_ = this->create_publisher<RMFApiRequest>(
    "/task_api_requests",
    transient_qos
  );

  dispenser_result_pub_ = this->create_publisher<DispenserResult>(
    "/dispenser_results",
    reliable_qos
  );

  dispenser_request_sub_ = this->create_subscription<DispenserRequest>(
    "/dispenser_requests",
    reliable_qos,
    [this](DispenserRequest::ConstSharedPtr msg)
    {
      if (rmf_workcell_state_ == nullptr)
      {
        return;
      }

      if (msg->request_guid != rmf_workcell_state_->task_id)
      {
        return;
      }

      auto wo_it = cancelled_wo_.find(rmf_workcell_state_->work_order_id);
      if (wo_it == cancelled_wo_.end())
      {
        return;
      }

      DispenserResult dispenser_result_msg;
      dispenser_result_msg.request_guid = msg->request_guid;
      dispenser_result_msg.source_guid = msg->target_guid;
      dispenser_result_msg.status = DispenserResult::FAILED;
      dispenser_result_pub_->publish(std::move(dispenser_result_msg));

      // Also cancel the RMF task.
      nlohmann::json cancel_json;
      cancel_json["type"] = "cancel_task_request";
      cancel_json["task_id"] = msg->request_guid;
      RMFApiRequest cancel_request;
      // Time since epoch as a unique id.
      auto now = std::chrono::steady_clock::now().time_since_epoch();
      cancel_request.request_id = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
      cancel_request.json_msg = cancel_json.dump();
      this->rmf_api_request_pub_->publish(std::move(cancel_request));
    }
  );

  rmf_state_sub_ = this->create_subscription<WorkcellState>(
    "/" + rmf_workcell_name + "/workcell_state",
    reliable_qos,
    [this](WorkcellState::ConstSharedPtr msg)
    {
      if (msg->work_order_id.empty())
      {
        return;
      }
      this->rmf_workcell_state_ = msg;
    }
  );

  wo_state_sub_ = this->create_subscription<WorkOrderState>(
    "/work_order_states",
    reliable_qos,
    [this](WorkOrderState::ConstSharedPtr msg)
    {
      if (msg->state == WorkOrderState::STATE_CANCELLED ||
        msg->state == WorkOrderState::STATE_FAILED)
      {
        cancelled_wo_[msg->id] = msg;
      }
    }
  );
}
}  // namespace nexus_demos

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus_demos::RMFTaskCanceller)
