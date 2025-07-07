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

#include "send_signal.hpp"

#include <nexus_endpoints.hpp>

namespace nexus::system_orchestrator {

BT::NodeStatus SendSignal::tick()
{
  auto task =
    this->getInput<nexus_orchestrator_msgs::msg::WorkcellTask>("task");
  if (!task)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [task] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    const auto assigned_workcell_id =
      this->_ctx->get_workcell_task_assignment(task->task_id);
    if (!assigned_workcell_id.has_value())
    {
      RCLCPP_ERROR(
        this->_ctx->get_node().get_logger(), "%s: Unable to find workcell assigned to task [%s]",
        this->name().c_str(), task->task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const auto session = this->_ctx->get_workcell_session(*assigned_workcell_id);
    if (!session)
    {
      RCLCPP_ERROR(this->_ctx->get_node().get_logger(),
        "%s: Unable to find workcell session for this workcell",
        this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto req =
      std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
    req->task_id = task->task_id;
    req->signal = *signal;
    auto resp = session->signal_wc_client->send_request(req);
    RCLCPP_INFO(
      this->_ctx->get_node().get_logger(), "%s: Sent signal [%s] to workcell [%s]",
      this->name().c_str(), signal->c_str(), (*assigned_workcell_id).c_str());
    if (!resp->success)
    {
      RCLCPP_WARN(
        this->_ctx->get_node().get_logger(),
        "%s: Workcell is not able to accept [%s] signal now. Queuing the signal to be sent on the next task request.",
        this->name().c_str(), signal->c_str());
      this->_ctx->add_queued_signal(task->task_id, *signal);
    }
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_ERROR(this->_ctx->get_node().get_logger(), "%s: %s",
      this->name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SignalTransporter::tick()
{
  auto transporter =
    this->getInput<std::string>("transporter");
  if (!transporter)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [transporter] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto transporter_task_id =
    this->getInput<std::string>("transporter_task_id");
  if (!transporter_task_id)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [transporter_task_id] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    const auto session = this->_ctx->get_transporter_session(*transporter);
    if (!session)
    {
      RCLCPP_ERROR(this->_ctx->get_node().get_logger(),
        "%s: Unable to find transporter session for this transporter",
        this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    // TODO(luca) change this to SignalTransporterService
    auto req =
      std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
    req->task_id = *transporter_task_id;
    req->signal = *signal;
    auto resp = session->signal_transporter_client->send_request(req);
    RCLCPP_INFO(
      this->_ctx->get_node().get_logger(), "%s: Sent signal [%s] to transporter [%s]",
      this->name().c_str(), signal->c_str(), transporter->c_str());
    if (!resp->success)
    {
      RCLCPP_WARN(
        this->_ctx->get_node().get_logger(),
        "%s: Transporter [%s] is not able to accept [%s].",
        this->name().c_str(), transporter->c_str(), signal->c_str());
      // Return failure?
    }
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_ERROR(this->_ctx->get_node().get_logger(), "%s: %s",
      this->name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}
