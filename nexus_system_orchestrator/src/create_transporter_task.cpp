/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "create_transporter_task.hpp"

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <nexus_transporter_msgs/msg/destination.hpp>

namespace nexus::system_orchestrator {

BT::NodeStatus CreateTransporterTask::tick()
{
  std::optional<TransportationRequest> result;
  this->setOutput("result", result);
  const auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  const auto workcell_task = this->getInput<WorkcellTask>("workcell_task");
  if (!workcell_task)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: [workcell_task] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto maybe_workcell_id =
    this->_ctx->get_workcell_task_assignment(workcell_task->task_id);
  if (!maybe_workcell_id.has_value())
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: no workcell found assigned to task [%s], this should not happen "
      "and is a bug.",
      this->name().c_str(),
      workcell_task->task_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (workcell_task->input_item_ids.empty())
  {
    // We actually don't need to transport anything here, just return success
    // and a nullopt transportation task
    return BT::NodeStatus::SUCCESS;
  }

  // TODO(ac): support tasks that require more than 1 inputs
  if (workcell_task->input_item_ids.size() > 1)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: only support creating transporter tasks for one input",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto input_item_id = workcell_task->input_item_ids[0];
  const auto designated_inputs =
    this->_ctx->get_workcell_task_inputs(workcell_task->task_id);
  if (designated_inputs.empty())
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: workcell task [%s] expects inputs, however none was provided.",
      this->name().c_str(),
      workcell_task->task_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::optional<std::string> input_station_id = std::nullopt;
  for (const auto& input : designated_inputs)
  {
    if (input.item_id == input_item_id)
    {
      input_station_id = input.station_id;
      break;
    }
  }
  if (!input_station_id.has_value())
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: workcell task [%s] expects input item [%s] to be at a station, "
      "however no station was provided.",
      this->name().c_str(),
      workcell_task->task_id.c_str(),
      input_item_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // TODO(luca) Implement a node that tracks the location of SKUs and query it
  // for the location, rather than using a context variable
  const auto sku_position = this->_ctx->get_sku_location(input_item_id);
  if (sku_position == std::nullopt)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(),
      "%s: cannot find location of SKU [%s], cannot create transporter task.",
      this->name().c_str(),
      input_item_id.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (sku_position.value() == *input_station_id)
  {
    RCLCPP_DEBUG(
      this->_ctx->get_node().get_logger(),
      "%s: input item [%s] is already at the target station [%s], no "
      "transporter task needed.",
      this->name().c_str(),
      input_item_id.c_str(),
      input_station_id->c_str());
    return BT::NodeStatus::SUCCESS;
  }

  result = TransportationRequest();
  result->id = this->_ctx->get_job_id();
  result->requester = node->get_name();
  // TODO(Yadunund): Parse work order and assign params.
  // See https://github.com/osrf/nexus/issues/68.
  result->destinations.emplace_back(
    nexus_transporter_msgs::build<nexus_transporter_msgs::msg::Destination>()
      .name(sku_position.value())
      .action(nexus_transporter_msgs::msg::Destination::ACTION_PICKUP)
      .params("")
  );
  result->destinations.emplace_back(
    nexus_transporter_msgs::build<nexus_transporter_msgs::msg::Destination>()
      .name(*input_station_id)
      .action(nexus_transporter_msgs::msg::Destination::ACTION_DROPOFF)
      .params("")
  );

  this->setOutput("result", result);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UnpackTransporterTask::tick()
{
  const auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  const auto maybe_request =
    this->getInput<std::optional<TransportationRequest>>("input");
  if (!maybe_request)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s: [input] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!maybe_request.value())
  {
    return BT::NodeStatus::FAILURE;
  }
  this->setOutput("output", *maybe_request.value());
  return BT::NodeStatus::SUCCESS;
}

}
