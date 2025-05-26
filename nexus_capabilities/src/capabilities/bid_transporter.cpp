/*
 * Copyright (C) 2025 Open Source Robotics Foundation.
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

#include <nexus_transporter_msgs/msg/destination.hpp>

#include "bid_transporter.hpp"

namespace nexus::capabilities {

//==============================================================================
BT::PortsList BidTransporter::providedPorts()
{
  return {
    BT::InputPort<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations"),
    BT::OutputPort<std::string>("result")
  };
}

//==============================================================================
rclcpp::Client<endpoints::BidTransporterService::ServiceType>::SharedPtr
BidTransporter::client()
{
  auto node = this->_w_node.lock();
  return node->create_client<endpoints::BidTransporterService::ServiceType>(
    endpoints::BidTransporterService::service_name());
}

//==============================================================================
endpoints::BidTransporterService::ServiceType::Request::SharedPtr
BidTransporter::make_request()
{
  auto node = this->_w_node.lock();

  auto maybe_destinations =
    this->getInput<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations");
  if (!maybe_destinations)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s: [destinations] param is required",
      this->name().c_str());
    return nullptr;
  }

  auto req =
    std::make_shared<endpoints::BidTransporterService::ServiceType::Request>();

  req->request.requester = node->get_name();
  req->request.id = this->_ctx_mgr->current_context()->task.task_id;
  req->request.destinations = *maybe_destinations;
  return req;
}

//==============================================================================
bool BidTransporter::on_response(
  rclcpp::Client<endpoints::BidTransporterService::ServiceType>::SharedResponse
  resp)
{
  if (resp->available)
  {
    this->setOutput("result", resp->transporter);
  }
  return resp->available;
}

} // namespace nexus::capabilities
