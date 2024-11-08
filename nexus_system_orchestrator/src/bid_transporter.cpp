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

#include "bid_transporter.hpp"

namespace nexus::system_orchestrator {

BT::PortsList BidTransporter::providedPorts()
{
  return { BT::InputPort<std::string>("destination"),
    BT::OutputPort<std::string>("result") };
}

BT::NodeStatus BidTransporter::onStart()
{
  auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  auto maybe_destination = this->getInput<std::string>("destination");
  if (!maybe_destination)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s: [destination] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto& destination = maybe_destination.value();

  // Source is not a mandatory parameter
  std::string source = "";
  auto maybe_source = this->getInput<std::string>("source");
  if (maybe_source)
  {
    source = maybe_source.value();
  }

  auto req =
    std::make_shared<endpoints::IsTransporterAvailableService::ServiceType::Request>();
  req->request.id = this->_ctx->job_id;
  req->request.requester = node->get_name();
  req->request.destination = destination;
  req->request.source = source;
  // send request to all transporters in parallel
  for (auto& [transporter_id, session] : this->_ctx->transporter_sessions)
  {
    auto fut = session->available_client->async_send_request(req);
    this->_ongoing_requests.emplace(transporter_id,
      OngoingRequest{session->available_client, std::move(fut)});
  }
  this->_time_limit = std::chrono::steady_clock::now() +
    std::chrono::seconds{5};

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BidTransporter::onRunning()
{
  auto result = this->_update_ongoing_requests();
  if (result != BT::NodeStatus::RUNNING)
  {
    // action is done, drop requests that have not received response to avoid callback leak.
    this->_cleanup_pending_requests();
  }
  return result;
}

void BidTransporter::onHalted()
{
  // can't cancel a service call, so we just drop the requests.
  this->_cleanup_pending_requests();
}

void BidTransporter::_cleanup_pending_requests()
{
  for (auto& [_, ongoing_req] : this->_ongoing_requests)
  {
    ongoing_req.client->remove_pending_request(ongoing_req.fut);
  }
}

BT::NodeStatus BidTransporter::_update_ongoing_requests()
{
  auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  if (std::chrono::steady_clock::now() > this->_time_limit)
  {
    for (const auto& [transporter_id, _] : this->_ongoing_requests)
    {
      RCLCPP_WARN(
        node->get_logger(), "%s: Skipped transporter [%s] (no response)",
        this->name().c_str(),
        transporter_id.c_str());
    }
    RCLCPP_ERROR(
      node->get_logger(), "%s: No transporter is able to perform task",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::string> finished_requests;
  BT::NodeStatus result{BT::NodeStatus::RUNNING};
  for (auto& [transporter_id, ongoing_req] : this->_ongoing_requests)
  {
    if (ongoing_req.fut.wait_for(std::chrono::seconds{0}) ==
      std::future_status::ready)
    {
      finished_requests.emplace_back(transporter_id);
      auto resp = ongoing_req.fut.get();
      // use first available transporter
      if (resp->available)
      {
        RCLCPP_INFO(
          node->get_logger(), "[%s]: Bid awarded to [%s]",
          this->name().c_str(),
          transporter_id.c_str());
        this->setOutput("result", transporter_id);
        result = BT::NodeStatus::SUCCESS;
        break;
      }
      else
      {
        RCLCPP_DEBUG(
          node->get_logger(), "%s: Transporter [%s] cannot perform task",
          this->name().c_str(), transporter_id.c_str());
      }
    }
  }

  for (const auto& transporter_id : finished_requests)
  {
    this->_ongoing_requests.erase(transporter_id);
  }

  return result;
}

}
