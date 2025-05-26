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

#ifndef SRC__CAPABILITIES__BID_TRANSPORTER_HPP
#define SRC__CAPABILITIES__BID_TRANSPORTER_HPP

#include <nexus_common/service_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::capabilities {

class BidTransporter : public nexus::common::
  ServiceClientBtNode<endpoints::BidTransporterService::ServiceType>
{
public: static BT::PortsList providedPorts();

public: inline BidTransporter(
  const std::string& name,
  const BT::NodeConfiguration& config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::chrono::milliseconds bid_transporter_timeout
) : ServiceClientBtNode<endpoints::BidTransporterService::ServiceType>(
  name, config, node->get_logger(), bid_transporter_timeout), _w_node(node) {}

protected: rclcpp::Client<endpoints::BidTransporterService::ServiceType>::
  SharedPtr client() override;

protected: endpoints::BidTransporterService::ServiceType::Request::SharedPtr
  make_request() override;

protected: bool on_response(
  rclcpp::Client<endpoints::BidTransporterService::ServiceType>::
  SharedResponse resp) override;

private: rclcpp_lifecycle::LifecycleNode::WeakPtr _w_node;
};

}  // namespace nexus::capabilities

#endif  // SRC__CAPABILITIES__BID_TRANSPORTER_HPP
