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

#ifndef NEXUS_CAPABILITIES__CAPABILITIES__DISPENSE_ITEM_HPP
#define NEXUS_CAPABILITIES__CAPABILITIES__DISPENSE_ITEM_HPP

#include "dispense_item_task_data.hpp"

#include <nexus_common/service_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/tree_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

namespace nexus::capabilities {

/**
 * Dispenses an item.
 *
 * Input Ports:
 *   item |std::string| The item to dispense.
 *   dispenser |std::string| OPTIONAL. Id of the dispenser to use to perform the action. If not provided, the first dispenser is used.
 */
class DispenseItem : public common::
  ServiceClientBtNode<endpoints::DispenserService::ServiceType>
{
public: struct DispenserSession
  {
    std::string dispenser_id;
    rclcpp::Client<endpoints::DispenserService::ServiceType>::SharedPtr client;
  };

  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("item", "The item to dispense."),
      BT::InputPort<std::string>(
        "dispenser",
        "OPTIONAL. Id of the dispenser to used to perform the action. If not provided, the first dispenser is used.") };
  }

  /**
   * @param name name of the BT node.
   * @param config config of the BT node.
   * @param node rclcpp node.
   * @param client_factory a callable that returns the rclcpp client to use.
   */
public: DispenseItem(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::vector<DispenserSession> dispensers,
    std::chrono::milliseconds dispenser_timeout)
  : common::ServiceClientBtNode<endpoints::DispenserService::ServiceType>(name,
      config, node->get_logger(), dispenser_timeout), _dispensers(dispensers),
    _w_node(
      node)
  {
  }

protected: rclcpp::Client<endpoints::DispenserService::ServiceType>::SharedPtr
  client() final;

protected: endpoints::DispenserService::ServiceType::Request::SharedPtr
  make_request() final;

protected: bool on_response(
    rclcpp::Client<endpoints::DispenserService::ServiceType>::SharedResponse resp)
  final;

private: std::vector<DispenserSession> _dispensers;
private: rclcpp_lifecycle::LifecycleNode::WeakPtr _w_node;
};

} // namespace nexus::capabilities

#endif
