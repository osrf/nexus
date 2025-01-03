/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__RMF_REQUEST_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__RMF_REQUEST_HPP

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common/sync_service_client.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <deque>

namespace nexus::capabilities {

namespace rmf {

struct Destination {
  // Either "pickup" or "dropoff"
  std::string action;
  std::string workcell;
};

/**
 * Returns RUNNING until a signal is received from system orchestrator.
 *
 * Input Ports:
 *   signal |std::string| Signal to wait for.
 *   clear |bool| Set this to true to clear the signal when this node is finished.
 */
class DispatchRequest : public BT::StatefulActionNode
{
// RMF interfaces
public: using ApiRequest = rmf_task_msgs::msg::ApiRequest;
public: using ApiResponse = rmf_task_msgs::msg::ApiResponse;

public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::deque<Destination>>("destinations", "Destinations to visit"),
      BT::OutputPort<std::string>("rmf_task_id", "The resulting RMF task id")
    };
  }

public: DispatchRequest(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(std::move(node)) {}

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override {}

private: void submit_itinerary(const std::deque<Destination>& destinations);

private: void api_response_cb(const ApiResponse& msg);

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;

private: rclcpp::Publisher<ApiRequest>::SharedPtr _api_request_pub;
private: rclcpp::Subscription<ApiResponse>::SharedPtr _api_response_sub;

// Populated by the API response callback
private: std::optional<std::string> rmf_task_id;
private: std::string requested_id;
};

// Parses the current task in the context and returns a queue of destinations to visit
class ExtractDestinations : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::deque<Destination>>("destinations"),
    };
  }

public: BT::NodeStatus tick() override;

public: ExtractDestinations(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node), _ctx_mgr(ctx_mgr) {}

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

class UnpackDestinationData : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Destination>("destination"),
      BT::OutputPort<std::string>("workcell"),
      BT::OutputPort<std::string>("type"),
    };
  }

public: UnpackDestinationData(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
};

// TODO(luca) consider implementing ingestors as well, will need a new input for action type
// Sends a DispenserResult to the AMR to signal that the workcell is done
class SignalAmr : public BT::SyncActionNode
{
public: using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("workcell"),
      BT::InputPort<std::string>("rmf_task_id"),
    };
  }

public: SignalAmr(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: rclcpp::Publisher<DispenserResult>::SharedPtr _dispenser_result_pub;
};

// Loops over destinations to extract them
// TODO(luca) use builtin LoopNode when migrating to behaviortreecpp v4
class LoopDestinations : public BT::DecoratorNode
{
  /**
   * Defines provided ports for the BT node.
    */
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::deque<Destination>>("queue"),
      BT::OutputPort<Destination>("value"),
    };
  }

/**
 * @param name name of the BT node.
 * @param config config of the BT node.
 */
public: LoopDestinations(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::DecoratorNode{name, config}, _node(node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: bool _initialized = false;
private: std::deque<Destination> _queue;
};

class WaitForAmr: public BT::StatefulActionNode
{
// RMF interfaces
public: using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;

public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("rmf_task_id"),
      BT::InputPort<std::string>("workcell")
    };
  }

public: WaitForAmr(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(std::move(node)) {}

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override {}

private: void dispenser_request_cb(const DispenserRequest& msg);

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;

private: rclcpp::Subscription<DispenserRequest>::SharedPtr _dispenser_request_sub;

private: std::string _rmf_task_id;
private: std::string _workcell;
private: bool _amr_ready = false;
};

/**
 * Sends a signal to the system orchestrator.
 * The signal will be tied to the current task id
 *
 * Input Ports:
 *   signal |std::string| Signal to send.
 */
class SendSignal : public BT::SyncActionNode
{
public: using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
public: static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("signal", "Signal to send.") };
  }

public: SendSignal(const std::string& name, const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node), _ctx_mgr(ctx_mgr) {}

public: BT::NodeStatus tick() override;

private: std::unique_ptr<common::SyncServiceClient<nexus::endpoints::SignalWorkcellService::ServiceType>>
  _signal_client;

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

}
}

#endif
