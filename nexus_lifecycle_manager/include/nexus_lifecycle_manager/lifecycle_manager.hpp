// Copyright 2022 Johnson & Johnson
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

#ifndef NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_transitions.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nexus_lifecycle_msgs/srv/manage_lifecycle_nodes.hpp>

#include <nexus_common/lifecycle_service_client.hpp>
#include <nexus_common/logging.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rmf_utils/impl_ptr.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using nexus::common::LifecycleServiceClient;
using nexus_lifecycle_msgs::srv::ManageLifecycleNodes;

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

namespace nexus::lifecycle_manager {

using ManageLifecycleNodes = nexus_lifecycle_msgs::srv::ManageLifecycleNodes;

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT& future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do
  {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0))
    {
      break;
    }
    status =
      future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

template<class NodeType = rclcpp_lifecycle::LifecycleNode>
class Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string,
    std::shared_ptr<nexus::common::LifecycleServiceClient<rclcpp::Node>>>
  _node_clients;

  bool _system_active{false};
  std::chrono::seconds _service_request_timeout{10s};
  // Lifecycle state that managed nodes should be in
  uint8_t _target_state = State::PRIMARY_STATE_UNCONFIGURED;

  std::shared_ptr<std::thread> spin_thread_;
  // Callback group used by services
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // The services provided by this node
  bool _are_services_active{false};
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr _manager_srv{nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _is_active_srv{nullptr};

  LifecycleTransitions transitionGraph;

  void message(const std::string& msg)
  {
    RCLCPP_INFO(
      this->node_->get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET,
      msg.c_str());
  }

  bool changeStateForAllNodes(std::uint8_t transition)
  {
    bool ok = true;
    for (const auto& [name, client] : this->_node_clients)
    {
      std::optional<uint8_t> state = client->get_state();
      if (!state.has_value())
      {
        ok = false;
        continue;
      }

      ok &= transitionGraph.atGoalState(state.value(), transition) ||
        client->change_state(transition);
      this->_target_state = state.value();
    }
    return ok;
  }

  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = this->_system_active;
  }

  void shutdownAllNodes()
  {
    this->message("Deactivate, cleanup, and shutdown nodes");

    this->changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
    this->changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
    this->changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  void destroyLifecycleServiceClients()
  {
    this->message("Destroying lifecycle service clients");
    for (auto& kv : this->_node_clients)
    {
      kv.second.reset();
    }
  }

  bool startup()
  {
    this->message("Starting managed nodes bringup...");
    if (!this->changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
      !this->changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Failed to bring up all requested nodes. Aborting bringup.");
      return false;
    }
    this->message("Managed nodes are active");
    this->_system_active = true;
    return true;
  }

  bool shutdown()
  {
    this->_system_active = false;

    this->message("Shutting down managed nodes...");
    this->shutdownAllNodes();
    this->destroyLifecycleServiceClients();
    this->message("Managed nodes have been shut down");
    return true;
  }

  bool reset()
  {
    this->_system_active = false;

    this->message("Resetting managed nodes...");
    // Should transition in reverse order
    if (!this->changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE) ||
      !this->changeStateForAllNodes(Transition::TRANSITION_CLEANUP))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Failed to reset nodes: aborting reset");
      return false;
    }

    this->message("Managed nodes have been reset");
    return true;
  }
  bool pause()
  {
    this->_system_active = false;

    this->message("Pausing managed nodes...");
    if (!this->changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Failed to pause nodes: aborting pause");
      return false;
    }

    this->message("Managed nodes have been paused");
    return true;
  }

  bool resume()
  {
    this->message("Resuming managed nodes...");
    if (!this->changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Failed to resume nodes: aborting resume");
      return false;
    }

    this->message("Managed nodes are active");
    this->_system_active = true;
    return true;
  }

  bool ChangeState(const std::string& _node_name, std::uint8_t transition)
  {
    auto client_change_state_ =
      this->node_->template create_client<lifecycle_msgs::srv::ChangeState>(
      _node_name + std::string("/change_state"));
    auto request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(_service_request_timeout))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Service %s is not available.",
        client_change_state_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result =
      client_change_state_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status =
      wait_for_result(future_result, _service_request_timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Server time out while getting current state for node %s",
        _node_name.c_str());
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success)
    {
      RCLCPP_INFO(
        this->node_->get_logger(), "Transition %d successfully triggered.",
        static_cast<int>(transition));
      return true;
    }
    else
    {
      RCLCPP_WARN(
        this->node_->get_logger(), "Failed to trigger transition %u",
        static_cast<unsigned int>(transition));
      return false;
    }
  }

  bool GetState(
    const std::string& _node_name, uint8_t& _state)
  {
    message("Getting the state of " + _node_name);
    auto service_get_state_name = std::string("/") +
      _node_name + std::string("/get_state");
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto client_get_state =
      this->node_->template create_client<lifecycle_msgs::srv::GetState>(
      service_get_state_name);

    if (!client_get_state->wait_for_service(_service_request_timeout))
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Service %s is not available.",
        client_get_state->get_service_name());
      return false;
    }
    // We send the service request for asking the current
    // state of the lc_talker node.std::shared_ptr<rclcpp::Node> _node = nullptr)
    auto future_result =
      client_get_state->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status =
      wait_for_result(future_result, _service_request_timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Server time out while getting current state for node %s",
        service_get_state_name.c_str());
      return false;
    }
    // We have an succesful answer. So let's print the current state.
    if (future_result.get())
    {
      RCLCPP_INFO(
        this->node_->get_logger(), "Node %s has current state %s.",
        service_get_state_name.c_str(),
        future_result.get()->current_state.label.c_str());

      _state = future_result.get()->current_state.id;
    }
    return true;
  }

  bool add_node(const std::string& name)
  {
    uint8_t currentState;
    if (!this->GetState(name, currentState))
    {
      message("The state of new node to add was not reacheable. "
        "This node is not inserted");
      return false;
    }

    // If the target state is different from the current, transition this node to it
    std::vector<int> solution = transitionGraph.dijkstra(currentState, this->_target_state);
    for (unsigned int i = 0; i < solution.size(); i++)
    {
      if (!ChangeState(name, solution[i]))
      {
        RCLCPP_ERROR(
          this->node_->get_logger(),
          "Not able to transition the node [%s]. This node is not inserted",
          name.c_str());
        return false;
      }
    }
    _node_clients.insert_or_assign(name,
      std::make_shared<nexus::common::LifecycleServiceClient<rclcpp::Node>>(
        name));
    return true;
  }

  bool change_state(const std::string& node_name, const std::uint8_t goal)
  {
    auto it = _node_clients.find(node_name);
    if (it == _node_clients.end())
    {
      return false;
    }

    uint8_t currentState;
    if (!this->GetState(node_name, currentState))
    {
      message("The state of new node to add was not recheable, "
        "Keep the new node is unconfigure state");
      return false;
    }

    std::vector<int> solution = transitionGraph.dijkstra(currentState, goal);
    for (unsigned int i = 0; i < solution.size(); i++)
    {
      if (!it->second->change_state(solution[i]))
      {
        RCLCPP_ERROR(
          this->node_->get_logger(),
          "Not able to transition the node [%s]. This node is not inserted",
          node_name.c_str());
        return false;
      }
    }
    return true;
  }
};

/**
 * @class nexus_lifecycle_manager::LifecycleManager
 * @brief Implements service interface to transition the lifecycle nodes of the
 * stack. It receives transition request and then uses lifecycle
 * interface to change lifecycle node's state.
 */
template<class NodeType = rclcpp_lifecycle::LifecycleNode>
class LifecycleManager
{
public:

  /// \brief A constructor for nexus::lifecycle_manager::LifecycleManager
  /// \param[in] node ROS node.
  /// \param[in] node_names Node names to manage the lifecycle.
  explicit LifecycleManager(
    const std::string& name,
    std::vector<std::string> node_names,
    bool activate_services = false,
    bool autostart = false,
    int service_request_timeout = 10)
  {
    _pimpl = rmf_utils::make_impl<Implementation<NodeType>>();
    _pimpl->_are_services_active = activate_services;
    // Set the target state to active if autostart is enabled
    if (autostart)
    {
      _pimpl->_target_state = State::PRIMARY_STATE_ACTIVE;
    }
    _pimpl->_service_request_timeout = std::chrono::seconds(
      service_request_timeout);

    std::string node_name = name + "_lifecycle_manager";
    _pimpl->node_ = std::make_shared<rclcpp::Node>(
      node_name, "",
      rclcpp::NodeOptions().arguments({"--ros-args", "-r",
        std::string("__node:=") + node_name, }));
    common::configure_logging(_pimpl->node_);

    _pimpl->spin_thread_ = std::make_shared<std::thread>([&]()
        {
          while (rclcpp::ok())
          {
            rclcpp::spin_some(_pimpl->node_);
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
          }
        });

    // We register any srv callbacks on parent node callback group
    auto n = _pimpl->node_;
    if (n && _pimpl->_are_services_active)
    {
      _pimpl->callback_group_ = n->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      _pimpl->_is_active_srv =
        n->template create_service<std_srvs::srv::Trigger>(
        "/" + node_name + std::string("/is_active"),
        std::bind(&Implementation<NodeType>::isActiveCallback,
        _pimpl.get(), _1, _2, _3),
        rclcpp::ServicesQoS(),
        _pimpl->callback_group_);

      _pimpl->_manager_srv = n->template create_service<ManageLifecycleNodes>(
        "/" + node_name + std::string("/manage_nodes"),
        [this](
          const ManageLifecycleNodes::Request::ConstSharedPtr request,
          ManageLifecycleNodes::Response::SharedPtr response)
        {
          switch (request->command)
          {
            case ManageLifecycleNodes::Request::STARTUP:
              response->success = this->_pimpl->startup();
              break;
            case ManageLifecycleNodes::Request::RESET:
              response->success = this->_pimpl->reset();
              break;
            case ManageLifecycleNodes::Request::SHUTDOWN:
              response->success = this->_pimpl->shutdown();
              break;
            case ManageLifecycleNodes::Request::PAUSE:
              response->success = this->_pimpl->pause();
              break;
            case ManageLifecycleNodes::Request::RESUME:
              response->success = this->_pimpl->resume();
              break;
            default:
              response->success = false;
              break;
          }
        },
        rclcpp::ServicesQoS(),
        _pimpl->callback_group_);
    }

    // Create map
    for (const auto& n : node_names)
    {
      _pimpl->add_node(n);
    }
  }

  ~LifecycleManager()
  {
    if (_pimpl->spin_thread_ && _pimpl->spin_thread_->joinable())
      _pimpl->spin_thread_->join();
  }

  /// \brief Add a node to the list
  /// \param[in] node_name_ New name to add to the list
  /// \return True if the node was added, False otherwise
  bool addNodeName(const std::string& node_name_)
  {
    return _pimpl->add_node(node_name_);
  }

  /// \brief Remove a node from the list
  /// \param[in] node_name_ Node name to remove
  /// \return True if the node name was remove from the list, false otherwise
  bool removeNodeName(const std::string& node_name_)
  {
    _pimpl->_node_clients.erase(node_name_);
    return true;
  }

  /**
   * @brief For each node in the map, transition to the new target state
   * If this method fails the state of some nodes might be undetermined
   *
   * @param transition Desired state, define in the nexus_lifecycle_msgs/ManageLifecycleNodes.srv
   *
   *  uint8 STARTUP = 0
   *  uint8 PAUSE = 1
   *  uint8 RESUME = 2
   *  uint8 RESET = 3
   *  uint8 SHUTDOWN = 4
   * \return True is everything went right, false otherwise
   */
  bool changeStateForAllNodes(std::uint8_t transition)
  {
    return _pimpl->changeStateForAllNodes(transition);
  }

  bool getState(const std::string& node_name_, std::uint8_t& state)
  {
    return _pimpl->GetState(node_name_, state);
  }

  // class Implementation;

private:
  rmf_utils::impl_ptr<Implementation<NodeType>> _pimpl;
};

}  // namespace nexus_lifecycle_manager

#endif  // NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
