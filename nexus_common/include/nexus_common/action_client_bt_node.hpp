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

#ifndef NEXUS_COMMON__ACTION_CLIENT_BT_NODE_HPP
#define NEXUS_COMMON__ACTION_CLIENT_BT_NODE_HPP

#include "node_utils.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/srv/cancel_goal.hpp>

#include <chrono>
#include <future>
#include <optional>
#include <memory>
#include <string>

namespace nexus::common {

/**
 * A BT action node that sends a goal to an action server.
 *
 * @tparam ActionT Type of the action.
 */
template<typename NodePtrT, typename ActionT>
class ActionClientBtNode : public BT::StatefulActionNode
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;


  /**
   * @param name The name of the node.
   * @param config BT node configuration.
   * @param node rclcpp node.
   * @param timeout The amount of time to wait for a goal response,
   *   this includes time spent in discovery.
   */
public: ActionClientBtNode(const std::string& name,
    const BT::NodeConfiguration& config, NodePtrT node,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(
      1000));

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override;

protected: NodePtrT _node;

protected: virtual std::string get_action_name() const = 0;

  /**
   * Returns the goal to send to the server.
   */
protected: virtual std::optional<typename ActionT::Goal> make_goal() = 0;

protected: virtual void on_feedback(typename ActionT::Feedback::ConstSharedPtr);

  /**
   * Called when the action finishes. Implementations should return true if
   * the action is considered a success and false otherwise.
   */
protected: virtual bool on_result(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result);

private: rclcpp::executors::SingleThreadedExecutor _executor;
private: rclcpp::CallbackGroup::SharedPtr _cb_group;
private: std::chrono::milliseconds _timeout;
private: typename rclcpp_action::Client<ActionT>::SharedPtr _client = nullptr;
private: typename GoalHandle::SharedPtr _goal_handle = nullptr;
private: std::shared_future<typename GoalHandle::WrappedResult> _result_fut;
};

/******************
 * Implementation *
 ******************/

template<typename NodePtrT, typename ActionT>
ActionClientBtNode<NodePtrT, ActionT>::ActionClientBtNode(
  const std::string& name,
  const BT::NodeConfiguration& config, NodePtrT node,
  std::chrono::milliseconds timeout)
: BT::StatefulActionNode(name, config),
  _node(std::move(node)),
  _executor(create_executor(this->_node)),
  _cb_group(this->_node->create_callback_group(rclcpp::CallbackGroupType::
    MutuallyExclusive, false)),
  _timeout(std::move(timeout))
{
  this->_executor.add_callback_group(this->_cb_group,
    this->_node->get_node_base_interface());
}

template<typename NodePtrT, typename ActionT>
BT::NodeStatus ActionClientBtNode<NodePtrT, ActionT>::onStart()
{
  this->_client = rclcpp_action::create_client<ActionT>(this->_node,
      this->get_action_name(),
      this->_cb_group);

  // synchronously send goal to avoid race conditions
  const auto start = std::chrono::steady_clock::now();
  if (!this->_client->wait_for_action_server(this->_timeout))
  {
    RCLCPP_DEBUG(
      this->_node->get_logger(), "%s: Cannot find action server",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto remaining_timeout = start + this->_timeout -
    std::chrono::steady_clock::now();

  typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
  options.feedback_callback =
    [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      typename ActionT::Feedback::ConstSharedPtr feedback)
    {
      this->on_feedback(feedback);
    };

  RCLCPP_INFO(this->_node->get_logger(), "%s: Sending goal",
    this->name().c_str());
  const auto goal = this->make_goal();
  if (!goal)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: Failed to make goal",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto goal_fut = this->_client->async_send_goal(*goal, options);
  if (!goal_fut.valid())
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: Failed to send goal",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (this->_executor.spin_until_future_complete(goal_fut,
    remaining_timeout) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: Timed out waiting for goal response",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_goal_handle = goal_fut.get();
  if (!this->_goal_handle || (this->_goal_handle->get_status() !=
    rclcpp_action::GoalStatus::STATUS_ACCEPTED &&
    this->_goal_handle->get_status() !=
    rclcpp_action::GoalStatus::STATUS_EXECUTING &&
    this->_goal_handle->get_status() !=
    rclcpp_action::GoalStatus::STATUS_SUCCEEDED))
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: Goal is rejected",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG(
    this->_node->get_logger(), "%s: Goal is accepted",
    this->name().c_str());
  this->_result_fut = this->_client->async_get_result(this->_goal_handle);
  return BT::NodeStatus::RUNNING;
}

template<typename NodePtrT, typename ActionT>
BT::NodeStatus ActionClientBtNode<NodePtrT, ActionT>::onRunning()
{
  this->_executor.spin_some();

  // check if goal is completed
  if (this->_result_fut.wait_for(std::chrono::seconds(0)) !=
    std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }
  auto result = this->_result_fut.get();
  bool success = this->on_result(result);

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !success)
  {
    RCLCPP_ERROR(this->_node->get_logger(), "%s: Goal failed",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    this->_node->get_logger(), "%s: Goal finished successfully",
    this->name().c_str());
  return BT::NodeStatus::SUCCESS;
}

template<typename NodePtrT, typename ActionT>
void ActionClientBtNode<NodePtrT, ActionT>::onHalted()
{
  if (this->_goal_handle)
  {
    RCLCPP_INFO(this->_node->get_logger(), "%s: Cancelling goal",
      this->name().c_str());
    auto fut = this->_client->async_cancel_goal(this->_goal_handle);
    this->_executor.spin_until_future_complete(fut);
    action_msgs::srv::CancelGoal::Response::SharedPtr cancel_resp = fut.get();
    if (cancel_resp->return_code !=
      decltype(cancel_resp)::element_type::ERROR_NONE)
    {
      RCLCPP_ERROR(
        this->_node->get_logger(),
        "failed to cancel goal, running action to completion");
      // run action to completion
      while (this->onRunning() == BT::NodeStatus::RUNNING)
      {
      }
    }
  }

  this->_result_fut = decltype(this->_result_fut){};
  this->_goal_handle.reset();
  this->_client.reset();
  this->_executor.cancel();
}

template<typename NodePtrT, typename ActionT>
void ActionClientBtNode<NodePtrT, ActionT>::on_feedback(
  typename ActionT::Feedback::ConstSharedPtr)
{
  // no-op
}

template<typename NodePtrT, typename ActionT>
bool ActionClientBtNode<NodePtrT, ActionT>::on_result(
  const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result)
{
  return true;
}

}

#endif
