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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__WORKCELL_ORCHESTRATOR_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__WORKCELL_ORCHESTRATOR_HPP

#include "task_parser.hpp"

#include <nexus_capabilities/capability.hpp>
#include <nexus_capabilities/context.hpp>
#include <nexus_capabilities/context_manager.hpp>
#include <nexus_capabilities/task_checker.hpp>

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_common/bt_store.hpp>
#include <nexus_common/task_remapper.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <nexus_endpoints.hpp>

#include <nexus_lifecycle_manager/lifecycle_manager.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <memory>
#include <vector>

namespace nexus::workcell_orchestrator {

class WorkcellOrchestrator : public
  rclcpp_lifecycle::LifecycleNode
{
private: using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public: using GoalHandlePtr =
    std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>;
public: using WorkcellState = endpoints::WorkcellStateTopic::MessageType;

public: WorkcellOrchestrator(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

public: CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

public: CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state)
  override;

public: CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
  override;

public: CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state)
  override;

private: rclcpp_action::Server<endpoints::WorkcellRequestAction::ActionType>::
  SharedPtr
    _cmd_server;
private: common::BTStore _bt_store;
private: std::unique_ptr<BT::BehaviorTreeFactory> _bt_factory;
private: rclcpp::TimerBase::SharedPtr _bt_timer;

  // ros objects used in registration
private: rclcpp::Client<endpoints::RegisterWorkcellService::ServiceType>::
  SharedPtr
    _register_workcell_client;
private: std::optional<rclcpp::Client<endpoints::RegisterWorkcellService::ServiceType>::
    SharedFutureAndRequestId> _ongoing_register;
private: rclcpp::TimerBase::SharedPtr _register_timer;
private: rclcpp::TimerBase::SharedPtr _register_timeout_timer;
private: tf2_ros::Buffer::SharedPtr _tf2_buffer;
private: std::unique_ptr<tf2_ros::TransformListener> _tf2_listener;

private: rclcpp::Publisher<endpoints::WorkcellStateTopic::MessageType>::
  SharedPtr
    _wc_state_pub;
private: WorkcellState _cur_state;
private: rclcpp::Service<endpoints::IsTaskDoableService::ServiceType>::SharedPtr
    _task_doable_srv;
private: rclcpp::Service<endpoints::SignalWorkcellService::ServiceType>::
  SharedPtr
    _signal_wc_srv;
private: std::future<void> _task_done;
  /// pluginlib clasloader to dynamically load capability plugins.
private: pluginlib::ClassLoader<Capability> _capability_loader;
private: std::unordered_map<std::string,
    pluginlib::UniquePtr<Capability>> _capabilities;
private: rclcpp::Service<endpoints::PauseWorkcellService::ServiceType>::
  SharedPtr
    _pause_srv;
private: rclcpp::Service<endpoints::QueueWorkcellTaskService::ServiceType>::
  SharedPtr
    _queue_task_srv;
private: rclcpp::Service<endpoints::RemovePendingTaskService::ServiceType>::
  SharedPtr
    _remove_pending_task_srv;
private: std::atomic<bool> _paused;
private: std::list<std::shared_ptr<Context>> _ctxs;
private: std::shared_ptr<ContextManager> _ctx_mgr;
private: std::unique_ptr<lifecycle_manager::LifecycleManager<>> _lifecycle_mgr{
    nullptr};
// Takes care of remapping tasks
private: std::shared_ptr<common::TaskRemapper> _task_remapper;
private: TaskParser _task_parser;
private: pluginlib::ClassLoader<TaskChecker> _task_checker_loader;
private: std::shared_ptr<TaskChecker> _task_checker;

private: CallbackReturn _configure(
    const rclcpp_lifecycle::State& previous_state);

  /**
   * Tick a behavior tree.
   */
private: void _tick_bt(const std::shared_ptr<Context>& ctx);

private: void _tick_all_bts();

private: void _cancel_all_tasks();

  /**
   * Register this workcell to the system orchestrator.
   */
private: void _register();

private: void _handle_register_response(
    endpoints::RegisterWorkcellService::ServiceType::Response::ConstSharedPtr resp);

private: void _process_signal(
    endpoints::SignalWorkcellService::ServiceType::Request::ConstSharedPtr req,
    endpoints::SignalWorkcellService::ServiceType::Response::SharedPtr resp);

  /**
   * Creates a BT from the workcell request. The context must already have a task.
   */
private: BT::Tree _create_bt(const std::shared_ptr<Context>& ctx);

private: void _handle_command_success(const std::shared_ptr<Context>& ctx);

private: void _handle_command_failed(const std::shared_ptr<Context>& ctx);

private: void _handle_task_doable(
    endpoints::IsTaskDoableService::ServiceType::Request::ConstSharedPtr req,
    endpoints::IsTaskDoableService::ServiceType::Response::SharedPtr resp);

private: int _max_parallel_jobs = 1;

private: std::unordered_set<std::string> _input_stations = {};

private: std::unordered_set<std::string> _output_stations = {};
};

}

#endif
