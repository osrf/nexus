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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__SYSTEM_ORCHESTRATOR_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__SYSTEM_ORCHESTRATOR_HPP

#include "job.hpp"
#include "session.hpp"

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_common/models/work_order.hpp>
#include <nexus_common/sync_service_client.hpp>
#include <nexus_common/task_remapper.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_lifecycle_manager/lifecycle_manager.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <nexus_orchestrator_msgs/msg/work_order_state.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nexus::system_orchestrator {

class SystemOrchestrator : public
  rclcpp_lifecycle::LifecycleNode
{
public:
  using WorkOrderAction = nexus::endpoints::WorkOrderAction;
  using WorkOrderActionType = WorkOrderAction::ActionType;
  using WorkOrderGoalHandle =
    rclcpp_action::ServerGoalHandle<WorkOrderActionType>;
  using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
  using WorkOrderState = nexus_orchestrator_msgs::msg::WorkOrderState;

  SystemOrchestrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override;

private:
  std::unordered_map<std::string, Job> _jobs;
  int64_t _max_parallel_jobs = 0;
  int64_t _bid_request_timeout = 5000;
  rclcpp::TimerBase::SharedPtr _jobs_timer;
  rclcpp_action::Server<WorkOrderActionType>::SharedPtr _work_order_srv;
  rclcpp::Service<endpoints::ListWorkcellsService::ServiceType>::SharedPtr
    _list_workcells_srv;
  rclcpp::ServiceBase::SharedPtr _register_workcell_srv;
  rclcpp::Service<endpoints::ListTransporterService::ServiceType>::SharedPtr
    _list_transporters_srv;
  rclcpp::ServiceBase::SharedPtr _register_transporter_srv;
  bool _paused;
  rclcpp::ServiceBase::SharedPtr _pause_system_srv;
  rclcpp::Publisher<endpoints::WorkOrderStatesTopic::MessageType>::SharedPtr
    _wo_states_pub;
  rclcpp::Service<endpoints::GetWorkOrderStateService::ServiceType>::SharedPtr
    _get_wo_state_srv;
  std::filesystem::path _bt_path;
  std::string _main_bt_filename;
  rclcpp::SubscriptionBase::SharedPtr _cancel_wo_sub;
  bool _activated = false;
  bool _estop_pressed = false;
  std::unordered_map<std::string,
    std::shared_ptr<WorkcellSession>> _workcell_sessions;
  std::unordered_map<std::string,
    std::shared_ptr<TransporterSession>> _transporter_sessions;
  std::unique_ptr<lifecycle_manager::LifecycleManager<>> _lifecycle_mgr{nullptr};
  rclcpp::TimerBase::SharedPtr _pre_configure_timer;
  rclcpp::SubscriptionBase::SharedPtr _estop_sub;
  // Manages task remapping
  std::shared_ptr<common::TaskRemapper> _task_remapper;
  std::shared_ptr<OnSetParametersCallbackHandle> _param_cb_handle;

  /**
   * Creates a BT from a work order.
   */
  BT::Tree _create_bt(const WorkOrderActionType::Goal& wo,
    std::shared_ptr<Context> ctx);

  /**
   * Create a new job and adds it to the list of current jobs. The new job is
   * queued as "not ready", `_init_job` needs to be called to populate the
   * required contexts and start the job.
   * @throws std::exception
   */
  void _create_job(
    const std::string& work_order_id, const WorkOrderActionType::Goal& goal);

  /**
   * Assigns all tasks and start the job associated with the goal handle.
   * If there is an error starting the job, the goal will be aborted with
   * an appropriate error.
   */
  void _init_job(const std::shared_ptr<WorkOrderGoalHandle> goal_handle);

  std::string _generate_task_id(
    const std::string& work_order_id,
    const std::string& process_id,
    std::size_t) const;

  std::vector<nexus_orchestrator_msgs::msg::WorkcellTask> _parse_wo(
    const std::string& work_order_id, const common::WorkOrder& work_order);

  void _handle_wo_cancel(
    const WorkOrderActionType::Goal& goal);

  void _handle_wo_failed(const Job& job);

  void _handle_register_workcell(
    endpoints::RegisterWorkcellService::ServiceType::Request::ConstSharedPtr req,
    endpoints::RegisterWorkcellService::ServiceType::Response::SharedPtr resp);

  void _handle_register_transporter(
    endpoints::RegisterTransporterService::ServiceType::Request::ConstSharedPtr req,
    endpoints::RegisterTransporterService::ServiceType::Response::SharedPtr resp);

  /**
   * Halt and fail a job from the job list.
   */
  void _halt_job(const std::string& job_id);

  /**
   * Halt, fail and remove a job from the job list.
   *
   * Be careful when calling this while there are references to the job as they
   * will be invalidated.
   */
  void _halt_fail_and_remove_job(const std::string& job_id);

  /**
   * Halt, fail and remove all jobs.
   */
  void _halt_fail_and_remove_all_jobs();

  /**
   * Spins all current BT once.
   */
  void _spin_bts_once();

  std::string _workcell_namespace(const std::string& workcell_id);

  /**
   * Checks if the requested filename points to a proper file
   */
  bool _bt_filename_valid(const std::string& bt_filename) const;

  /**
   * Send bid requests and assign the task to the most suitable workcell.
   * Currently this always assign to the first workcell that accepts the bid.
   *
   * @param on_done Called with the assigned workcell id or std::nullopt if no
   *   workcell is able to perform the task.
   */
private: void _assign_workcell_task(const WorkcellTask& task,
    std::function<void(const std::optional<std::string>& assigned_wc)> on_done);

  /**
   * Batch assign tasks to workcells.
   *
   * @param on_done Called with the map of task ids and their assigned workcell ids
   *   when all tasks have been assigned.
   */
private: void _assign_all_tasks(const std::vector<WorkcellTask>& tasks,
    std::function<void(const std::unordered_map<std::string,
    std::optional<std::string>>&)> on_done);

private: WorkOrderState _get_wo_state(const Job& job)
  const;

private: std::vector<WorkOrderState> _get_current_wo_states() const;

  /**
  * Publishes work order feedback based on the current state of workcells.
  * Be sure to update the states in `workcell_sessions` before calling this.
  */
private: void _publish_wo_feedback(const Context& ctx);

private: void _publish_wo_states(const Job& job);
};

}

#endif
