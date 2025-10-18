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

#include "system_orchestrator.hpp"

#include "bid_transporter.hpp"
#include "context.hpp"
#include "create_transporter_task.hpp"
#include "exceptions.hpp"
#include "execute_task.hpp"
#include "for_each_task.hpp"
#include "job.hpp"
#include "send_signal.hpp"
#include "transporter_request.hpp"
#include "workcell_request.hpp"

#include <nexus_common/batch_service_call.hpp>
#include <nexus_common/logging.hpp>
#include <nexus_common/models/work_order.hpp>
#include <nexus_common/pausable_sequence.hpp>
#include <nexus_common/sync_service_client.hpp>
#include <nexus_orchestrator_msgs/msg/item_at_station.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>
#include <nexus_orchestrator_msgs/msg/task_progress.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_description.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace nexus::system_orchestrator {

static constexpr std::chrono::milliseconds BT_TICK_RATE{10};

using ExecuteWorkOrder = endpoints::WorkOrderAction::ActionType;
using TaskProgress = nexus_orchestrator_msgs::msg::TaskProgress;
using TaskState = nexus_orchestrator_msgs::msg::TaskState;
using WorkcellState = endpoints::WorkcellStateTopic::MessageType;

using rcl_interfaces::msg::ParameterDescriptor;

/**
 * Error thrown when a work order cannot be completed.
 */
class ImpossibleWorkOrderError : public std::runtime_error
{
public: ImpossibleWorkOrderError(const std::string& msg)
  : std::runtime_error{msg} {}
};

SystemOrchestrator::SystemOrchestrator(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("system_orchestrator", options)
{
  common::configure_logging(this);

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Path to a directory containing behavior trees. Each file in the directory should be a behavior tree xml, the file name denotes the task type for that behavior tree.";
    this->_bt_path = this->declare_parameter("bt_path", "", desc);
    if (this->_bt_path.empty())
    {
      throw std::runtime_error("param [bt_path] is required");
    }

    if (!std::filesystem::exists(this->_bt_path) ||
      !std::filesystem::is_directory(this->_bt_path))
    {
      throw std::runtime_error(
              "path specified in [bt_path] param must be a folder");
    }
  }

  {
    ParameterDescriptor desc;
    desc.description =
      "Filename of the main behavior tree to run. Paths will be resolved relative to the \"bt_path\" parameter. Defaults to \"main.xml\".";
    this->_main_bt_filename = this->declare_parameter("main_bt_filename", "main.xml", desc);

    if (!this->_bt_filename_valid(this->_main_bt_filename))
    {
      throw std::runtime_error(
              "[bt_path] and [main_bt_filename] don't point to a file");
    }
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "A yaml containing a dictionary of task types and an array of remaps.";
    const auto param = this->declare_parameter("remap_task_types", "", desc);
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Max number of parallel work orders to execute at once, set to 0 to make it unlimited.";
    this->declare_parameter("max_jobs", 0, desc);
    this->_max_parallel_jobs = this->get_parameter("max_jobs").as_int();
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Duration (in ms) to wait for workcells and transports to response to bid requests.";
    this->declare_parameter("bid_request_timeout", 5000, desc);
    this->_bid_request_timeout =
      this->get_parameter("bid_request_timeout").as_int();
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "A list of BT node names whose state changes should not be logged.";
    this->declare_parameter("bt_logging_blocklist", std::vector<std::string>{}, desc);
  }

  this->_lifecycle_mgr =
    std::make_unique<lifecycle_manager::LifecycleManager<>>(
    this->get_name(), std::vector<std::string>{}, false, true);

  this->_pre_configure_timer = this->create_wall_timer(std::chrono::seconds{0},
      [this]()
      {
        this->_pre_configure_timer.reset();
        // FIXME(koonpeng): disable again due to https://github.com/osrf/nexus/issues/92
        // this->_lifecycle_mgr = std::make_unique<lifecycle_manager::LifecycleManager<>>(
        //   this->get_name(), std::vector<std::string>{});
      });

  this->_param_cb_handle = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter: parameters)
        {
          if (parameter.get_name() == "main_bt_filename" && !this->_bt_filename_valid(parameter.get_value<std::string>()))
          {
            result.reason = "main_bt_filename points to a non existing file";
            result.successful = false;
            break;
          }
        }
        return result;
      });

}

auto SystemOrchestrator::on_configure(const rclcpp_lifecycle::State& previous)
-> CallbackReturn
{
  // Create map for remapping capabilities.
  const std::string remap_caps = this->get_parameter("remap_task_types").as_string();
  try
  {
    YAML::Node node = YAML::Load(remap_caps);
    this->_task_remapper = std::make_shared<common::TaskRemapper>(std::move(node));
  }
  catch (YAML::ParserException& e)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to parse remap_task_types parameter: (%s)",
      e.what());
    return CallbackReturn::FAILURE;
  }

  // create list workcells service
  this->_list_workcells_srv =
    this->create_service<endpoints::ListWorkcellsService::ServiceType>(
    endpoints::ListWorkcellsService::service_name(),
    [this](endpoints::ListWorkcellsService::ServiceType::Request::
    ConstSharedPtr req,
    endpoints::ListWorkcellsService::ServiceType::Response::SharedPtr resp)
    {
      resp->workcells.reserve(this->_workcell_sessions.size());
      for (const auto& [_, wc] : this->_workcell_sessions)
      {
        resp->workcells.emplace_back(wc->description);
      }
    });

  // create action server for work order requests
  this->_work_order_srv = rclcpp_action::create_server<WorkOrderActionType>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    WorkOrderAction::action_name(),
    [this](const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const WorkOrderActionType::Goal> goal)
    -> rclcpp_action::GoalResponse
    {
      try
      {
        if (this->_max_parallel_jobs > 0 &&
        this->_jobs.size() >= static_cast<size_t>(this->_max_parallel_jobs))
        {
          auto result = std::make_shared<WorkOrderActionType::Result>();
          result->message = "Max number of parallel work orders reached";
          RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }

        this->_create_job(*goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to create job: %s", e.what());
        return rclcpp_action::GoalResponse::REJECT;
      }
    },
    [this](const std::shared_ptr<WorkOrderGoalHandle> goal_handle)
    -> rclcpp_action::CancelResponse
    {
      // handle_cancel
      this->_handle_wo_cancel(*(goal_handle->get_goal()));
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<WorkOrderGoalHandle> goal_handle)
    {
      this->_init_job(goal_handle);
    }
  );

  // create workcell registration service
  this->_register_workcell_srv =
    this->create_service<endpoints::RegisterWorkcellService::ServiceType>(
    endpoints::RegisterWorkcellService::service_name(),
    [this](
      endpoints::RegisterWorkcellService::ServiceType::Request::ConstSharedPtr
      req,
      endpoints::RegisterWorkcellService::ServiceType::Response::SharedPtr resp)
    {
      this->_handle_register_workcell(req, resp);
    });

  // create list transporters service
  this->_list_transporters_srv =
    this->create_service<endpoints::ListTransporterService::ServiceType>(
    endpoints::ListTransporterService::service_name(),
    [this](endpoints::ListTransporterService::ServiceType::Request::
    ConstSharedPtr req,
    endpoints::ListTransporterService::ServiceType::Response::SharedPtr resp)
    {
      resp->workcells.reserve(this->_transporter_sessions.size());
      for (const auto& [_, s] : this->_transporter_sessions)
      {
        resp->workcells.emplace_back(s->description);
      }
      return resp;
    });

  // create transporter registration service
  this->_register_transporter_srv =
    this->create_service<endpoints::RegisterTransporterService::ServiceType>(
    endpoints::RegisterTransporterService::service_name(),
    [this](
      endpoints::RegisterTransporterService::ServiceType::Request::
      ConstSharedPtr
      req,
      endpoints::RegisterTransporterService::ServiceType::Response::SharedPtr
      resp)
    {
      this->_handle_register_transporter(req, resp);
    });

  if (!this->_lifecycle_mgr->changeStateForAllNodes(
      Transition::TRANSITION_CONFIGURE))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Lifecycle Manager failed to configure all nodes");
    return CallbackReturn::FAILURE;
  }

  // subscribe to estop
  this->_estop_sub =
    this->create_subscription<endpoints::EmergencyStopTopic::MessageType>(
    endpoints::EmergencyStopTopic::topic_name(),
    endpoints::EmergencyStopTopic::qos(),
    [this](endpoints::EmergencyStopTopic::MessageType::ConstSharedPtr msg)
    {
      this->_estop_pressed = msg->emergency_button_stop;
      if (msg->emergency_button_stop)
      {
        // this is technically not an error, but using error to make it stand out.
        RCLCPP_ERROR(this->get_logger(),
        "Emergency stop pressed! Cancelling current work orders.");
        for (auto& [job_id, _] : this->_jobs)
        {
          this->_halt_fail_and_remove_job(job_id);
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Emergency stop is resetted");
      }
    });

  // create PauseSystem service
  this->_pause_system_srv =
    this->create_service<endpoints::PauseSystemService::ServiceType>(
    endpoints::PauseSystemService::service_name(),
    [this](rclcpp::Service<endpoints::PauseSystemService::ServiceType>::
    SharedPtr handle,
    std::shared_ptr<rmw_request_id_t> req_id,
    endpoints::PauseSystemService::ServiceType::Request::ConstSharedPtr req)
    {
      using PauseSystemService = endpoints::PauseSystemService::ServiceType;
      using PauseWorkcellService = endpoints::PauseWorkcellService::ServiceType;
      std::unordered_map<std::string,
      common::BatchServiceReq<PauseWorkcellService>> reqs;

      for (auto& p : this->_workcell_sessions)
      {
        auto& wc = p.second;
        auto wc_req = std::make_shared<endpoints::PauseWorkcellService::ServiceType::Request>();
        wc_req->pause = req->pause;
        reqs.emplace(p.first,
        common::BatchServiceReq<PauseWorkcellService>{
          wc->pause_client,
          wc_req,
        });
      }

      common::batch_service_call(this, reqs, std::chrono::milliseconds{5000},
      [this, handle, req, req_id](const std::unordered_map<std::string,
      common::BatchServiceResult<PauseWorkcellService>>&
      results)
      {
        auto resp = std::make_shared<PauseSystemService::Response>();
        resp->success = true;
        std::string verb = req->pause ? "pause" : "unpause";
        for (const auto& [wc_id, result] : results)
        {
          const auto& wc_resp = result.resp;
          if (!wc_resp->success)
          {
            std::ostringstream oss;
            oss << "Failed to " << verb << " workcell [" << wc_id << "] (" <<
              wc_resp->message << ")";
            RCLCPP_ERROR_STREAM(this->get_logger(), oss.str());
            resp->message += oss.str() + ";";
            resp->success = false;
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Workcell [%s] %sd", verb.c_str(),
            wc_id.c_str());
          }
        }
        handle->send_response(*req_id, *resp);
      });
    });

  this->_wo_states_pub =
    this->create_publisher<endpoints::WorkOrderStatesTopic::MessageType>(
    endpoints::WorkOrderStatesTopic::topic_name(),
    endpoints::WorkOrderStatesTopic::qos());

  this->_get_wo_state_srv =
    this->create_service<endpoints::GetWorkOrderStateService::ServiceType>(
    endpoints::GetWorkOrderStateService::service_name(),
    [this](endpoints::GetWorkOrderStateService::ServiceType::Request::
    ConstSharedPtr req,
    endpoints::GetWorkOrderStateService::ServiceType::Response::SharedPtr resp)
    {
      try
      {
        const auto& job = this->_jobs.at(req->work_order_id);
        resp->work_order_state = this->_get_wo_state(job);
        resp->success = true;
      }
      catch (const std::out_of_range&)
      {
        resp->success = false;
        std::ostringstream oss;
        oss << "Work order [" << req->work_order_id << "] does not exist";
        resp->message = oss.str();
      }
    });

  RCLCPP_INFO(this->get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

auto SystemOrchestrator::on_activate(const rclcpp_lifecycle::State& previous)
-> CallbackReturn
{
  if (!this->_lifecycle_mgr->changeStateForAllNodes(
      Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Lifecycle Manager failed to activate all nodes");
    return CallbackReturn::FAILURE;
  }

  this->_activated = true;
  this->_jobs_timer = this->create_wall_timer(BT_TICK_RATE, [this]()
      {
        this->_spin_bts_once();
      });

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

auto SystemOrchestrator::on_deactivate(const rclcpp_lifecycle::State& previous)
-> CallbackReturn
{
  this->_activated = false;
  this->_jobs_timer.reset();

  RCLCPP_INFO(this->get_logger(), "Halting all jobs");
  this->_halt_fail_and_remove_all_jobs();

  if (!this->_lifecycle_mgr->changeStateForAllNodes(
      Transition::TRANSITION_DEACTIVATE))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Lifecycle Manager failed to deactivate all nodes");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

auto SystemOrchestrator::on_cleanup(const rclcpp_lifecycle::State& previous)
-> CallbackReturn
{
  if (!this->_lifecycle_mgr->changeStateForAllNodes(
      Transition::TRANSITION_CLEANUP))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Lifecycle Manager failed to clean up");
    return CallbackReturn::FAILURE;
  }

  this->_pause_system_srv.reset();
  this->_estop_sub.reset();

  this->_register_transporter_srv.reset();
  this->_list_transporters_srv.reset();
  this->_register_workcell_srv.reset();
  this->_work_order_srv.reset();

  this->_list_workcells_srv.reset();

  this->_jobs.clear();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");

  return CallbackReturn::SUCCESS;
}

BT::Tree SystemOrchestrator::_create_bt(const WorkOrderActionType::Goal& wo,
  std::shared_ptr<Context> ctx)
{
  // we need different context for each bt so we can't reuse the same factory.
  auto bt_factory = std::make_shared<BT::BehaviorTreeFactory>();

  bt_factory->registerNodeType<common::PausableSequence>("PausableSequence");
  bt_factory->registerSimpleAction("IsPauseTriggered",
    [this](BT::TreeNode& bt_node)
    {
      bt_node.setOutput("paused", this->_paused);
      return BT::NodeStatus::SUCCESS;
    }, { BT::OutputPort<bool>("paused") });

  bt_factory->registerBuilder<WorkcellRequest>(
    "WorkcellRequest",
    [this, ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<WorkcellRequest>(name, config, *this, ctx,
      [this, ctx](const auto& task_states)
      {
        // handle feedback internally in system orchestrator
        for (const auto& it : task_states)
        {
          for (const auto& m : it.second.messages)
          {
            RCLCPP_INFO(
              this->get_logger(), "%s: %s", it.first.c_str(), m.c_str());
          }
          for (const auto& em: it.second.error_messages)
          {
            RCLCPP_ERROR(
              this->get_logger(), "%s: %s", it.first.c_str(), em.c_str());
          }
        }

        this->_publish_wo_feedback(*ctx);
        try
        {
          this->_publish_wo_states(this->_jobs.at(ctx->get_job_id()));
        }
        catch (const std::out_of_range&)
        {
          RCLCPP_WARN(this->get_logger(),
          "Error when publishing work order states: Work order [%s] not found",
          ctx->get_job_id().c_str());
        }
      });
    });

  bt_factory->registerBuilder<CreateTransporterTask>("CreateTransporterTask",
    [this, ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<CreateTransporterTask>(name, config,
        this->shared_from_this(), ctx);
    });

  bt_factory->registerBuilder<UnpackTransporterTask>("UnpackTransporterTask",
    [this, ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<UnpackTransporterTask>(name, config,
        this->shared_from_this());
    });

  bt_factory->registerBuilder<BidTransporter>("BidTransporter",
    [this, ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<BidTransporter>(name, config,
      this->shared_from_this(), ctx);
    });

  bt_factory->registerBuilder<TransporterRequest>(
    "TransporterRequest",
    [this, ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<TransporterRequest>(name, config, *this, ctx);
    });

  bt_factory->registerBuilder<ForEachTask>("ForEachTask",
    [this, ctx](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<ForEachTask>(name, config,
      this->get_logger(), ctx);
    });

  bt_factory->registerBuilder<ExecuteTask>("ExecuteTask",
    [this, ctx, bt_factory](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<ExecuteTask>(name, config, ctx, this->_bt_path,
      bt_factory);
    });

  bt_factory->registerBuilder<SendSignal>("SendSignal",
    [ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SendSignal>(name, config, ctx);
    });

  bt_factory->registerBuilder<SignalTransporter>("SignalTransporter",
    [ctx](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SignalTransporter>(name, config, ctx);
    });

  return bt_factory->createTreeFromFile(this->_bt_path / this->_main_bt_filename);
}

void SystemOrchestrator::_create_job(const WorkOrderActionType::Goal& goal)
{
  const std::string& work_order_id = goal.order.work_order_id;
  auto wo =
    YAML::Load(goal.order.work_order).as<common::WorkOrder>();
  auto tasks = this->_parse_wo(work_order_id, wo);

  // using `new` because make_shared does not work with aggregate initializer
  std::shared_ptr<Context> ctx{new Context(*this)};
  ctx->set_job_id(work_order_id)
  .set_work_order(wo)
  .set_tasks(tasks)
  .set_task_remapper(this->_task_remapper)
  .set_workcell_sessions(this->_workcell_sessions)
  .set_transporter_sessions(this->_transporter_sessions);
  auto bt = this->_create_bt(goal, ctx);

  const auto& [it,
    inserted] = this->_jobs.emplace(work_order_id, Job{std::move(bt), ctx});
  if (!inserted)
  {
    auto result = std::make_shared<WorkOrderActionType::Result>();
    result->message = "A work order with the same id is already running!";
    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    throw DuplicatedWorkOrderError{result->message};
  }
}

void SystemOrchestrator::_init_job(
  const std::shared_ptr<WorkOrderGoalHandle> goal_handle)
{
  if (this->_estop_pressed)
  {
    auto result = std::make_shared<WorkOrderAction::ActionType::Result>();
    result->message =
      "Cannot start work orders while emergency stop is active!";
    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto& work_order_id = goal_handle->get_goal()->order.work_order_id;
  if (!this->_jobs.count(work_order_id))
  {
    auto result = std::make_shared<WorkOrderActionType::Result>();
    result->message = "Failed to start job, work order [" + work_order_id +
      "] not associated with any jobs";
    RCLCPP_ERROR_STREAM(this->get_logger(), result->message);
    goal_handle->abort(result);
    return;
  }

  try
  {
    auto& job = this->_jobs.at(work_order_id);
    job.ctx->set_goal_handle(goal_handle);
    job.bt_logging = std::make_unique<common::BtLogging>(
      job.bt,
      this->shared_from_this(),
      this->get_parameter("bt_logging_blocklist").as_string_array());
    job.state = WorkOrderState::STATE_BIDDING;
    _publish_wo_states(job);
    this->_assign_all_tasks(job.ctx->get_tasks(),
      [this, &job, goal_handle,
      work_order_id](const std::unordered_map<std::string,
      std::optional<WorkcellTaskAssignment>>& maybe_task_assignments)
      {
        // We iterate through all the task assignments and exit early if any task
        // was not successfully assigned.
        // This prevents assigning tasks to workcells for a work order that will not
        // be executed.
        for (const auto& [task_id, maybe_assignment] : maybe_task_assignments)
        {
          if (!maybe_assignment.has_value())
          {
            auto result = std::make_shared<ExecuteWorkOrder::Result>();
            result->message = "One or more tasks cannot be performed!";
            RCLCPP_ERROR_STREAM(this->get_logger(), result->message);
            goal_handle->abort(result);
            job.state = WorkOrderState::STATE_FAILED;
            this->_publish_wo_states(job);
            this->_jobs.erase(work_order_id);
            return;
          }
        }
        for (const auto& [task_id, maybe_assignment] : maybe_task_assignments)
        {
          // Task assignments are valid, they have been checked in the previous loop
          job.ctx->set_workcell_task_assignment(
            task_id,
            maybe_assignment->workcell_id);
          auto task_state = TaskState();
          task_state.workcell_id = maybe_assignment->workcell_id;
          task_state.task_id = task_id;
          auto req =
          std::make_shared<endpoints::QueueWorkcellTaskService::ServiceType::Request>();
          req->task_id = task_id;
          try
          {
            // hard to say if this will cause a race condition. We need to queue tasks from
            // work orders in the order that they come in, although the bidding requests are
            // sent asynchronously, there may still be a guarantee that response from later
            // bids will not arrive before the earlier bids if the underlying middleware
            // uses tcp (and that calls from the same client uses the same tcp stream).
            const auto resp =
              this->_workcell_sessions.at(maybe_assignment->workcell_id)
              ->queue_task_client->send_request(req);
            if (!resp->success)
            {
              RCLCPP_ERROR(this->get_logger(),
              "Failed to assign task [%s] to workcell [%s]: %s",
              task_id.c_str(),
              maybe_assignment->workcell_id.c_str(), resp->message.c_str());
              this->_halt_fail_and_remove_job(work_order_id);
              return;
            }
          }
          catch (const common::TimeoutError& e)
          {
            RCLCPP_ERROR(
              this->get_logger(),
              "Failed to assign task [%s] to workcell [%s]: %s",
              task_id.c_str(),
              maybe_assignment->workcell_id.c_str(), e.what());
            this->_halt_fail_and_remove_job(work_order_id);
            return;
          }
          task_state.status = TaskState::STATUS_ASSIGNED;
          job.ctx->set_task_state(task_id, TaskState(task_state));
        }
        job.state = WorkOrderState::STATE_EXECUTING;
        this->_publish_wo_states(job);
        RCLCPP_INFO(this->get_logger(), "Starting work order [%s]",
        job.ctx->get_job_id().c_str());
      });
  }
  catch (const std::exception& e)
  {
    auto result =
      std::make_shared<endpoints::WorkOrderAction::ActionType::Result>();
    result->message =
      (std::ostringstream{} << "Unable to create work order: [" <<
      e.what() << "]").str();
    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    this->_jobs.erase(work_order_id);
    return;
  }
}

std::string SystemOrchestrator::_generate_task_id(
  const std::string& work_order_id,
  const std::string& process_id,
  std::size_t step_index) const
{
  std::stringstream ss;
  ss << work_order_id << "/" << process_id << "/" << step_index;
  return ss.str();
}

std::vector<nexus_orchestrator_msgs::msg::WorkcellTask> SystemOrchestrator::
_parse_wo(const std::string& work_order_id, const common::WorkOrder& work_order)
{
  std::vector<nexus_orchestrator_msgs::msg::WorkcellTask> tasks;
  const auto steps = work_order.steps();
  tasks.reserve(steps.size());
  uint64_t step_index = 0;
  for (const auto& step : steps)
  {
    nexus_orchestrator_msgs::msg::WorkcellTask task;
    task.work_order_id = work_order_id;
    task.task_id =
      this->_generate_task_id(work_order_id, step.process_id(), step_index++);
    task.type = step.process_id();

    for (const auto ii : step.input_items())
    {
      task.input_item_ids.push_back(ii.guid());
    }
    for (const auto oi : step.output_items())
    {
      task.output_item_ids.push_back(oi.guid());
    }

    // Inject metadata into payload for workcell.
    YAML::Node processed_step = step.yaml;
    processed_step["metadata"] =
      work_order.metadata().has_value() ? work_order.metadata().value() :
      YAML::Node{};
    YAML::Emitter out;
    out << processed_step;
    task.payload = out.c_str();

    tasks.emplace_back(task);
  }
  return tasks;
}

void SystemOrchestrator::_handle_wo_cancel(
  const WorkOrderActionType::Goal& goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Cancelling all work orders");
  for (auto& [job_id, job] : this->_jobs)
  {
    RCLCPP_INFO(this->get_logger(), "Cancelling work order [%s]",
      job_id.c_str());
    this->_halt_job(job_id);
    this->_handle_wo_failed(job);
    job.state = WorkOrderState::STATE_CANCELLED;
    this->_publish_wo_states(job);
    RCLCPP_INFO(
      this->get_logger(), "Work order [%s] cancelled successfully",
      job_id.c_str());
  }
  this->_jobs.clear();
}

void SystemOrchestrator::_handle_wo_failed(const Job& job)
{
  std::ostringstream oss;
  for (const auto& e : job.ctx->get_errors())
  {
    oss << "[" << e << "]";
  }
  std::string reasons = oss.str();

  RCLCPP_ERROR_STREAM(
    this->get_logger(), "Failed to execute work order:" << reasons);
  auto result_msg = std::make_shared<WorkOrderActionType::Result>();
  result_msg->message = reasons;
  job.ctx->get_goal_handle()->abort(std::move(result_msg));
}

void SystemOrchestrator::_handle_register_workcell(
  endpoints::RegisterWorkcellService::ServiceType::Request::ConstSharedPtr req,
  endpoints::RegisterWorkcellService::ServiceType::Response::SharedPtr resp)
{
  RCLCPP_DEBUG(this->get_logger(), "Got register workcell request");

  if (this->get_current_state().label() != "active")
  {
    resp->success = false;
    resp->message = "Node must be active to register workcells!";
    resp->error_code =
      endpoints::RegisterWorkcellService::ServiceType::Response::
      ERROR_NOT_READY;
    RCLCPP_WARN(this->get_logger(), "%s", resp->message.c_str());
    return;
  }

  const auto& workcell_id = req->description.workcell_id;
  WorkcellState state;
  state.workcell_id = workcell_id;
  state.work_order_id = "";
  state.status = WorkcellState::STATUS_IDLE;
  const auto added = this->_workcell_sessions.emplace(workcell_id,
    std::make_shared<WorkcellSession>(WorkcellSession{
      req->description,
      std::move(state),
      this->create_client<endpoints::IsTaskDoableService::ServiceType>(
        endpoints::IsTaskDoableService::service_name(
          workcell_id)),
      this->create_client<endpoints::PauseWorkcellService::ServiceType>(
        endpoints::
        PauseWorkcellService::service_name(workcell_id)),
      std::make_unique<common::SyncServiceClient<endpoints::SignalWorkcellService::ServiceType>>(
        this, endpoints::SignalWorkcellService::service_name(workcell_id)),
      std::make_unique<common::SyncServiceClient<endpoints::QueueWorkcellTaskService::ServiceType>>(
        this, endpoints::QueueWorkcellTaskService::service_name(workcell_id)),
      std::make_unique<common::SyncServiceClient<endpoints::RemovePendingTaskService::ServiceType>>(
        this, endpoints::RemovePendingTaskService::service_name(workcell_id))
    }));

  if (!this->_lifecycle_mgr->addNodeName(workcell_id))
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to add workcell %s (state transition failed)",
      workcell_id.c_str());
    resp->success = false;
    resp->message = "state transition failed";
    return;
  }

  resp->success = true;

  std::stringstream ss;
  for (const auto& station : added.first->second->description.io_stations)
  {
    ss << station.name << ",";
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Registered workcell [%s] with stations: [%s]",
    workcell_id.c_str(),
    ss.str().c_str());
}

void SystemOrchestrator::_handle_register_transporter(
  endpoints::RegisterTransporterService::ServiceType::Request::ConstSharedPtr req,
  endpoints::RegisterTransporterService::ServiceType::Response::SharedPtr resp)
{
  RCLCPP_DEBUG(this->get_logger(), "Got register transporter request");

  if (this->get_current_state().label() != "active")
  {
    resp->success = false;
    resp->message = "Node must be active to register transporters!";
    resp->error_code =
      endpoints::RegisterTransporterService::ServiceType::Response::
      ERROR_NOT_READY;
    RCLCPP_ERROR(this->get_logger(), "%s", resp->message.c_str());
    return;
  }

  auto transporter_id = req->description.workcell_id;
  this->_transporter_sessions.emplace(transporter_id,
    std::make_shared<TransporterSession>(TransporterSession{
      req->description,
      this->create_client<endpoints::IsTransporterAvailableService::ServiceType>(
        endpoints::IsTransporterAvailableService::service_name(
          transporter_id)),
      std::make_unique<common::SyncServiceClient<endpoints::SignalTransporterService::ServiceType>>(
        this, endpoints::SignalTransporterService::service_name(transporter_id))
    }));

  resp->success = true;

  RCLCPP_INFO(this->get_logger(), "Registered transporter %s",
    transporter_id.c_str());
}

void SystemOrchestrator::_halt_job(const std::string& job_id)
{
  try
  {
    auto& job = this->_jobs.at(job_id);
    job.bt.haltTree();
    job.ctx->add_error("Work order halted");
    for (const auto& [task_id, wc_id] : job.ctx->get_workcell_task_assignments())
    {
      // should we try to remove all tasks anyway even if they are not pending?
      const auto task_state = job.ctx->get_task_state(task_id);
      if (!task_state.has_value())
      {
        RCLCPP_WARN(
          this->get_logger(), "Failed to remove pending task [%s]: missing task state",
          task_id.c_str());
        continue;
      }
      if ((*task_state).status != TaskState::STATUS_ASSIGNED)
      {
        continue;
      }

      const auto req =
        std::make_shared<endpoints::RemovePendingTaskService::ServiceType::Request>();
      req->task_id = task_id;
      const auto resp =
        this->_workcell_sessions.at(wc_id)->remove_pending_task_client->
        send_request(req);
      if (!resp->success)
      {
        RCLCPP_WARN(
          this->get_logger(), "Failed to remove pending task [%s]: [%s]",
          task_id.c_str(), resp->message.c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Halted job [%s]", job_id.c_str());
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_WARN(
      this->get_logger(), "Failed to halt job [%s]: job does not exist",
      job_id.c_str());
  }
}

void SystemOrchestrator::_halt_fail_and_remove_job(const std::string& job_id)
{
  try
  {
    auto& job = this->_jobs.at(job_id);
    this->_halt_job(job_id);
    job.state = WorkOrderState::STATE_FAILED;
    this->_handle_wo_failed(job);
    this->_jobs.erase(job_id);
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_WARN(
      this->get_logger(), "Failed to halt, fail and remove job [%s]: job does not exist",
      job_id.c_str());
  }
}

void SystemOrchestrator::_halt_fail_and_remove_all_jobs()
{
  for (auto& [job_id, job] : this->_jobs)
  {
    this->_halt_job(job_id);
    job.state = WorkOrderState::STATE_FAILED;
    this->_handle_wo_failed(job);
  }
  this->_jobs.clear();
  RCLCPP_INFO(this->get_logger(), "All jobs have been halted.");
}

void SystemOrchestrator::_spin_bts_once()
{
  std::vector<std::string> completed_jobs;
  bool job_failed = false;
  for (auto& [job_id, job] : this->_jobs)
  {
    if (job.state != WorkOrderState::STATE_EXECUTING)
    {
      continue;
    }

    auto goal_handle = job.ctx->get_goal_handle();
    try
    {
      const auto bt_status = job.bt.tickRoot();
      switch (bt_status)
      {
        case BT::NodeStatus::SUCCESS: {
          RCLCPP_INFO(
            this->get_logger(), "Finished work order [%s]",
            job.ctx->get_job_id().c_str());
          auto result_msg = std::make_shared<WorkOrderActionType::Result>();
          auto report = job.bt_logging->generate_report();
          result_msg->message = common::ReportConverter::to_string(report);
          RCLCPP_INFO(this->get_logger(), "%s", result_msg->message.c_str());
          goal_handle->succeed(std::move(result_msg));
          job.state = WorkOrderState::STATE_SUCCESS;
          this->_publish_wo_states(job);
          completed_jobs.emplace_back(job_id);
          break;
        }
        case BT::NodeStatus::FAILURE: {
          job.state = WorkOrderState::STATE_FAILED;
          this->_handle_wo_failed(job);
          this->_publish_wo_states(job);
          completed_jobs.emplace_back(job_id);
          // TODO(koonpeng): Force stopping a task may lead to unexpected behaviors.
          //   We need custom cancel logic in bt to gracefully stop a task.
          // RCLCPP_WARN(
          //   this->get_logger(), "Cancelling all work orders because work order [%s] failed",
          //   job_id.c_str());
          // job_failed = true;
          break;
        }
        case BT::NodeStatus::RUNNING:
          break;
        default: {
          std::ostringstream oss;
          oss << "Failed to execute work order [" << job_id <<
            "]: unexpected status [" << bt_status << "]";
          job.ctx->add_error(oss.str());
          job.state = WorkOrderState::STATE_FAILED;
          this->_handle_wo_failed(job);
          this->_publish_wo_states(job);
          // TODO(koonpeng): Force stopping a task may lead to unexpected behaviors.
          //   We need custom cancel logic in bt to gracefully stop a task.
          // RCLCPP_WARN(
          //   this->get_logger(), "Cancelling all work orders because work order [%s] failed",
          //   job_id.c_str());
          // job_failed = true;
          break;
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "BT failed with exception: %s",
        e.what());
      this->_handle_wo_failed(job);
      this->_publish_wo_states(job);
      completed_jobs.emplace_back(job_id);
    }
  }

  for (const auto& job_id : completed_jobs)
  {
    this->_jobs.erase(job_id);
  }

  if (job_failed)
  {
    this->_halt_fail_and_remove_all_jobs();
  }
}

bool SystemOrchestrator::_bt_filename_valid(const std::string& bt_filename) const
{
  const auto resolved_bt = this->_bt_path / bt_filename;
  if (!std::filesystem::exists(resolved_bt) ||
    !std::filesystem::is_regular_file(resolved_bt))
  {
    return false;
  }
  return true;
}

void SystemOrchestrator::_assign_workcell_task(const WorkcellTask& task,
  std::function<void(const std::optional<WorkcellTaskAssignment>&)> on_done)
{
  using IsTaskDoableService = endpoints::IsTaskDoableService::ServiceType;

  RCLCPP_INFO(this->get_logger(), "Assigning workcell tasks");
  auto task_assignments = std::make_shared<std::unordered_map<std::string,
      std::string>>();
  std::unordered_map<std::string,
    common::BatchServiceReq<IsTaskDoableService>> batch;
  auto req =
    std::make_shared<endpoints::IsTaskDoableService::ServiceType::Request>();
  req->task = task;
  for (const auto& [wc_id, s] : this->_workcell_sessions)
  {
    batch.emplace(wc_id,
      common::BatchServiceReq<IsTaskDoableService>{s->task_doable_client,
        req});
  }

  common::batch_service_call(this, batch,
    std::chrono::milliseconds{this->_bid_request_timeout},
    [this, on_done, task](const std::unordered_map<std::string,
    common::BatchServiceResult<IsTaskDoableService>>&
    results)
    {
      std::string assigned;
      for (const auto& [wc_id, result] : results)
      {
        if (!result.success)
        {
          RCLCPP_WARN(
            this->get_logger(), "Skipped workcell [%s] (no response)",
            wc_id.c_str());
        }
        if (result.success && result.resp->success)
        {
          // TODO(kp): assign based on some heuristics
          assigned = wc_id;
          break;
        }
      }
      if (assigned.empty())
      {
        RCLCPP_ERROR(this->get_logger(),
        "No workcell is able perform task [%s]", task.task_id.c_str());
        on_done(std::nullopt);
      }
      else
      {
        RCLCPP_INFO(
          this->get_logger(), "Task [%s] assigned to workcell [%s]",
          task.task_id.c_str(), assigned.c_str());
        on_done(WorkcellTaskAssignment{
          task.task_id, assigned, result->resp->inputs, result->resp->outputs});
      }
    });
}

void SystemOrchestrator::_assign_all_tasks(
  const std::vector<WorkcellTask>& tasks,
  std::function<void(const std::unordered_map<std::string,
  std::optional<WorkcellTaskAssignment>>&)> on_done)
{
  auto num_tasks = tasks.size();
  auto task_assignments = std::make_shared<std::unordered_map<std::string,
      std::optional<WorkcellTaskAssignment>>>();
  for (const auto& task : tasks)
  {
    this->_assign_workcell_task(task,
      [on_done, num_tasks, task_assignments, task](
        const std::optional<WorkcellTaskAssignment>& assigned)
      {
        task_assignments->emplace(task.task_id, assigned);
        if (task_assignments->size() == num_tasks)
        {
          on_done(*task_assignments);
        }
      });
  }
}

auto SystemOrchestrator::_get_wo_state(const Job& job) const -> WorkOrderState
{
  WorkOrderState state;
  state.id = job.ctx->get_job_id();
  state.state = job.state;
  const auto task_states = job.ctx->get_task_states();
  state.task_states.reserve(task_states.size());
  for (const auto& [_, task_state] : task_states)
  {
    state.task_states.emplace_back(task_state);
  }
  return state;
}

auto SystemOrchestrator::_get_current_wo_states() const -> std::vector<WorkOrderState>
{
  std::vector<WorkOrderState> wo_states;
  wo_states.reserve(this->_jobs.size());
  for (const auto& [_, job] : this->_jobs)
  {
    wo_states.emplace_back(this->_get_wo_state(job));
  }
  return wo_states;
}

void SystemOrchestrator::_publish_wo_feedback(const Context& ctx)
{
  auto wo_feedback =
    std::make_shared<endpoints::WorkOrderAction::ActionType::Feedback>();
  const auto task_states = ctx.get_task_states();
  wo_feedback->task_states.reserve(task_states.size());
  for (const auto& [_, state] : task_states)
  {
    wo_feedback->task_states.emplace_back(state);
  }
  ctx.get_goal_handle()->publish_feedback(wo_feedback);
}

void SystemOrchestrator::_publish_wo_states(const Job& job)
{
  const auto wo_state = this->_get_wo_state(job);
  this->_wo_states_pub->publish(wo_state);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus::system_orchestrator::SystemOrchestrator)
