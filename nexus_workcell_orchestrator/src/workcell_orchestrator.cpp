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

#include "workcell_orchestrator.hpp"

#include "exceptions.hpp"
#include "get_joint_constraints.hpp"
#include "get_result.hpp"
#include "make_transform.hpp"
#include "serialization.hpp"
#include "set_result.hpp"
#include "signals.hpp"
#include "transform_pose.hpp"

#include <nexus_capabilities/context.hpp>
#include <nexus_capabilities/exceptions.hpp>
#include <nexus_capabilities/utils.hpp>
#include <nexus_common/logging.hpp>
#include <nexus_common/pausable_sequence.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_description.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <yaml-cpp/exceptions.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace nexus::workcell_orchestrator {

using TaskState = nexus_orchestrator_msgs::msg::TaskState;
using WorkcellRequest = endpoints::WorkcellRequestAction::ActionType;

using rcl_interfaces::msg::ParameterDescriptor;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

static constexpr size_t MAX_PARALLEL_TASK = 10;
static constexpr std::chrono::milliseconds BT_TICK_RATE{10};
static constexpr std::chrono::seconds REGISTER_TICK_RATE{1};

WorkcellOrchestrator::WorkcellOrchestrator(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("workcell_orchestrator", options),
  _capability_loader("nexus_capabilities", "nexus::Capability"),
  _task_checker_loader("nexus_capabilities", "nexus::TaskChecker")
{
  common::configure_logging(this);

  std::vector<std::string> caps;
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Capabilities supported by the workcell, refer to the README to find the list of capabilities supported.";
    caps = this->declare_parameter("capabilities", caps, desc);
  }
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Path to a directory containing behavior trees to use to perform tasks. Each file in the directory should be a behavior tree xml, the file name denotes the task type for that behavior tree.";
    auto param = this->declare_parameter<std::string>("bt_path", "", desc);
    if (param.empty())
    {
      throw std::runtime_error("param [bt_path] is required");
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
      "Name of hardware nodes that will be managed by the workcell orchestrator lifecycle manager.";
    this->declare_parameter("hardware_nodes", std::vector<std::string>{}, desc);
  }

  for (const auto& cap_id : caps)
  {
    try
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Attempting to load capability [%s]...",
        cap_id.c_str()
      );
      auto cap = _capability_loader.createUniqueInstance(cap_id);
      cap->declare_params(*this);
      this->_capabilities.emplace(cap_id, std::move(cap));
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load capability plugin [%s]. Detailed error: %s",
        cap_id.c_str(),
        e.what()
      );
      continue;
    }
  }
  if (this->_capabilities.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Workcell can only handle generic tasks because it has no capabilities.");
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Fully qualified plugin name for the TaskChecker.";
    auto param = this->declare_parameter<std::string>(
      "task_checker_plugin", "nexus::task_checkers::FilepathChecker", desc);
    if (param.empty())
    {
      throw std::runtime_error("param [task_checker_plugin] is required.");
    }
  }

  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Max number of parallel work orders to execute at once, set to 0 to make "
      "it unlimited.";
    this->declare_parameter("max_jobs", 0, desc);
    this->_max_parallel_jobs = this->get_parameter("max_jobs").as_int();
  }

  this->_register_workcell_client =
    this->create_client<endpoints::RegisterWorkcellService::ServiceType>(
    endpoints::RegisterWorkcellService::service_name());

  this->_lifecycle_mgr =
    std::make_unique<lifecycle_manager::LifecycleManager<>>(
    this->get_name(), std::vector<std::string>{});

  // Add each hardware node to lifecycle manager
  auto hardware_nodes = this->get_parameter("hardware_nodes").as_string_array();
  if (hardware_nodes.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "No hardware nodes added to this workcell.");
  }
  else
  {
    for (const auto& n : hardware_nodes)
    {
      if (n.empty())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Hardware node name cannot be empty. Ignoring..."
        );
        continue;
      }
      if (!this->_lifecycle_mgr->addNodeName(n))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to add node %s (state transition failed)", n.c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
          "Added node %s to workcell lifecycle manager.", n.c_str());
      }
    }
  }

  this->_register_timer = this->create_wall_timer(REGISTER_TICK_RATE,
      [this]()
      {
        this->_register();
      });
}

auto WorkcellOrchestrator::on_configure(
  const rclcpp_lifecycle::State& previous_state) -> CallbackReturn
{
  auto result = this->_configure(previous_state);
  if (result != CallbackReturn::SUCCESS)
  {
    if (this->on_cleanup(previous_state) != CallbackReturn::SUCCESS)
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "FATAL ERROR! Node failed to configure and failed to cleanup, it is now in a undefined state");
    }
  }
  return result;
}

auto WorkcellOrchestrator::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Workcell activated");
  this->_bt_timer = this->create_wall_timer(BT_TICK_RATE, [this]()
      {
        this->_tick_all_bts();
      });
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(),
    "Cancelling ongoing task before deactivating workcell");
  this->_cancel_all_tasks();

  this->_bt_timer->cancel();
  this->_bt_timer.reset();

  RCLCPP_INFO(this->get_logger(), "Workcell deactivated");
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_cleanup(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  this->_bt_factory.reset();
  this->_tf2_listener.reset();
  this->_tf2_buffer.reset();
  this->_remove_pending_task_srv.reset();
  this->_queue_task_srv.reset();
  this->_pause_srv.reset();
  this->_signal_wc_srv.reset();
  this->_task_checker.reset();
  this->_task_doable_srv.reset();
  this->_cmd_server.reset();
  this->_ctx_mgr.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::_configure(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  this->_ctx_mgr = std::make_shared<ContextManager>();
  this->_bt_store.on_register_handlers.emplace_back([this](const std::
    string& key, const std::filesystem::path& filepath)
    {
      RCLCPP_INFO(
        this->get_logger(), "Registered behavior tree: %s (%s)",
        key.c_str(),
        filepath.c_str());
    });
  auto bt_path = this->get_parameter("bt_path").get_value<std::string>();
  this->_bt_store.register_from_path(bt_path);

  try
  {
    const std::string plugin_name = this->get_parameter("task_checker_plugin").get_value<std::string>();
    RCLCPP_INFO(
      this->get_logger(),
      "Loading TaskChecker plugin [ %s ].",
      plugin_name.c_str()
    );
    _task_checker = _task_checker_loader.createSharedInstance(plugin_name);
    _task_checker->initialize(*this);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load TaskChecker plugin. Error: %s\n", ex.what());
    return CallbackReturn::FAILURE;
  }

  // create workcell request action server
  this->_cmd_server =
    rclcpp_action::create_server<endpoints::WorkcellRequestAction::ActionType>(
    this,
    endpoints::WorkcellRequestAction::action_name(this->get_name()),
    // handle goal callback
    [this](const rclcpp_action::GoalUUID& /* uuid */,
    endpoints::WorkcellRequestAction::ActionType::Goal::ConstSharedPtr goal)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got workcell task request");

      if (this->_max_parallel_jobs > 0 &&
      this->_ctxs.size() >= static_cast<size_t>(this->_max_parallel_jobs))
      {
        auto result = std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
        result->message = "Max number of parallel work orders reached";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        RCLCPP_ERROR(this->get_logger(), "%d", static_cast<int>(this->_ctxs.size()));
        return rclcpp_action::GoalResponse::REJECT;
      }

      const auto it =
      std::find_if(this->_ctxs.cbegin(), this->_ctxs.cend(),
      [&goal](const std::shared_ptr<Context>& ctx)
      {
        return ctx->task.task_id == goal->task.task_id;
      });
      if (it != this->_ctxs.cend() && (*it)->goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(),
        "A task with the same id is already executing");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // handle cancel callback
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>
    goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got cancel request");
      const auto& goal = goal_handle->get_goal();
      auto it =
      std::find_if(this->_ctxs.begin(), this->_ctxs.end(),
      [&goal](const std::shared_ptr<Context>& ctx)
      {
        return goal->task.task_id == ctx->task.task_id;
      });

      if (it == this->_ctxs.end())
      {
        RCLCPP_WARN(this->get_logger(),
        "Fail to cancel task [%s]: task does not exist", goal->task.task_id.c_str());
      }
      else
      {
        // we can just remove a task that is not running
        if ((*it)->task_state.status != TaskState::STATUS_RUNNING)
        {
          this->_ctxs.erase(it);
        }
      }
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // handle accepted callback
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>
    goal_handle)
    {
      if (this->get_current_state().label() != "active")
      {
        RCLCPP_ERROR(this->get_logger(),
        "Can only process tasks when node is active!");
        goal_handle->abort(
          std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>());
        return;
      }

      std::shared_ptr<Context> ctx = nullptr;
      const auto it =
      std::find_if(this->_ctxs.begin(), this->_ctxs.end(),
      [&goal_handle](const std::shared_ptr<Context>& ctx)
      {
        return ctx->task.task_id == goal_handle->get_goal()->task.task_id;
      });
      if (it != this->_ctxs.end())
      {
        ctx = *it;
      }
      else
      {
        ctx = std::make_shared<Context>(*this);
      }
      ctx->goal_handle = goal_handle;
      const auto goal = goal_handle->get_goal();
      try
      {
        ctx->task = this->_task_parser.parse_task(goal->task);
        if (!this->_task_checker->is_task_doable(
          ctx->task))
        {
          auto result = std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
          result->message = "Workcell cannot perform task " + ctx->task.type;
          result->success = false;
          RCLCPP_ERROR_STREAM(this->get_logger(), result->message);
          goal_handle->abort(
            result);
          return;
        }
        ctx->task_state.workcell_id = this->get_name();
        ctx->task_state.task_id = ctx->task.task_id;
        ctx->bt = this->_create_bt(ctx);
        ctx->bt_logging = std::make_unique<common::BtLogging>(ctx->bt,
        this->shared_from_this());
      }
      catch (const std::exception& e)
      {
        std::ostringstream oss;
        auto result = std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
        oss << "Failed to create task: " << e.what();
        result->message = oss.str();
        result->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), result->message);
        goal_handle->abort(result);
        // make sure to clear previously queued task
        this->_ctxs.erase(it);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Queuing task [%s]",
      goal->task.task_id.c_str());
      ctx->task_state.status = TaskState::STATUS_QUEUED;
      auto fb = std::make_shared<WorkcellRequest::Feedback>();
      fb->state = ctx->task_state;
      goal_handle->publish_feedback(fb);

      if (it == this->_ctxs.end())
      {
        this->_ctxs.emplace_back(ctx);
      }
    });

  this->_wc_state_pub =
    this->create_publisher<endpoints::WorkcellStateTopic::MessageType>(endpoints::WorkcellStateTopic::topic_name(
        this->get_name()), endpoints::WorkcellStateTopic::qos());
  this->_cur_state = WorkcellState();
  this->_cur_state.workcell_id = this->get_name();
  this->_cur_state.work_order_id = "";
  this->_cur_state.status = WorkcellState::STATUS_IDLE;
  this->_wc_state_pub->publish(this->_cur_state);

  // create task doable server
  this->_task_doable_srv =
    this->create_service<endpoints::IsTaskDoableService::ServiceType>(
    endpoints::IsTaskDoableService::service_name(this->get_name()),
    [this](endpoints::IsTaskDoableService::ServiceType::Request::ConstSharedPtr
    req,
    endpoints::IsTaskDoableService::ServiceType::Response::SharedPtr resp)
    {
      this->_handle_task_doable(req, resp);
    });

  this->_signal_wc_srv =
    this->create_service<endpoints::SignalWorkcellService::ServiceType>(endpoints::SignalWorkcellService::service_name(
        this->get_name()),
      [this](endpoints::SignalWorkcellService::ServiceType::Request::
      ConstSharedPtr req,
      endpoints::SignalWorkcellService::ServiceType::Response::SharedPtr resp)
      {
        this->_process_signal(req, resp);
      });

  // create pause service
  this->_pause_srv =
    this->create_service<endpoints::PauseWorkcellService::ServiceType>(endpoints::PauseWorkcellService::service_name(
        this->get_name()),
      [this](endpoints::PauseWorkcellService::ServiceType::Request::
      ConstSharedPtr req,
      endpoints::PauseWorkcellService::ServiceType::Response::SharedPtr resp)
      {
        this->_paused = req->pause;
        resp->success = true;
        std::string verb = req->pause ? "paused" : "unpaused";
        RCLCPP_INFO(this->get_logger(), "workcell %s", verb.c_str());
      });

  this->_queue_task_srv =
    this->create_service<endpoints::QueueWorkcellTaskService::ServiceType>(endpoints::QueueWorkcellTaskService::service_name(
        this->get_name()),
      [this](endpoints::QueueWorkcellTaskService::ServiceType::Request::
      ConstSharedPtr req,
      endpoints::QueueWorkcellTaskService::ServiceType::Response::SharedPtr resp)
      {
        const auto it =
        std::find_if(this->_ctxs.begin(), this->_ctxs.end(),
        [&req](const std::shared_ptr<Context>& ctx)
        {
          return ctx->task.task_id == req->task_id;
        });
        if (it != this->_ctxs.end())
        {
          resp->success = false;
          resp->message = "A task with the same id already exists";
          RCLCPP_INFO(this->get_logger(), "Failed to queue task [%s]: %s",
          req->task_id.c_str(), resp->message.c_str());
          return;
        }
        const auto& ctx =
        this->_ctxs.emplace_back(std::make_shared<Context>(*this));
        ctx->task.task_id = req->task_id;
        ctx->task_state.task_id = req->task_id;
        ctx->task_state.workcell_id = this->get_name();
        ctx->task_state.status = TaskState::STATUS_ASSIGNED;
        resp->success = true;
        RCLCPP_INFO(this->get_logger(), "queued task %s", ctx->task.task_id.c_str());
      });

  this->_remove_pending_task_srv =
    this->create_service<endpoints::RemovePendingTaskService::ServiceType>(endpoints::RemovePendingTaskService::service_name(
        this->get_name()),
      [this](endpoints::RemovePendingTaskService::ServiceType::Request::
      ConstSharedPtr req,
      endpoints::RemovePendingTaskService::ServiceType::Response::SharedPtr resp)
      {
        RCLCPP_DEBUG(this->get_logger(),
        "received request to remove pending task [%s]", req->task_id.c_str());
        const auto it =
        std::find_if(this->_ctxs.begin(), this->_ctxs.end(),
        [&req](const std::shared_ptr<Context>& ctx)
        {
          return ctx->task.task_id == req->task_id;
        });
        if (it == this->_ctxs.end() ||
        (*it)->task_state.status != TaskState::STATUS_ASSIGNED)
        {
          resp->success = false;
          resp->message = "task does not exist or is not pending";
          RCLCPP_DEBUG_STREAM(this->get_logger(), resp->message);
          return;
        }
        this->_ctxs.erase(it);
        RCLCPP_INFO(this->get_logger(), "removed task [%s]",
        req->task_id.c_str());
        resp->success = true;
      });

  this->_tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->_tf2_listener = std::make_unique<tf2_ros::TransformListener>(
    *this->_tf2_buffer, this);

  // Instantiate the BTFactory
  _bt_factory = std::make_unique<BT::BehaviorTreeFactory>();

  // create IsPauseTriggered BT node, it is a simple action node with output port "paused".
  this->_bt_factory->registerSimpleAction("IsPauseTriggered",
    [this](BT::TreeNode& node)
    {
      node.setOutput("paused", this->_paused.load());
      return BT::NodeStatus::SUCCESS;
    }, { BT::OutputPort<bool>("paused") });

  // create a BT node to convert a string into a vector of strings.
  this->_bt_factory->registerSimpleAction("StringToVector",
    [this](BT::TreeNode& node)
    {
      auto maybe_string = node.getInput<std::string>("string");
      if (!maybe_string.has_value())
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "StringToVector missing required input port [string]."
        );
        return BT::NodeStatus::FAILURE;
      }
      std::string str = maybe_string.value();
      if (str.empty())
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "StringToVector input port [string] cannot be empty."
        );
        return BT::NodeStatus::FAILURE;
      }
      auto maybe_delimiter = node.getInput<std::string>("delimiter");
      // Default to comma delimiter if not provided.
      const std::string delimiter = maybe_delimiter.has_value() ?
      maybe_delimiter.value() : ",";

      // Split input string based on the delimiter.
      std::size_t pos = 0;
      std::vector<std::string> strings;
      while ((pos = str.find(delimiter)) != std::string::npos)
      {
        strings.push_back(str.substr(0, pos));
        str.erase(0, pos + delimiter.length());
      }
      node.setOutput("strings", strings);
      return BT::NodeStatus::SUCCESS;
    },
    {
      BT::InputPort<std::string>("string"),
      BT::InputPort<std::string>("delimiter"),
      BT::OutputPort<std::vector<string>>("strings")
    }
  );

  // register PausableSequence
  this->_bt_factory->registerNodeType<common::PausableSequence>(
    "PausableSequence");

  this->_bt_factory->registerBuilder<SetResult>("SetResult",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetResult>(name, config, this->_ctx_mgr);
    });
  this->_bt_factory->registerBuilder<GetResult>("GetResult",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetResult>(name, config, this->_ctx_mgr, *this);
    });

  this->_bt_factory->registerNodeType<MakeTransform>("MakeTransform");
  this->_bt_factory->registerBuilder<ApplyTransform>(
    "ApplyTransform",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<ApplyTransform>(name, config, *this);
    });
  this->_bt_factory->registerBuilder<GetTransform>(
    "GetTransform",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetTransform>(name, config, *this,
      this->_tf2_buffer);
    });

  this->_bt_factory->registerBuilder<SerializeDetections>(
    "SerializeDetections",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SerializeDetections>(name, config, *this);
    });
  this->_bt_factory->registerBuilder<DeserializeDetections>(
    "DeserializeDetections",
    [this](
      const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<DeserializeDetections>(name, config, *this);
    });

  this->_bt_factory->registerBuilder<WaitForSignal>("WaitForSignal",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<WaitForSignal>(name, config, this->_ctx_mgr);
    });

  this->_bt_factory->registerBuilder<SetSignal>("SetSignal",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetSignal>(name, config, this->_ctx_mgr);
    });

  this->_bt_factory->registerBuilder<GetJointConstraints>(
    "GetJointConstraints",
    [me = this->weak_from_this()](
      const std::string& name,
      const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetJointConstraints>(
        name,
        config,
        me
      );
    });

  // create capabilities
  if (this->_capabilities.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Workcell can only handle generic tasks because it has no capabilities.");
  }

  // configure capabilities
  try
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Configuring generic capabilities..."
    );
    for (auto& [cap_id, cap] : this->_capabilities)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "configuring capability [%s]",
        cap_id.c_str()
      );
      cap->configure(
        this->shared_from_this(), this->_ctx_mgr, *_bt_factory);
    }
  }
  catch (const CapabilityError& e)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to configure capability (%s)", e.what());
    return CallbackReturn::FAILURE;
  }

  // Create map for remapping capabilities
  auto remap_caps = this->get_parameter("remap_task_types").as_string();

  try
  {
    YAML::Node node = YAML::Load(remap_caps);
    this->_task_remapper = std::make_shared<common::TaskRemapper>(node);
  }
  catch (YAML::ParserException& e)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to parse remap_task_types parameter: (%s)",
      e.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Workcell configured");
  return CallbackReturn::SUCCESS;
}

void WorkcellOrchestrator::_tick_bt(const std::shared_ptr<Context>& ctx)
{
  if (ctx->task_state.status == TaskState::STATUS_FINISHED ||
    ctx->task_state.status == TaskState::STATUS_FAILED)
  {
    RCLCPP_WARN(
      this->get_logger(), "Not executing task [%s] that is already finished",
      ctx->task.task_id.c_str());
    return;
  }

  this->_ctx_mgr->set_active_context(ctx);
  const auto& goal_handle = ctx->goal_handle;
  ++ctx->tick_count;

  if (ctx->task_state.status == TaskState::STATUS_ASSIGNED)
  {
    // only print on first tick
    if (ctx->tick_count == 1)
    {
      RCLCPP_INFO(
        this->get_logger(), "Task [%s] is pending, call \"%s\" to start this task",
        ctx->task.task_id.c_str(),
        endpoints::WorkcellRequestAction::action_name(this->get_name()).c_str());
    }
    return;
  }

  if (ctx->task_state.status == TaskState::STATUS_QUEUED)
  {
    RCLCPP_INFO(this->get_logger(), "Starting task [%s]", ctx->task.task_id.c_str());
    ctx->task_state.status = TaskState::STATUS_RUNNING;
    auto fb = std::make_shared<WorkcellRequest::Feedback>();
    fb->state = ctx->task_state;
    goal_handle->publish_feedback(fb);
  }

  if (ctx->task_state.status == TaskState::STATUS_RUNNING)
  {
    try
    {
      switch (ctx->bt.tickRoot())
      {
        case BT::NodeStatus::RUNNING:
          return;
        case BT::NodeStatus::SUCCESS: {
          this->_handle_command_success(ctx);
          return;
        }
        case BT::NodeStatus::FAILURE: {
          this->_handle_command_failed(ctx);
          return;
        }
        default: {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed executing task [%s]: unexpected BT status",
            ctx->task.task_id.c_str());
          this->_handle_command_failed(ctx);
          return;
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "BT failed with exception: %s",
        e.what());
      ctx->bt.haltTree();
      this->_handle_command_failed(ctx);
      return;
    }
  }

  RCLCPP_WARN(
    this->get_logger(), "Failed executing task [%s]: unexpected status [%u]",
    ctx->task.task_id.c_str(), ctx->task_state.status);
  ctx->bt.haltTree();
  this->_handle_command_failed(ctx);
  return;
}

void WorkcellOrchestrator::_tick_all_bts()
{
  // cancelling an ongoing task may leave the workcell in a undetermined state
  // so we  must cancel all tasks when an ongoing task is cancelled.
  for (const auto& ctx : this->_ctxs)
  {
    if (ctx->task_state.status == TaskState::STATUS_RUNNING &&
      ctx->goal_handle->is_canceling())
    {
      RCLCPP_WARN(
        this->get_logger(), "Cancelling all tasks because an ongoing task [%s] is being cancelled",
        ctx->task.task_id.c_str());
      // NOTE: iterators are invalidated, it is important to
      // not access them after this line.
      this->_cancel_all_tasks();
      break;
    }
  }

  size_t count = 0;
  auto it = this->_ctxs.begin();
  for (; count < MAX_PARALLEL_TASK && it != this->_ctxs.end();
    ++count, ++it)
  {
    this->_tick_bt(*it);
    const auto task_status = (*it)->task_state.status;
    // `_tick_bt` returns true if the task is finished
    if (task_status == TaskState::STATUS_FINISHED ||
      task_status == TaskState::STATUS_FAILED)
    {
      // NOTE: iterator is invalidated, it is important to
      // not access it after this line.
      this->_ctxs.erase(it);
    }
    // cancel all other tasks when any task fails. note that the failing task is
    // removed from the list first because we don't want to cancel an already
    // failed task.
    if (task_status == TaskState::STATUS_FAILED)
    {
      this->_cancel_all_tasks();
      break;
    }
  }

  auto new_state = this->_cur_state;
  // check if status has changed and publish new state
  if (this->_ctxs.size() > 0)
  {
    new_state.status = WorkcellState::STATUS_BUSY;
    new_state.work_order_id = this->_ctxs.front()->task.work_order_id;
    new_state.task_id = this->_ctxs.front()->task.task_id;
  }
  else if (this->_ctxs.size() == 0)
  {
    new_state.status = WorkcellState::STATUS_IDLE;
    new_state.work_order_id = "";
    new_state.task_id = "";
  }
  if (new_state.work_order_id != this->_cur_state.work_order_id ||
    new_state.task_id != this->_cur_state.task_id ||
    new_state.status != this->_cur_state.status)
  {
    this->_cur_state = new_state;
    this->_wc_state_pub->publish(this->_cur_state);
  }
}

void WorkcellOrchestrator::_cancel_all_tasks()
{
  for (const auto& ctx: this->_ctxs)
  {
    RCLCPP_INFO(this->get_logger(), "Canceling task [%s]",
      ctx->task.task_id.c_str());
    this->_ctx_mgr->set_active_context(ctx);
    ctx->bt.haltTree();
    this->_handle_command_failed(ctx);
  }
  this->_ctxs.clear();
  return;
}

void WorkcellOrchestrator::_register()
{
  if (!this->_lifecycle_mgr->changeStateForAllNodes(
      Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Hardware nodes transition failed");
    return;
  }

  if (this->_ongoing_register)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to register: No response from system orchestrator.");
    if (!this->_register_workcell_client->remove_pending_request(*this->
      _ongoing_register))
    {
      RCLCPP_WARN(this->get_logger(),
        "Unable to remove pending request during workcell registration.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Registering with system orchestrator");
  auto register_cb =
    [this](rclcpp::Client<endpoints::RegisterWorkcellService::ServiceType>::
      SharedFuture future)
    {
      this->_ongoing_register = std::nullopt;
      auto resp = future.get();
      if (!resp->success)
      {
        switch (resp->error_code)
        {
          case endpoints::RegisterWorkcellService::ServiceType::Response::
            ERROR_NOT_READY:
            RCLCPP_ERROR(
              this->get_logger(),
              "Error while registering with system orchestrator, retrying again... [%s]",
              resp->message.c_str());
            break;
          default:
            RCLCPP_FATAL(this->get_logger(),
              "Failed to register with system orchestrator! [%s]",
              resp->message.c_str());
            throw RegistrationError(resp->message, resp->error_code);
        }
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "Successfully registered with system orchestrator");
      this->_register_timer->cancel();
      // TODO(luca) reintroduce once https://github.com/ros2/rclcpp/issues/2652 is fixed and released
      // this->_register_timer.reset();
    };

  if (!this->_register_workcell_client->wait_for_service(
      std::chrono::seconds{0}))
  {
    std::string msg = "Could not find system orchestrator";
    auto secs = std::chrono::seconds(REGISTER_TICK_RATE).count();
    RCLCPP_ERROR(
      this->get_logger(), "Failed to register [%s], retrying in %ld secs",
      msg.c_str(), secs);
    // timer is not canceled so it will run again.
    return;
  }

  auto req =
    std::make_shared<endpoints::RegisterWorkcellService::ServiceType::Request>();
  std::vector<std::string> caps;
  caps.reserve(this->_capabilities.size());
  for (const auto& [k, _] : this->_capabilities)
  {
    caps.emplace_back(k);
  }
  req->description.capabilities = caps;
  req->description.workcell_id = this->get_name();
  this->_ongoing_register = this->_register_workcell_client->async_send_request(
    req,
    register_cb);
}

void WorkcellOrchestrator::_process_signal(
  endpoints::SignalWorkcellService::ServiceType::Request::ConstSharedPtr req,
  endpoints::SignalWorkcellService::ServiceType::Response::SharedPtr resp)
{
  for (const auto& ctx : this->_ctxs)
  {
    if (ctx->task.task_id == req->task_id)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Received signal [%s] for task [%s] from system orchestrator",
        req->signal.c_str(), req->task_id.c_str());
      ctx->signals.emplace(req->signal);
      resp->success = true;
      return;
    }
  }

  std::ostringstream oss;
  oss << "Received signal [" << req->signal << "] for task [" << req->task_id <<
    "] which is not currently running.";
  resp->message = oss.str();
  RCLCPP_WARN_STREAM(this->get_logger(), resp->message);
  resp->success = false;
}

BT::Tree WorkcellOrchestrator::_create_bt(const std::shared_ptr<Context>& ctx)
{
  // To keep things simple, the task type is used as the key for the behavior tree to use.
  this->_ctx_mgr->set_active_context(ctx);
  const auto new_task = this->_task_remapper->remap(ctx->task.type);
  auto bt_name = ctx->task.type;
  if (new_task.has_value())
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Loading remapped BT [%s] for original task type [%s]",
      new_task.value().c_str(),
      ctx->task.type.c_str()
    );
    bt_name = new_task.value();
  }
  return this->_bt_factory->createTreeFromFile(this->_bt_store.get_bt(
        bt_name));
}

void WorkcellOrchestrator::_handle_command_success(
  const std::shared_ptr<Context>& ctx)
{
  RCLCPP_INFO(this->get_logger(), "Task finished successfully");
  ctx->task_state.status = TaskState::STATUS_FINISHED;
  auto result =
    std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
  result->success = true;
  result->result = YAML::Dump(ctx->task.previous_results);
  auto report = ctx->bt_logging->generate_report();
  result->message = common::ReportConverter::to_string(report);
  RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
  ctx->goal_handle->succeed(result);
}

void WorkcellOrchestrator::_handle_command_failed(
  const std::shared_ptr<Context>& ctx)
{
  RCLCPP_ERROR(this->get_logger(), "Task failed");
  ctx->task_state.status = TaskState::STATUS_FAILED;
  // Publish feedback.
  auto fb = std::make_shared<WorkcellRequest::Feedback>();
  fb->state = ctx->task_state;
  ctx->goal_handle->publish_feedback(std::move(fb));
  // Abort the action request.
  auto result =
    std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
  ctx->goal_handle->abort(result);
}

void WorkcellOrchestrator::_handle_task_doable(
  endpoints::IsTaskDoableService::ServiceType::Request::ConstSharedPtr req,
  endpoints::IsTaskDoableService::ServiceType::Response::SharedPtr resp)
{
  RCLCPP_DEBUG(this->get_logger(), "Got request to check task doable");
  try
  {
    auto task = this->_task_parser.parse_task(req->task);
    resp->success = this->_task_checker->is_task_doable(task);
    if (resp->success)
    {
      RCLCPP_DEBUG(this->get_logger(), "Workcell can perform task");
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Workcell cannot perform task");
    }
  }
  catch (const YAML::Exception& e)
  {
    // parsing failed, just assume that we can't perform the task.
    RCLCPP_ERROR(this->get_logger(), "failed to parse request (%s)", e.what());
    resp->success = false;
    return;
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  nexus::workcell_orchestrator::WorkcellOrchestrator)
