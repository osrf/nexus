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

#include "exit_code.hpp"
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

static constexpr size_t MAX_PARALLEL_TASK = 1;
static constexpr std::chrono::milliseconds BT_TICK_RATE{10};
static constexpr std::chrono::seconds REGISTER_TICK_RATE{1};

WorkcellOrchestrator::WorkcellOrchestrator(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("workcell_orchestrator", options),
  _capability_loader("nexus_capabilities", "nexus::Capability")
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
      RCLCPP_FATAL(this->get_logger(), "param [bt_path] is required");
      std::exit(EXIT_CODE_INVALID_PARAMETER);
    }
    this->_bt_path = std::move(param);
  }
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "A yaml containing a dictionary of task types and an array of remaps.";
    const auto yaml = this->declare_parameter("remap_task_types", std::string{},
        desc);
    const auto remaps = YAML::Load(yaml);
    for (const auto& n : remaps)
    {
      const auto task_type = n.first.as<std::string>();
      const auto& mappings = n.second;
      for (const auto& m : mappings)
      {
        this->_task_remaps.emplace(m.as<std::string>(), task_type);
      }
    }
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
        this->_job_mgr.value().tick().value();
      });
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(),
    "Halting ongoing task before deactivating workcell");
  this->_job_mgr->halt_all_jobs().value();

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
  this->_task_doable_srv.reset();
  this->_cmd_server.reset();
  this->_job_mgr->halt_all_jobs().value();
  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::_configure(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  this->_job_mgr = JobManager(this->shared_from_this(), 1);
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

  // create workcell request action server
  this->_cmd_server =
    rclcpp_action::create_server<endpoints::WorkcellRequestAction::ActionType>(
    this,
    endpoints::WorkcellRequestAction::action_name(this->get_name()),
    [this](const rclcpp_action::GoalUUID& /* uuid */,
    endpoints::WorkcellRequestAction::ActionType::Goal::ConstSharedPtr goal)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got workcell task request");
      const auto& jobs = this->_job_mgr->jobs();
      const auto it =
      std::find_if(jobs.begin(), jobs.end(), [&goal](const Job& j)
      {
        return j.task_state.task_id == goal->task.id;
      });
      if (it == jobs.end())
      {
        RCLCPP_ERROR(this->get_logger(),
        "Task [%s] not assigned to this workcell", goal->task.id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>
    goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got cancel request");
      const auto& goal = goal_handle->get_goal();

      if (!goal->task.id.empty())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Cancelling specific task is no longer supported, all tasks will be cancelled together to ensure line consistency");
      }

      this->_job_mgr->halt_all_jobs().value();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
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

      auto ctx = std::make_shared<Context>(this->shared_from_this());
      auto task_result =
      this->_task_parser.parse_task(goal_handle->get_goal()->task);
      if (task_result.error())
      {
        auto result = std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
        result->success = false;
        result->message = task_result.error()->what();
        goal_handle->abort(result);
        return;
      }
      ctx->task = task_result.value();

      auto bt = this->_create_bt(ctx);
      auto job_result =
      this->_job_mgr->queue_task(goal_handle, ctx, std::move(bt));
      if (job_result.error())
      {
        auto result = std::make_shared<endpoints::WorkcellRequestAction::ActionType::Result>();
        result->success = false;
        result->message = job_result.error()->what();
        goal_handle->abort(result);
        return;
      }
    });

  this->_wc_state_pub =
    this->create_publisher<endpoints::WorkcellStateTopic::MessageType>(endpoints::WorkcellStateTopic::topic_name(
        this->get_name()), endpoints::WorkcellStateTopic::qos());
  this->_cur_state = WorkcellState();
  this->_cur_state.workcell_id = this->get_name();
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
        auto r = this->_job_mgr->assign_task(req->task_id);
        if (r.error())
        {
          resp->success = false;
          resp->message = r.error()->what();
          return;
        }
        resp->success = true;
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
        auto r = this->_job_mgr->remove_assigned_task(req->task_id);
        if (r.error())
        {
          resp->success = false;
          resp->message = r.error()->what();
          return;
        }
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
      return std::make_unique<SetResult>(name, config,
      this->_job_mgr->context_manager());
    });
  this->_bt_factory->registerBuilder<GetResult>("GetResult",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetResult>(name, config,
      this->_job_mgr->context_manager());
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
      return std::make_unique<WaitForSignal>(name, config,
      this->_job_mgr->context_manager());
    });

  this->_bt_factory->registerBuilder<SetSignal>("SetSignal",
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetSignal>(name, config,
      this->_job_mgr->context_manager());
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
        this->shared_from_this(), this->_job_mgr->context_manager(),
        *_bt_factory);
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
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      for (std::size_t i = 0; i < it->second.size(); ++i)
      {
        this->_task_parser.add_remap_task_type(
          it->second[i].as<std::string>(), it->first.as<std::string>());
      }
    }
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
            std::exit(EXIT_CODE_REGISTRATION_FAILED);
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
  for (const auto& job : this->_job_mgr->jobs())
  {
    if (job.ctx->task.id == req->task_id)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Received signal [%s] for task [%s] from system orchestrator",
        req->signal.c_str(), req->task_id.c_str());
      job.ctx->signals.emplace(req->signal);
      resp->success = true;
      return;
    }
  }

  std::ostringstream oss;
  oss << "Failed to set signal signal [" << req->signal << "] for task [" <<
    req->task_id << "]: Task not found";
  resp->message = oss.str();
  RCLCPP_WARN_STREAM(this->get_logger(), resp->message);
  resp->success = false;
}

BT::Tree WorkcellOrchestrator::_create_bt(const std::shared_ptr<Context>& ctx)
{
  // To keep things simple, the task type is used as the key for the behavior tree to use.
  return this->_bt_factory->createTreeFromFile(this->_bt_store.get_bt(
        ctx->task.type));
}

void WorkcellOrchestrator::_handle_task_doable(
  endpoints::IsTaskDoableService::ServiceType::Request::ConstSharedPtr req,
  endpoints::IsTaskDoableService::ServiceType::Response::SharedPtr resp)
{
  RCLCPP_DEBUG(this->get_logger(), "Got request to check task doable");
  try
  {
    auto r = this->_task_parser.parse_task(req->task);
    if (r.error())
    {
      resp->success = false;
      return;
    }
    resp->success = this->_can_perform_task(r.value());
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

bool WorkcellOrchestrator::_can_perform_task(const Task& task)
{
  try
  {
    this->_bt_store.get_bt(task.type);
    return true;
  }
  catch (const std::out_of_range&)
  {
    return false;
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  nexus::workcell_orchestrator::WorkcellOrchestrator)
