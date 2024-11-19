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

#include <chrono>
#include <utility>
#include <vector>
#include "nexus_robot_controller/robot_controller_server.hpp"

// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const K_SCHED_PRIORITY = 50;
int const CM_DEFAULT_UPDATE_RATE = 100;

/// Constants defining string labels corresponding to lifecycle states
constexpr char UNKNOWN[] = "unknown";
constexpr char UNCONFIGURED[] = "unconfigured";
constexpr char INACTIVE[] = "inactive";
constexpr char ACTIVE[] = "active";
constexpr char FINALIZED[] = "finalized";

namespace nexus::robot_controller_server {
struct RobotControllerServer::Implementation
{
  Implementation(std::shared_ptr<rclcpp::Executor> executor,
    std::string ns,
    std::string cm_node_name)
  : executor_(executor),
    ns_(ns),
    cm_node_name_(cm_node_name),
    cm_thread_run_flag_(false) {}
  ~Implementation() {}

  std::shared_ptr<rclcpp::Executor> executor_;
  std::string ns_;

  // Controller Manager Related
  std::shared_ptr<controller_manager::ControllerManager> cm_node_;
  std::string cm_node_name_;

  std::atomic<bool> cm_thread_run_flag_;
  std::unique_ptr<std::thread> cm_thread_ptr_;

  // Logging prefixes
  std::ostringstream server_logging_prefix_;
};


RobotControllerServer::RobotControllerServer(
  std::shared_ptr<rclcpp::Executor> executor,
  const std::string& node_name,
  const std::string& ns,
  const std::string& cm_node_name,
  const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
  pimpl_(std::make_unique<Implementation>(executor, ns, cm_node_name))
{
  declare_parameter("robot_description", "");
  declare_parameter("managed_controllers", std::vector<std::string>());

  pimpl_->server_logging_prefix_ << "[CONTROLLER SERVER: " << pimpl_->ns_ <<
    "/" << node_name << "] ";

  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Started. Needs configuration...");
}

RobotControllerServer::~RobotControllerServer()
{
  pimpl_->cm_thread_run_flag_ = false;
  pimpl_->cm_thread_ptr_->join();
  pimpl_->cm_thread_ptr_.reset();
  pimpl_->cm_node_.reset();
}

// =============================================================================
// LIFECYCLE METHODS
// =============================================================================
CallbackReturn
RobotControllerServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Configuring");
  auto node_ = shared_from_this();

  // Prep params
  RCLCPP_INFO_STREAM(
    get_logger(),
    "ROBOT_DESCRIPTION PARAM VALUE:" <<
      node_->get_parameter("robot_description").as_string()
    );
  if (node_->get_parameter("robot_description").as_string().empty())
  {
    RCLCPP_WARN_STREAM(get_logger(), pimpl_->server_logging_prefix_.str()
        << "Failed to configure. Missing robot_description parameter!");
    return CallbackReturn::FAILURE;
  }

  // Setup controller manager (with thread)
  pimpl_->cm_node_ = std::make_shared<controller_manager::ControllerManager>(
    pimpl_->executor_, pimpl_->cm_node_name_, pimpl_->ns_);

  pimpl_->cm_thread_run_flag_ = true;
  pimpl_->cm_thread_ptr_ = std::unique_ptr<std::thread>(
    new std::thread(
      [this]()
      {
        std::ostringstream cm_logging_prefix_;
        cm_logging_prefix_ << "[CONTROLLER MANAGER: "
                           << this->pimpl_->ns_ << "/" <<
          this->pimpl_->cm_node_name_
                           << "] ";

        int update_rate = CM_DEFAULT_UPDATE_RATE;
        auto cm_ = this->pimpl_->cm_node_;

        if (!pimpl_->cm_node_->get_parameter("update_rate", update_rate))
        {
          RCLCPP_WARN_STREAM(
            pimpl_->cm_node_->get_logger(),
            cm_logging_prefix_.str() <<
              "'update_rate' parameter not set, using default value."
    );
        }
        RCLCPP_INFO_STREAM(
          pimpl_->cm_node_->get_logger(),
          cm_logging_prefix_.str() << "update rate is " <<
            update_rate << " Hz");

        if (realtime_tools::has_realtime_kernel())
        {
          if (!realtime_tools::configure_sched_fifo(K_SCHED_PRIORITY))
          {
            RCLCPP_WARN_STREAM(cm_->get_logger(),
            cm_logging_prefix_.str()
              << "Could not enable FIFO RT scheduling policy");
          }
        }
        else
        {
          RCLCPP_INFO_STREAM(cm_->get_logger(),
          cm_logging_prefix_.str() <<
            "RT kernel is recommended for better performance");
        }

        auto const period =
        std::chrono::nanoseconds(1'000'000'000 / cm_->get_update_rate());
        std::chrono::system_clock::time_point next_iteration_time =
        std::chrono::system_clock::time_point(
          std::chrono::nanoseconds(cm_->now().nanoseconds())
        );

        rclcpp::Time previous_time = cm_->now();

        while (rclcpp::ok() && this->pimpl_->cm_thread_run_flag_)
        {
          auto const current_time = cm_->now();
          auto const measured_period = current_time - previous_time;
          previous_time = current_time;

          cm_->read(cm_->now(), measured_period);
          cm_->update(cm_->now(), measured_period);
          cm_->write(cm_->now(), measured_period);

          next_iteration_time += period;
          std::this_thread::sleep_until(next_iteration_time);
        }
        RCLCPP_INFO_STREAM(get_logger(),
        cm_logging_prefix_.str() << "CM Thread Ending");
        return;
      }
    )
  );
  pimpl_->executor_->add_node(pimpl_->cm_node_);

  std::vector<std::string> managed_controllers;
  // TODO(methylDragon): Refactor out the use of managed_controllers once new rclcpp feature is in:
  //                     https://github.com/ros2/rclcpp/pull/2090
  node_->get_parameter("managed_controllers", managed_controllers);
  for (auto& controller_name : managed_controllers)
  {
    pimpl_->cm_node_->load_controller(controller_name);
    pimpl_->cm_node_->configure_controller(controller_name);
  }

  if (!pimpl_->cm_thread_ptr_->joinable())
  {
    RCLCPP_WARN_STREAM(get_logger(),
      pimpl_->server_logging_prefix_.str() << "Failed to configure");
    pimpl_->cm_thread_run_flag_ = false;
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
RobotControllerServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Cleaning Up");
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Unloading Controllers...");
  for (auto controller : pimpl_->cm_node_->get_loaded_controllers())
  {
    if (pimpl_->cm_node_->unload_controller(controller.info.name) ==
      controller_interface::return_type::OK)
    {
      RCLCPP_INFO_STREAM(
        get_logger(),
        pimpl_->server_logging_prefix_.str() << "Unloaded " <<
          controller.info.name);
    }
  }

  pimpl_->cm_thread_run_flag_ = false;
  pimpl_->cm_thread_ptr_->join();
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Thread Terminated");

  pimpl_->executor_->remove_node(pimpl_->cm_node_);

  pimpl_->cm_node_.reset();
  pimpl_->cm_thread_ptr_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
RobotControllerServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Activating");

  std::vector<std::string> controller_names;
  for (auto controller : pimpl_->cm_node_->get_loaded_controllers())
  {
    if (controller.c->get_lifecycle_state().label() != ACTIVE)
    {
      RCLCPP_INFO_STREAM(
        get_logger(),
        pimpl_->server_logging_prefix_.str() << "Switching on " <<
          controller.info.name);
      controller_names.push_back(controller.info.name);
    }
  }

  pimpl_->cm_node_->switch_controller(controller_names,
    std::vector<std::string>(), true, true);
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Switched on Controllers...");


  return CallbackReturn::SUCCESS;
}

CallbackReturn
RobotControllerServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Deactivating");

  std::vector<std::string> controller_names;
  for (auto controller : pimpl_->cm_node_->get_loaded_controllers())
  {
    if (controller.c->get_lifecycle_state().label() == ACTIVE)
    {
      RCLCPP_INFO_STREAM(
        get_logger(),
        pimpl_->server_logging_prefix_.str() << "Switching off " <<
          controller.info.name);
      controller_names.push_back(controller.info.name);
    }
  }

  pimpl_->cm_node_->switch_controller(
    std::vector<std::string>(), controller_names, true, true);
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Switched off Controllers...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
RobotControllerServer::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() << "Shutting Down");
  RCLCPP_INFO_STREAM(get_logger(),
    pimpl_->server_logging_prefix_.str() <<
      "Shutdown: Running deactivate and cleanup");

  on_deactivate(state);
  on_cleanup(state);

  return CallbackReturn::SUCCESS;
}

}  // end namespace nexus::nexus_controller_server
