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

#include <chrono>

#include "nexus_rviz_plugins/nexus_panel.hpp"

#include <action_msgs/srv/cancel_goal.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <QtConcurrent/QtConcurrent>

using namespace std::chrono_literals;

namespace nexus_rviz_plugins {
NEXUSPanel::NEXUSPanel(QWidget* parent)
: Panel(parent),
  server_timeout_(100)
{
  // Create the control button and its tooltip
  start_reset_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;
  cancel_button_ = new QPushButton;
  lifecycle_state_indicator_ = new QLabel;

  // Create the state machine used to present the proper control button states in the UI
  const char* startup_msg = "Configure and activate all nexus lifecycle nodes";
  const char* reset_msg = "Deactivate and cleanup all nexus lifecycle nodes";
  const char* pause_msg =
    "Pause all actions and deactivate all nexus lifecycle nodes";
  const char* resume_msg = "Unpause and activate all nexus lifecycle nodes";
  const char* cancel_msg = "Cancel the work order";

  const QString orchestrator_active_string(
    "<table><tr><td width=100><b>System Orchestrator:</b></td>"
    "<td>active</td></tr></table>");
  const QString orchestrator_inactive_string(
    "<table><tr><td width=100><b>System Orchestrator:</b></td>"
    "<td>inactive</td></tr></table>");

  lifecycle_state_indicator_->setText(orchestrator_inactive_string);
  lifecycle_state_indicator_->setSizePolicy(
    QSizePolicy::Fixed, QSizePolicy::Fixed);

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  assignButtonToState(pre_initial_, start_reset_button_,
    false, "Startup", startup_msg);
  assignButtonToState(pre_initial_, pause_resume_button_,
    false, "Pause", pause_msg);
  assignButtonToState(pre_initial_, cancel_button_,
    false, "Cancel", cancel_msg);

  // State when the system is not executing a work order
  initial_ = new QState();
  initial_->setObjectName("initial");
  assignButtonToState(initial_, start_reset_button_,
    true, "Startup", startup_msg);
  assignButtonToState(initial_, pause_resume_button_,
    false, "Pause", pause_msg);
  assignButtonToState(initial_, cancel_button_,
    false, "Cancel", cancel_msg);

  // State entered while the System Orchestrator is active
  running_ = new QState();
  running_->setObjectName("running");
  assignButtonToState(running_, start_reset_button_,
    false, "Reset (Pause to enable)", reset_msg);
  assignButtonToState(running_, pause_resume_button_,
    true, "Pause", pause_msg);
  assignButtonToState(running_, cancel_button_,
    true, "Cancel", cancel_msg);

  // State when the system is not executing a work order
  idle_ = new QState();
  idle_->setObjectName("paused");
  assignButtonToState(idle_, start_reset_button_,
    true, "Reset", reset_msg);
  assignButtonToState(idle_, pause_resume_button_,
    true, "Resume", resume_msg);
  assignButtonToState(idle_, cancel_button_,
    true, "Cancel", cancel_msg);

  // State entered to resume the nexus lifecycle nodes
  resuming_ = new QState();
  resuming_->setObjectName("resuming");

  // State entered to cancel the work order
  cancelling_ = new QState();
  cancelling_->setObjectName("cancelling");

  // State entered to reset the nexus lifecycle nodes
  resetting_ = new QState();
  resetting_->setObjectName("resetting");

  // State entered to pause the nexus lifecycle nodes
  pausing_ = new QState();
  pausing_->setObjectName("pausing");

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  QObject::connect(resetting_, SIGNAL(exited()), this, SLOT(onReset()));
  QObject::connect(pausing_, SIGNAL(exited()), this, SLOT(onPause()));
  QObject::connect(resuming_, SIGNAL(exited()), this, SLOT(onResume()));
  QObject::connect(cancelling_, SIGNAL(exited()), this, SLOT(onCancel()));

  // Start/Reset button click transitions
  initial_->addTransition(start_reset_button_, SIGNAL(clicked()), running_);
  idle_->addTransition(start_reset_button_, SIGNAL(clicked()), resetting_);

  // Pause/Resume button click transitions
  running_->addTransition(pause_resume_button_, SIGNAL(clicked()), pausing_);
  idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), resuming_);

  // Cancel button click transitions
  running_->addTransition(cancel_button_, SIGNAL(clicked()), cancelling_);
  idle_->addTransition(cancel_button_, SIGNAL(clicked()), cancelling_);

  // Internal state transitions
  resetting_->addTransition(resetting_, SIGNAL(entered()), initial_);
  resuming_->addTransition(resuming_, SIGNAL(entered()), running_);
  cancelling_->addTransition(cancelling_, SIGNAL(entered()), idle_);
  pausing_->addTransition(pausing_, SIGNAL(entered()), idle_);

  // This overrides the logging and name remapping
  auto options = rclcpp::NodeOptions().arguments({
        "--ros-args",
        "--remap", "_client_node:__node:=nexus_dialog_action_client",
        "--log-level", "info"});
  client_node_ = std::make_shared<rclcpp::Node>("_client_node", options);

  client_lifecycle_orchestrator_ =
    std::make_shared<nexus::lifecycle_manager::LifecycleManagerClient>(
    "rviz_lifecycle_manager", client_node_);

  initial_thread_ = new InitialThread(client_lifecycle_orchestrator_);
  connect(initial_thread_, &InitialThread::finished, initial_thread_,
    &QObject::deleteLater);

  // Transition on active/inactive event for preinitial
  QSignalTransition* activeSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::orchestratorActive);
  activeSignal->setTargetState(running_);
  pre_initial_->addTransition(activeSignal);

  QSignalTransition* inactiveSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::orchestratorInactive);
  inactiveSignal->setTargetState(initial_);
  pre_initial_->addTransition(inactiveSignal);

  QObject::connect(
    initial_thread_, &InitialThread::orchestratorActive,
    [this, orchestrator_active_string]
    {
      lifecycle_state_indicator_->setText(orchestrator_active_string);
    });

  QObject::connect(
    initial_thread_, &InitialThread::orchestratorInactive,
    [this, orchestrator_inactive_string]
    {
      lifecycle_state_indicator_->setText(orchestrator_inactive_string);
    });

  state_machine_.addState(pre_initial_);
  state_machine_.addState(initial_);
  state_machine_.addState(running_);
  state_machine_.addState(idle_);
  state_machine_.addState(resetting_);
  state_machine_.addState(pausing_);
  state_machine_.addState(resuming_);
  state_machine_.addState(cancelling_);

  state_machine_.setInitialState(pre_initial_);

  std::vector<std::string> node_names = {"system_orchestrator"};
  lifecycle_manager_ =
    std::make_shared<nexus::lifecycle_manager::LifecycleManager<rclcpp::Node>>(
    "rviz", // "_lifecycle_manager" will be appended to the node name
    node_names,
    true);

  // Initialize service client to pause/resume system
  pause_system_client_ =
    this->client_node_->create_client<PauseSystemService::ServiceType>(
    PauseSystemService::service_name());

  // Initialize service client to cancel work order
  cancel_wo_client_ =
    this->client_node_->create_client<action_msgs::srv::CancelGoal>(
    "/system_orchestrator/execute_order/_action/cancel_goal");

  // delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this,
    SLOT(startThread()));
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(lifecycle_state_indicator_);
  main_layout->addWidget(pause_resume_button_);
  main_layout->addWidget(start_reset_button_);
  main_layout->addWidget(cancel_button_);

  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);
}

NEXUSPanel::~NEXUSPanel()
{
}

void
NEXUSPanel::onInitialize()
{
}

void
NEXUSPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
NEXUSPanel::load(const rviz_common::Config& config)
{
  Panel::load(config);
}

void
NEXUSPanel::assignButtonToState(QState* state, QPushButton* button,
  bool enabled,
  const char* text, const char* tooltip)
{
  state->assignProperty(button, "text", text);
  state->assignProperty(button, "enabled", enabled);
  state->assignProperty(button, "toolTip", tooltip);
}

void NEXUSPanel::startThread()
{
  // start initial thread now that state machine is started
  initial_thread_->start();
}

void NEXUSPanel::onStartup()
{
  QtConcurrent::run(
    std::bind(
      &nexus::lifecycle_manager::LifecycleManagerClient::startup,
      client_lifecycle_orchestrator_.get(), std::placeholders::_1),
    server_timeout_).waitForFinished();

  QtConcurrent::run(
    std::bind(
      &NEXUSPanel::pause_system, this,
      std::placeholders::_1, std::placeholders::_2),
    false, std::chrono::seconds(5));
}

void NEXUSPanel::onReset()
{
  QtConcurrent::run(
    std::bind(
      &nexus::lifecycle_manager::LifecycleManagerClient::reset,
      client_lifecycle_orchestrator_.get(), std::placeholders::_1),
    server_timeout_);
}

void NEXUSPanel::onPause()
{
  QtConcurrent::run(
    std::bind(
      &nexus::lifecycle_manager::LifecycleManagerClient::pause,
      client_lifecycle_orchestrator_.get(), std::placeholders::_1),
    server_timeout_);

  QtConcurrent::run(
    std::bind(
      &NEXUSPanel::pause_system, this,
      std::placeholders::_1, std::placeholders::_2),
    true, std::chrono::seconds(5));
}

void NEXUSPanel::onResume()
{
  QtConcurrent::run(
    std::bind(
      &nexus::lifecycle_manager::LifecycleManagerClient::resume,
      client_lifecycle_orchestrator_.get(), std::placeholders::_1),
    server_timeout_);

  QtConcurrent::run(
    std::bind(
      &NEXUSPanel::pause_system, this,
      std::placeholders::_1, std::placeholders::_2),
    false, std::chrono::seconds(5));
}

void NEXUSPanel::onCancel()
{
  QtConcurrent::run(
    std::bind(
      &NEXUSPanel::cancel_workorder, this,
      std::placeholders::_1),
    std::chrono::seconds(5));
}

void NEXUSPanel::pause_system(const bool& pause,
  const std::chrono::seconds& timeout)
{
  auto request = std::make_shared<PauseSystemService::ServiceType::Request>();
  request->pause = pause;

  while (!pause_system_client_->wait_for_service(timeout))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(
        this->client_node_->get_logger(),
        "Interrupted while waiting for %s service.",
        PauseSystemService::service_name().c_str());
      return;
    }
    RCLCPP_INFO(
      this->client_node_->get_logger(),
      "service %s not available, exceeded timeout while waiting",
      PauseSystemService::service_name().c_str());
  }

  auto result = pause_system_client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->client_node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->success)
    {
      const std::string action = pause ? "paused" : "resumed";
      RCLCPP_INFO(
        this->client_node_->get_logger(),
        "Successfully %s the system.", action.c_str());
    }
    else
    {
      RCLCPP_ERROR(
        this->client_node_->get_logger(),
        "Failed to pause the system. Error: %s", response->message.c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(
      this->client_node_->get_logger(),
      "Failed to call service %s",
      PauseSystemService::service_name().c_str());
  }

}

void NEXUSPanel::cancel_workorder(const std::chrono::seconds& timeout)
{
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

  while (!cancel_wo_client_->wait_for_service(timeout))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(
        this->client_node_->get_logger(),
        "Interrupted while waiting for %s service.",
        cancel_wo_client_->get_service_name());
      return;
    }
    RCLCPP_INFO(
      this->client_node_->get_logger(),
      "service %s not available, exceeded timeout while waiting",
      cancel_wo_client_->get_service_name());
  }

  auto result = cancel_wo_client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->client_node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->return_code ==
      action_msgs::srv::CancelGoal::Response::ERROR_NONE)
    {
      RCLCPP_INFO(
        this->client_node_->get_logger(),
        "Successfully cancelled the workorder.");
    }
    else
    {
      RCLCPP_ERROR(
        this->client_node_->get_logger(),
        "Failed to cancel the workorder. Error code %d", response->return_code);
    }
  }
  else
  {
    RCLCPP_ERROR(
      this->client_node_->get_logger(),
      "Failed to call service %s",
      cancel_wo_client_->get_service_name());
  }

}

}  // namespace nexus_rviz_plugins


#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nexus_rviz_plugins::NEXUSPanel, rviz_common::Panel)
