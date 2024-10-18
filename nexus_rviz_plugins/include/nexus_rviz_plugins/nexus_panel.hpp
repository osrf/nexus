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

#ifndef JNJ_RVIZ_PLUGINS__JNJ_PANEL_HPP_
#define JNJ_RVIZ_PLUGINS__JNJ_PANEL_HPP_

#include <QtWidgets>

#include "rviz_common/panel.hpp"

#include "nexus_rviz_plugins/ros_action_qevent.hpp"

#include <nexus_lifecycle_manager/lifecycle_manager_client.hpp>
#include <nexus_lifecycle_manager/lifecycle_manager.hpp>

#include <nexus_endpoints.hpp>

#include <rclcpp/rclcpp.hpp>

namespace nexus_rviz_plugins {

using PauseSystemService = nexus::endpoints::PauseSystemService;

class InitialThread;

/// Panel to interface to the NEXUS stack
class NEXUSPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NEXUSPanel(QWidget* parent = 0);
  virtual ~NEXUSPanel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void startThread();
  void onStartup();
  void onReset();
  void onPause();
  void onResume();
  void onCancel();

  // Send a pause/resume system service request
  void pause_system(const bool& pause,
    const std::chrono::seconds& timeout = std::chrono::seconds(5));

  // Send a cancel workorder action request
  void cancel_workorder(
    const std::chrono::seconds& timeout = std::chrono::seconds(5));

  // Helper method to assign properties to QState objects
  void assignButtonToState(QState* state, QPushButton* button,
    bool enabled, const char* text, const char* tooltip = "");

private:
  QPushButton* start_reset_button_{nullptr};
  QPushButton* pause_resume_button_{nullptr};
  QPushButton* cancel_button_{nullptr};

  QLabel* lifecycle_state_indicator_{nullptr};

  QState* initial_{nullptr};
  QState* running_{nullptr};
  QState* idle_{nullptr};
  // Internal states
  QState* pre_initial_{nullptr};
  QState* resuming_{nullptr};
  QState* pausing_{nullptr};
  QState* cancelling_{nullptr};
  QState* resetting_{nullptr};

  // Timeout value when waiting for action servers to respnd
  std::chrono::milliseconds server_timeout_;

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  std::shared_ptr<nexus::lifecycle_manager::LifecycleManager<rclcpp::Node>>
  lifecycle_manager_;

  std::thread spin_thread_;

  std::shared_ptr<nexus::lifecycle_manager::LifecycleManagerClient>
  client_lifecycle_orchestrator_;

  std::shared_ptr<rclcpp::Client<PauseSystemService::ServiceType>>
  pause_system_client_;

  std::shared_ptr<rclcpp::Client<action_msgs::srv::CancelGoal>>
  cancel_wo_client_;

  QStateMachine state_machine_;
  InitialThread* initial_thread_;

  // GetStateThread* get_state_thread_;
};

// class GetStateThread : public QThread
// {
//   Q_OBJECT
// public:
//   GetStateThread(std::shared_ptr<nexus::lifecycle_manager::LifecycleManager<rclcpp::Node>>&
//     _lifecycle_manager)
//   : lifecycle_manager_(_lifecycle_manager)
//   {
//   }

//   void run() override
//   {
//     while (true){

//     }
//   }
// signals:
//   void orchestratorActive();
//   void orchestratorInactive();

// private:
//   std::shared_ptr<nexus::lifecycle_manager::LifecycleManager<rclcpp::Node>>
//   lifecycle_manager_;
// }

class InitialThread : public QThread
{
  Q_OBJECT

public:
  InitialThread(std::shared_ptr<nexus::lifecycle_manager::LifecycleManagerClient>&
    _client_orchestrator)
  : client_orchestrator_(_client_orchestrator)
  {
  }

  void run() override
  {
    while (true)
    {
      nexus::lifecycle_manager::SystemStatus status_orchestrator =
        nexus::lifecycle_manager::SystemStatus::TIMEOUT;

      while (status_orchestrator ==
        nexus::lifecycle_manager::SystemStatus::TIMEOUT)
      {
        if (status_orchestrator ==
          nexus::lifecycle_manager::SystemStatus::TIMEOUT)
        {
          status_orchestrator = client_orchestrator_->is_active(std::chrono::seconds(
                1));
        }
      }

      if (status_orchestrator == nexus::lifecycle_manager::SystemStatus::ACTIVE)
      {
        emit orchestratorActive();
      }
      else
      {
        emit orchestratorInactive();
      }
    }
  }

signals:
  void orchestratorActive();
  void orchestratorInactive();

private:
  std::shared_ptr<nexus::lifecycle_manager::LifecycleManagerClient>
  client_orchestrator_;
};

}  // namespace nexus_rviz_plugins

#endif  // JNJ_RVIZ_PLUGINS__JNJ_PANEL_HPP_
