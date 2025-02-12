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

#include <nexus_endpoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "std_srvs/srv/set_bool.hpp"


class MockEmergencyAlarm : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;
  using EStopMessageType = nexus::endpoints::EmergencyStopTopic::MessageType;

  MockEmergencyAlarm(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("mock_emergency_alarm_node", options)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm is running...");
  }

  CallbackReturn on_configure(const State& previous_state) override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm configuring...");

    // Create EmergencyStop publisher
    estop_pub_ =
      this->create_publisher<EStopMessageType>(
      nexus::endpoints::EmergencyStopTopic::topic_name(),
      nexus::endpoints::EmergencyStopTopic::qos());

    // Create a service to handle emergency stop trigger requests
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "/test/set_estop",
      [this](
        const std::shared_ptr<const std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        // Check if node is active
        if (this->get_current_state().label() != "active")
        {
          RCLCPP_WARN(
            this->get_logger(),
            "Mock Emergency Alarm is not active. Ignoring EmergencyStop request."
          );
          return;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "Incoming request to trigger emergency stop state");

        this->estop_state_ = request->data;
        // Publish emergency stop state
        publish_current_state();
        response->success = true;
      }
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm configured.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const State& previous_state) override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm cleaning up...");

    estop_pub_.reset();
    estop_srv_.reset();
    estop_state_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const State& previous_state) override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm shutting down...");

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm has shut down.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const State& previous_state) override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm activating...");

    // Activate a publish initialized estop state.
    estop_pub_->on_activate();
    publish_current_state();

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const State& previous_state) override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm deactivating...");

    // Deactivate publisher
    estop_pub_->on_deactivate();

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Emergency Alarm deactivated.");
    return CallbackReturn::SUCCESS;
  }

  void publish_current_state()
  {
    EStopMessageType msg;
    msg.emergency_button_stop = estop_state_;
    if (estop_state_)
    {
      msg.emergency_state = EStopMessageType::STATE_ACTIVE;
      RCLCPP_INFO(
        this->get_logger(), "Emergency stop state set to active");
    }
    else
    {
      msg.emergency_state = EStopMessageType::STATE_INACTIVE;
      RCLCPP_INFO(
        this->get_logger(), "Emergency stop state set to inactive");
    }
    estop_pub_->publish(msg);
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<EStopMessageType>::SharedPtr
    estop_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  bool estop_state_ = false;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MockEmergencyAlarm)
