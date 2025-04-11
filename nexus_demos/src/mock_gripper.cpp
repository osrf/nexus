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

#include "mock_gripper.hpp"

#include <memory>

using namespace std::literals;

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//==============================================================================
namespace nexus_demos {

//==============================================================================
MockGripper::MockGripper(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("mock_gripper", options)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Mock Gripper is running..."
  );

  _delay_millis = this->declare_parameter<int>("delay_millis", 50);
  bool autostart = this->declare_parameter<bool>("autostart", false);
  // If 'autostart' parameter is true, the node self-transitions to 'active' state upon startup
  if (autostart)
  {
    this->configure();
    this->activate();
  }
}

//==============================================================================
CallbackReturn MockGripper::on_configure(const rclcpp_lifecycle::State& previous)
{
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Mock gripper configuring...");

  this->action_server_ =
    rclcpp_action::create_server<GripperCommandAction::ActionType>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    GripperCommandAction::action_name(this->get_name()),
    [&](const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GripperCommandAction::ActionType::Goal> goal)
    {
      // Handle goal.
      if (this->get_current_state().label() != "active")
      {
        RCLCPP_ERROR(this->get_logger(), "Mock gripper is not activated. "
          "Rejecting goal request %s!",
          rclcpp_action::to_string(uuid).c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "Received goal request with order");
      // Accept all goals.
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&](const std::shared_ptr<GoalHandleGripper> goal_handle)
    {
      // Handle cancel.
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](const std::shared_ptr<GoalHandleGripper> goal_handle)
    {
      auto goal = goal_handle->get_goal();
      auto result =
        std::make_shared<GripperCommandAction::ActionType::Result>();
      result->position = goal->command.position;
      result->effort = goal->command.max_effort;
      result->reached_goal = true;
      // Delay for configured time.
      std::this_thread::sleep_for(std::chrono::milliseconds(_delay_millis));
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    });

  RCLCPP_INFO(this->get_logger(), "Mock gripper configured");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockGripper::on_activate(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock gripper activated");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockGripper::on_deactivate(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock gripper deactivated");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockGripper::on_cleanup(const rclcpp_lifecycle::State& previous)
{
  action_server_.reset();
  RCLCPP_INFO(this->get_logger(), "Mock gripper cleaned up");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockGripper::on_shutdown(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock gripper has shut down");
  return CallbackReturn::SUCCESS;
}

}  // namespace nexus_demos

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus_demos::MockGripper)
