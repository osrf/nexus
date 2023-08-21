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

#include <memory>

#include <nexus_endpoints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using namespace std::literals;

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MockGripper : public rclcpp_lifecycle::LifecycleNode
{
public:
  using GripperCommandAction = nexus::endpoints::GripperCommandAction;

  using GoalHandleGripper =
    rclcpp_action::ServerGoalHandle<GripperCommandAction::ActionType>;

  explicit MockGripper(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    double gripperMaxValue = std::numeric_limits<double>::max())
  : rclcpp_lifecycle::LifecycleNode("mock_gripper", options),
    gripperMaxValue_(gripperMaxValue)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock Gripper is running..."
    );

    increment_ = this->declare_parameter("increment", 0.01);

    bool autostart;
    this->declare_parameter<bool>("autostart", false);
    this->get_parameter<bool>("autostart", autostart);
    // If 'autostart' parameter is true, the node self-transitions to 'active' state upon startup
    if (autostart)
    {
      this->configure();
      this->activate();
    }
  }

  ~MockGripper()
  {
    if (this->get_current_state().label() != "unconfigured"
      && this->get_current_state().label() != "finalized")
    {
      on_cleanup(this->get_current_state());
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override
  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Mock gripper configuring...");

    this->declare_parameter<double>("gripper_max_value",
      this->gripperMaxValue_);

    if (gripperMaxValue_ == std::numeric_limits<double>::max())
    {
      auto parameters = this->get_parameters({"gripper_max_value"});
      this->gripperMaxValue_ = parameters[0].get_value<double>();
    }
    else
    {
      this->gripperMaxValue_ = gripperMaxValue_;
    }

    this->action_server_ =
      rclcpp_action::create_server<GripperCommandAction::ActionType>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      GripperCommandAction::action_name(this->get_name()),
      std::bind(&MockGripper::handle_goal, this, _1, _2),
      std::bind(&MockGripper::handle_cancel, this, _1),
      std::bind(&MockGripper::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Mock gripper configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock gripper activating...");

    RCLCPP_INFO(this->get_logger(), "Mock gripper activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock gripper deactivating...");

    RCLCPP_INFO(this->get_logger(), "Mock gripper deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock gripper cleaning up...");

    action_server_.reset();

    if (this->thread_execute_.joinable())
      this->thread_execute_.join();

    position_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Mock gripper cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock gripper shutting down...");

    RCLCPP_INFO(this->get_logger(), "Mock gripper has shut down");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_action::Server<GripperCommandAction::ActionType>::
  SharedPtr action_server_;
  std::thread thread_execute_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GripperCommandAction::ActionType::Goal> goal)
  {
    if (this->get_current_state().label() != "active")
    {
      RCLCPP_ERROR(this->get_logger(), "Mock gripper is not activated. "
        "Rejecting goal request %s!",
        rclcpp_action::to_string(uuid).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Received goal request with order");
    if (goal->command.position < 0 ||
      goal->command.position > this->gripperMaxValue_)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    this->thread_execute_ =
      std::thread{std::bind(&MockGripper::execute, this, _1), goal_handle};
    this->thread_execute_.join();
  }

  void execute(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();

    auto feedback =
      std::make_shared<GripperCommandAction::ActionType::Feedback>();
    feedback->position = this->position_;

    auto result =
      std::make_shared<GripperCommandAction::ActionType::Result>();

    while (this->position_ != goal->command.position)
    {
      if (goal_handle->is_canceling())
      {
        result->position = this->position_;
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (goal->command.position > this->position_)
      {
        this->position_ = this->position_ + increment_;
      }
      else
      {
        this->position_ = this->position_ - increment_;
      }

      if (this->position_ < 0)
      {
        this->position_ = 0;
        break;
      }
      if (abs(this->position_ - goal->command.position) < increment_)
      {
        this->position_ = goal->command.position;
        break;
      }

      feedback->position = this->position_;

      // Publish feedback
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->position = this->position_;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  double position_{0};
  double gripperMaxValue_;
  double increment_;
};  // class MockGripper
