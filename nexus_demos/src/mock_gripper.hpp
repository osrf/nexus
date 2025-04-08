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

//==============================================================================
namespace nexus_demos {

class MockGripper : public rclcpp_lifecycle::LifecycleNode
{
public:
  using GripperCommandAction = nexus::endpoints::GripperCommandAction;
  using GoalHandleGripper =
    rclcpp_action::ServerGoalHandle<GripperCommandAction::ActionType>;

  explicit MockGripper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override;

private:
  rclcpp_action::Server<GripperCommandAction::ActionType>::
  SharedPtr action_server_;
  int _delay_millis;
};

}  // namespace nexus_demos
