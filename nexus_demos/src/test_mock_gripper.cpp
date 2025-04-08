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

#include <rmf_utils/catch.hpp>

#include "mock_gripper.hpp"

#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/client.hpp>

using GripperAction = nexus::endpoints::GripperCommandAction::ActionType;
using GoalHandleGripperAction =
  rclcpp_action::ClientGoalHandle<GripperAction>;

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;

template<typename FutureT>
void spin_until_future_complete(std::shared_future<FutureT>& future)
{
  std::future_status status;
  do
  {
    rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), future);
    status = future.wait_for(std::chrono::seconds(0));
  } while (std::future_status::ready != status);
}

SCENARIO("Test Gripper Mock")
{
  rclcpp::init(0, nullptr);

  node = std::make_shared<nexus_demos::MockGripper>(rclcpp::NodeOptions());

  node->configure();
  node->activate();

  auto client =
    rclcpp_action::create_client<GripperAction>(
    node,
    nexus::endpoints::GripperCommandAction::action_name(node->get_name()));

  if (!client->wait_for_action_server(std::chrono::seconds(10)))
  {
    REQUIRE(false);
    rclcpp::shutdown();
    return;
  }

  auto goal_msg = GripperAction::Goal();
  goal_msg.command.position = 0.6;
  bool goal_response = false;
  double end_position = 0.0;

  auto goal_response_callback =
    [&goal_response](GoalHandleGripperAction::SharedPtr goal_handle)
    {
      if (goal_handle)
      {
        goal_response = true;
      }
    };

  auto result_callback =
    [&end_position](const GoalHandleGripperAction::WrappedResult& result)
    {
      end_position = result.result->position;
    };

  auto send_goal_options =
    rclcpp_action::Client<GripperAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = goal_response_callback;
  send_goal_options.result_callback = result_callback;
  auto future = client->async_send_goal(goal_msg, send_goal_options);
  spin_until_future_complete(future);

  auto goal_handle = future.get();
  CHECK(goal_response == false);
  CHECK(nullptr == goal_handle.get());

  goal_msg = GripperAction::Goal();
  goal_msg.command.position = 0.2;

  future = client->async_send_goal(goal_msg, send_goal_options);
  spin_until_future_complete(future);

  goal_handle = future.get();
  CHECK(rclcpp_action::GoalStatus::STATUS_ACCEPTED ==
    goal_handle->get_status());

  auto future_result = client->async_get_result(goal_handle);
  spin_until_future_complete(future_result);

  CHECK(goal_response == true);
  CHECK(0.2 == end_position);

  node.reset();

  rclcpp::shutdown();
}
