/*
 * Copyright (C) 2023 Johnson & Johnson
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

#include "job_manager.hpp"

#include <nexus_common_test/test_utils.hpp>

#include <catch2/catch.hpp>

namespace nexus::workcell_orchestrator::test {

using nexus_orchestrator_msgs::msg::TaskState;
using nexus_orchestrator_msgs::action::WorkcellTask;

TEST_CASE("JobManager") {
  auto fixture = common::test::RosFixture<rclcpp_lifecycle::LifecycleNode>();
  BT::BehaviorTreeFactory bt_factory;
  JobManager job_mgr(fixture.node, 2);

  std::function<void(const JobManager::GoalHandlePtr&)> handle_accepted =
    [](const JobManager::GoalHandlePtr&) {};

  const auto server = rclcpp_action::create_server<WorkcellTask>(fixture.node,
      "test",
      [](const rclcpp_action::GoalUUID&,
      const endpoints::WorkcellRequestAction::ActionType::Goal::ConstSharedPtr&)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const JobManager::GoalHandlePtr&)
      {
        return rclcpp_action::CancelResponse::REJECT;
      }, [&](const JobManager::GoalHandlePtr& goal_handle)
      {
        handle_accepted(goal_handle);
      });
  const auto client = rclcpp_action::create_client<WorkcellTask>(fixture.node,
      "test");

  std::string bt =
    R"(
<?xml version='1.0' ?>
<root main_tree_to_execute="Test">
  <BehaviorTree ID="Test">
    <AlwaysSuccess />
  </BehaviorTree>
</root>
)";

  fixture.spin_in_background();

  SECTION("cannot assign same task twice") {
    CHECK_NOTHROW(job_mgr.assign_task("test"));
    CHECK_THROWS_AS(job_mgr.assign_task("test"), JobError);
  }

  SECTION("cannot queue task that is not assigned") {
    handle_accepted = [&](const JobManager::GoalHandlePtr& goal_handle)
      {
        CHECK_THROWS_AS(job_mgr.queue_task(goal_handle, nullptr,
          bt_factory.createTreeFromText(bt)), JobError);
      };

    WorkcellTask::Goal goal;
    goal.task.id = "test";
    std::promise<void> done;
    decltype(client)::element_type::SendGoalOptions goal_opts;
    goal_opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<WorkcellTask>::WrappedResult&)
      {
        done.set_value();
      };
    client->async_send_goal(goal, goal_opts);
    REQUIRE(done.get_future().wait_for(std::chrono::seconds(
        1)) == std::future_status::ready);
  }

  SECTION("assign task sets job data correctly") {
    Job* job;
    job = &job_mgr.assign_task("test");
    CHECK(!job->bt.has_value());
    CHECK(!job->bt_logging.has_value());
    CHECK(job->ctx == nullptr);
    CHECK(job->goal_handle == nullptr);
    CHECK(job->task_state.task_id == "test");
    CHECK(job->task_state.workcell_id == fixture.node->get_name());
    CHECK(job->task_state.status == TaskState::STATUS_ASSIGNED);
    CHECK(job->tick_count == 0);

    SECTION("ticking assigned but unqueued task is a noop") {
      job_mgr.tick();
      CHECK(job->tick_count == 0);
    }

    SECTION("can queue an assigned task") {
      handle_accepted = [&](const JobManager::GoalHandlePtr& goal_handle)
        {
          auto ctx = std::make_shared<Context>(fixture.node);
          CHECK(&job_mgr.queue_task(goal_handle, ctx,
            bt_factory.createTreeFromText(bt)) == job);
          CHECK(job->bt.has_value());
          CHECK(job->bt_logging.has_value());
          REQUIRE(job->ctx == ctx);
          CHECK(job->goal_handle == goal_handle);
          CHECK(job->task_state.task_id == "test");
          CHECK(job->task_state.status == TaskState::STATUS_QUEUED);
          CHECK(job->tick_count == 0);
          goal_handle->succeed(std::make_shared<WorkcellTask::Result>());
        };

      WorkcellTask::Goal goal;
      goal.task.id = "test";
      std::promise<void> done;
      decltype(client)::element_type::SendGoalOptions goal_opts;
      goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<WorkcellTask>::WrappedResult&)
        {
          done.set_value();
        };
      client->async_send_goal(goal, goal_opts);
      REQUIRE(done.get_future().wait_for(std::chrono::seconds(
          1)) == std::future_status::ready);
    }
  }

  SECTION("tick at most max_concurrent jobs") {
    std::vector<std::string> task_ids{"test1", "test2", "test3"};
    for (const auto& task_id : task_ids)
    {
      job_mgr.assign_task(task_id);
    }

    std::promise<void> done;
    size_t queued = 0;
    handle_accepted = [&](const JobManager::GoalHandlePtr& goal_handle)
      {
        auto ctx = std::make_shared<Context>(fixture.node);
        REQUIRE_NOTHROW(job_mgr.queue_task(goal_handle, ctx,
          bt_factory.createTreeFromText(bt)));
        if (++queued == task_ids.size())
        {
          done.set_value();
        }
      };

    // queue all tasks
    for (const auto& task_id : task_ids)
    {
      WorkcellTask::Goal goal;
      goal.task.id = task_id;
      decltype(client)::element_type::SendGoalOptions goal_opts;
      client->async_send_goal(goal, goal_opts);
    }
    REQUIRE(done.get_future().wait_for(std::chrono::seconds(
        1)) == std::future_status::ready);

    job_mgr.tick();
    // test1 and test2 should be finished
    CHECK(job_mgr.jobs().size() == 1);

    job_mgr.tick();
    // test3 should now be finished
    CHECK(job_mgr.jobs().size() == 0);
  }
}

}
