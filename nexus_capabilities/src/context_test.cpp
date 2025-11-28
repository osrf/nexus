#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include <chrono>
#include <thread>
#include "nexus_capabilities/context.hpp"

#include "nexus_common_test/test_utils.hpp"

namespace nexus::capabilities::test {

using TaskState = nexus_orchestrator_msgs::msg::TaskState;

bool check_order(const ContextSet& set, const std::vector<std::string> expected_tasks) {
  REQUIRE(set.size() == expected_tasks.size());
  std::cout << "BEGIN CHECK" << std::endl;
  for (std::size_t i = 0; i < set.size(); ++i) {
    const auto it_opt = set.get_at(i);
    REQUIRE(it_opt != std::nullopt);
    std::cout << (*it_opt)->task.task_id << " " << expected_tasks[i] << std::endl;
    if ((*it_opt)->task.task_id != expected_tasks[i]) {
      return false;
    }
  }
  return true;
}

TEST_CASE("test_context_ordering") {
  ContextSet contexts;
  nexus::common::test::RosFixture<rclcpp_lifecycle::LifecycleNode> uut_fixture;
  auto& uut_node = *uut_fixture.node;

  // We don't really need the node so we can reuse it
  auto ctx1 = Context::make(uut_node);
  auto ctx2 = Context::make(uut_node);

  ctx1->task.task_id = "task_1";
  ctx2->task.task_id = "task_2";
  
  // Both tasks are added in a sequence, expect the first to be on top
  contexts.insert(ctx1);
  contexts.insert(ctx2);

  REQUIRE(contexts.size() == 2);
  CHECK(check_order(contexts, {"task_1", "task_2"}));

  // Queue task_2, it should now be on top
  contexts.modify_task_id("task_2", [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_QUEUED);
  });
  CHECK(check_order(contexts, {"task_2", "task_1"}));

  // Now execute task_1, then add a new task afterwards, we should get task_1, task_2, task_3
  contexts.modify_task_id("task_1", [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_RUNNING);
  });
  auto ctx3 = Context::make(uut_node);
  ctx3->task.task_id = "task_3";
  contexts.insert(ctx3);
  CHECK(check_order(contexts, {"task_1", "task_2", "task_3"}));
  // If the new task gets queued, nothing changes because one is running already and another
  // one was queued before
  contexts.modify_task_id("task_3", [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_QUEUED);
  });
  CHECK(check_order(contexts, {"task_1", "task_2", "task_3"}));
  // If the task that was queued is set to running, it will still stay in its place because we
  // can't pre-empt currently running tasks
  contexts.modify_task_id("task_2", [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_RUNNING);
  });
  CHECK(check_order(contexts, {"task_1", "task_2", "task_3"}));
  // Once the task finishes or fails, it gets pushed to the end of the queue
  contexts.modify_task_id("task_1", [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_FINISHED);
  });
  CHECK(check_order(contexts, {"task_2", "task_3", "task_1"}));
  // Unnecessary but for multiple finished / failed the earliest updated is more in front of the queue
  contexts.modify_at(0, [](Context& ctx) {
    ctx.set_task_status(TaskState::STATUS_FAILED);
  });
  CHECK(check_order(contexts, {"task_3", "task_1", "task_2"}));
}

}
