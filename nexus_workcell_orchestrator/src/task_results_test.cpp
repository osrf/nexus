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

#include "get_result.hpp"
#include "set_result.hpp"

#include <nexus_capabilities/context.hpp>
#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common_test/test_utils.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rmf_utils/catch.hpp>

namespace nexus::workcell_orchestrator::test {

TEST_CASE("get and set results") {
  auto fixture = common::test::RosFixture<rclcpp_lifecycle::LifecycleNode>{};
  auto ctx_mgr = std::make_shared<ContextManager>();
  auto ctx = Context::make(*fixture.node);
  ctx_mgr->set_active_context(ctx);
  BT::BehaviorTreeFactory bt_factory;

  bt_factory.registerBuilder<SetResult>("SetResult",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SetResult>(name, config, ctx_mgr);
    });
  bt_factory.registerBuilder<GetResult>("GetResult",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetResult>(name, config, ctx_mgr, *fixture.node);
    });

  auto bt = bt_factory.createTreeFromText(
    R"(
    <?xml version='1.0' ?>
    <root main_tree_to_execute="Test">
      <BehaviorTree ID="Test">
        <Sequence>
          <SetResult key="hello" value="world" />
          <GetResult key="hello" result="{param}" />
        </Sequence>
      </BehaviorTree>
    </root>
  )");

  bt.tickRoot();
  CHECK(ctx->task.previous_results["hello"].as<std::string>() == "world");
  CHECK(bt.blackboard_stack.at(0)->getAny(
      "param")->cast<std::string>() == "world");
}

}
