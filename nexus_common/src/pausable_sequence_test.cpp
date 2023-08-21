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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "pausable_sequence.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

namespace nexus::common::test {

TEST_CASE("ActionClientBtNode", "[BehaviorTree]")
{
  BT::BehaviorTreeFactory bt_factory;
  bt_factory.registerNodeType<PausableSequence>("PausableSequence");

  size_t run_count = 0;
  bt_factory.registerSimpleAction("NothingThere",
    [&run_count](BT::TreeNode&)
    {
      ++run_count;
      return BT::NodeStatus::SUCCESS;
    }, {});
  bt_factory.registerSimpleAction("BOOM", [](BT::TreeNode&)
    {
      return BT::NodeStatus::FAILURE;
    });

  SECTION("all children are ran") {
    auto bt = bt_factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <PausableSequence pause="false">
            <NothingThere/>
            <NothingThere/>
            <BOOM/>
          </PausableSequence>
        </BehaviorTree>
      </root>
      )");

    auto result = bt.tickRoot();
    for (size_t i = 0; result == BT::NodeStatus::RUNNING && i < 10;
      result = bt.tickRoot(), ++i)
    {
    }
    CHECK(result == BT::NodeStatus::FAILURE);
    CHECK(run_count == 2);
  }

  SECTION("gives other nodes a chance to pause") {
    bool force_resume = false;
    bt_factory.registerSimpleAction("ShouldPause", [&](BT::TreeNode& node)
      {
        node.setOutput("pause", run_count >= 2 && !force_resume);
        return BT::NodeStatus::SUCCESS;
      }, { BT::OutputPort<bool>("pause") });

    auto bt = bt_factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <ReactiveSequence>
            <ShouldPause pause="{pause}"/>
            <PausableSequence pause="{pause}">
              <NothingThere/>
              <NothingThere/>
              <BOOM/>
            </PausableSequence>
          </ReactiveSequence>
        </BehaviorTree>
      </root>
      )");

    auto result = bt.tickRoot();
    for (size_t i = 0; result == BT::NodeStatus::RUNNING && i < 10;
      result = bt.tickRoot(), ++i)
    {
    }
    CHECK(result == BT::NodeStatus::RUNNING);
    CHECK(run_count == 2);

    SECTION("should resume from the paused node") {
      force_resume = true;
      result = bt.tickRoot();
      CHECK(result == BT::NodeStatus::FAILURE);
      CHECK(run_count == 2);
    }
  }
}

}
