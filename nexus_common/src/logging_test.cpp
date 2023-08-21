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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "logging.hpp"
#include "test_utils.hpp"

namespace nexus::common::test {

bool starts_with(const std::string& s, const std::string& prefix)
{
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

TEST_CASE("log_bt_statuses") {
  BT::BehaviorTreeFactory bt_factory;
  bt_factory.registerSimpleAction("NoOp", [](BT::TreeNode&)
    {
      return BT::NodeStatus::SUCCESS;
    });

  auto bt = bt_factory.createTreeFromText(
    R"(
      <root>
        <BehaviorTree>
          <Repeat num_cycles="2">
            <Sequence>
              <NoOp name="first" />
              <NoOp name="second" />
            </Sequence>
          </Repeat>
        </BehaviorTree>
      </root>
      )");

  std::vector<std::string> logs;
  BtLogging bt_logging(bt, [&logs](const auto& s)
    {
      logs.emplace_back(s);
    });
  CHECK(bt.tickRoot() == BT::NodeStatus::SUCCESS);
  REQUIRE(logs.size() == 14);
  CHECK(logs[0] == "Started [Repeat]");
  CHECK(logs[1] == "Started [Sequence]");
  CHECK(logs[2] == "Started [first]");
  CHECK(starts_with(logs[3], "Finished [first] elapsed_time: "));
  CHECK(logs[4] == "Started [second]");
  CHECK(starts_with(logs[5], "Finished [second] elapsed_time: "));
  CHECK(starts_with(logs[6], "Finished [Sequence] elapsed_time: "));
  CHECK(logs[7] == "Started [Sequence]");
  CHECK(logs[8] == "Started [first]");
  CHECK(starts_with(logs[9], "Finished [first] elapsed_time: "));
  CHECK(logs[10] == "Started [second]");
  CHECK(starts_with(logs[11], "Finished [second] elapsed_time: "));
  CHECK(starts_with(logs[12], "Finished [Sequence] elapsed_time: "));
  CHECK(starts_with(logs[13], "Finished [Repeat] elapsed_time: "));

  auto report = bt_logging.generate_report();
  REQUIRE(report.total_elasped_time.size() == 4);
  CHECK(report.total_elasped_time[0].node_name == "Repeat");
  CHECK(report.total_elasped_time[1].node_name == "Sequence");
  CHECK(report.total_elasped_time[2].node_name == "first");
  CHECK(report.total_elasped_time[3].node_name == "second");

  std::cerr << ReportConverter::to_string(report) << std::endl;
}

TEST_CASE("configure_logging") {
  RosFixture fixture;
  int level = 0;

  SECTION("should be cAsE iNsEnSiTiVe") {
    configure_logging(fixture.node, "dEbUg");
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    CHECK(level == RCUTILS_LOG_SEVERITY_DEBUG);

    configure_logging(fixture.node, "InFo");
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    CHECK(level == RCUTILS_LOG_SEVERITY_INFO);

    configure_logging(fixture.node, "wArN");
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    CHECK(level == RCUTILS_LOG_SEVERITY_WARN);

    configure_logging(fixture.node, "ErRoR");
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    CHECK(level == RCUTILS_LOG_SEVERITY_ERROR);

    configure_logging(fixture.node, "fAtAl");
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    CHECK(level == RCUTILS_LOG_SEVERITY_FATAL);
  }

  SECTION("smoke test for non-existing env var") {
    REQUIRE(std::getenv("NEXUS_LOG_LEVEL") == nullptr);
    fixture.node->get_logger().set_level(rclcpp::Logger::Level::Fatal);
    configure_logging(fixture.node);
    level = rcutils_logging_get_logger_level(
      fixture.node->get_logger().get_name());
    // no changes should be made
    CHECK(level == RCUTILS_LOG_SEVERITY_FATAL);
  }
}

}
