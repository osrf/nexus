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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__FOR_EACH_TASK__HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__FOR_EACH_TASK__HPP

#include "context.hpp"
#include "session.hpp"

#include <nexus_common/models/work_order.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <rclcpp/logger.hpp>

#include <behaviortree_cpp_v3/decorator_node.h>

namespace nexus::system_orchestrator {

/**
 * Run the child node once for each task in a work order.
 *
 * Output Ports:
 *   task |nexus_orchestrator_msgs::msg::WorkcellTask| The current task being processed.
 *   workcell |std::string| The workcell id assigned to the task.
 */
class ForEachTask : public BT::DecoratorNode
{
  /**
   * Defines provided ports for the BT node.
    */
public: static BT::PortsList providedPorts();

/**
 * @param name name of the BT node.
 * @param config config of the BT node.
 * @param logger rclcpp logger to use.
 * @param tasks list of tasks in the work order.
 * @param assignments map of task id and the assigned workcells
 */
public: ForEachTask(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Logger logger,
    std::shared_ptr<Context> ctx)
  : BT::DecoratorNode{name, config}, _logger{logger}, _ctx{std::move(ctx)}
  {
  }

public: BT::NodeStatus tick() override;

private: size_t _current_idx = 0;
private: rclcpp::Logger _logger;
private: std::shared_ptr<Context> _ctx;
private: bool _first_tick = true;
};

}

#endif
