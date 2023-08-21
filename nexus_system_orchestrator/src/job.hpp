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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__JOB_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__JOB_HPP

#include "context.hpp"

#include <nexus_common/logging.hpp>
#include <nexus_orchestrator_msgs/msg/work_order_state.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

namespace nexus::system_orchestrator {

class Job
{
public: BT::Tree bt;
public: std::shared_ptr<Context> ctx;
public: uint8_t state =
    nexus_orchestrator_msgs::msg::WorkOrderState::STATE_NONE;
public: std::unique_ptr<common::BtLogging> bt_logging;
};

}

#endif
