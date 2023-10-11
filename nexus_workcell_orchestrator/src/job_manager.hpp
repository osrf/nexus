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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__JOB_MANAGER_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__JOB_MANAGER_HPP

#include "job.hpp"

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

namespace nexus::workcell_orchestrator {

/**
 * Manage execution of jobs according to the nexus task lifecycle design.
 */
class JobManager
{
public: using GoalHandlePtr =
    std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>;
public: using JobIterator = std::list<Job>::iterator;

public: JobManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    size_t max_concurrent)
  : _node(std::move(node)), _max_concurrent(max_concurrent) {}

public: std::pair<JobIterator, std::string> assign_task(
    const std::string& task_id);

public: std::string queue_task(const GoalHandlePtr& goal_handle,
    const std::shared_ptr<Context>& ctx, BT::Tree&& bt);

public: std::pair<JobIterator, std::string> remove_assigned_task(
    const std::string& task_id);

  /**
   * Forcefully stop and clear all jobs. Jobs will be stopped as soon as possible, unlike
   * `cancel_all_jobs`, the behavior trees cannot react to this event.
   */
public: void halt_all_jobs();

  /**
   * TODO: Implement on cancel bt node.
   * Request to cancel all jobs, the behavior trees of the jobs can react to this event
   * and stop the job gracefully.
   */
// public: void cancel_all_jobs();

  /**
   * Tick all active jobs.
   */
public: void tick();

public: const std::list<Job>& jobs() const
  {
    return this->_jobs;
  }

public: std::shared_ptr<const ContextManager> context_manager() const
  {
    return this->_ctx_mgr;
  }

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: size_t _max_concurrent;
private: std::list<Job> _jobs;
private: std::shared_ptr<ContextManager> _ctx_mgr;

private: void _tick_bt(Job& job);
private: uint8_t _tick_job(Job& job);

};

}

#endif
