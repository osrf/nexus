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

#ifndef NEXUS_COMMON__TEST_UTILS_HPP
#define NEXUS_COMMON__TEST_UTILS_HPP

#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <future>
#include <thread>

namespace nexus::common::test {

std::string unique_node_name(const std::string& prefix);

/**
 * Tick a BT until a predicate is successful.
 * @param pred A functor with the signature `bool f(BT::NodeStatus)`.
 * @param timeout Max time to tick for.
 * @return true if the predicate succeed before the timeout.
 */
template<typename Pred>
bool tick_until(BT::Tree& bt, Pred pred, std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < end)
  {
    auto result = bt.tickRoot();
    if (pred(result))
    {
      return true;
    }
    else if (result != BT::NodeStatus::RUNNING)
    {
      return false;
    }
  }
  // timeout
  return false;
}

inline bool tick_until_status(BT::Tree& bt, BT::NodeStatus status,
  std::chrono::milliseconds timeout)
{
  return tick_until(bt, [status](
        auto result) { return result == status; }, timeout);
}

template<typename NodeT = rclcpp::Node,
  typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class RosFixture
{
public: rclcpp::Context::SharedPtr context;
public: typename NodeT::SharedPtr node;
public: std::thread spin_thread;
public: typename ExecutorT::SharedPtr executor;

public: RosFixture();
public: ~RosFixture();
public: RosFixture(const RosFixture&) = delete;
public: RosFixture(RosFixture&&) = delete;
public: RosFixture& operator=(const RosFixture&) = delete;
public: RosFixture& operator=(RosFixture&&) = delete;

/**
 * Spin the node in a background thread.
 */
public: void spin_in_background();

/**
 * Stop spinning and wait for the thread to exit.
 */
public: void stop_spinning();
};

template<typename NodeT, typename ExecutorT>
RosFixture<NodeT, ExecutorT>::RosFixture()
: context{std::make_shared<rclcpp::Context>()}
{
  this->context->init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.context(this->context);
  this->node = std::make_shared<NodeT>(unique_node_name(
        "test"), std::move(opts));
  rclcpp::ExecutorOptions executor_opts;
  executor_opts.context = this->context;
  this->executor = std::make_shared<ExecutorT>(std::move(executor_opts));
  this->executor->add_node(this->node->get_node_base_interface());
}

template<typename NodeT, typename ExecutorT>
RosFixture<NodeT, ExecutorT>::~RosFixture()
{
  this->stop_spinning();
}

template<typename NodeT, typename ExecutorT>
void RosFixture<NodeT, ExecutorT>::spin_in_background()
{
  if (this->executor->is_spinning())
  {
    throw std::runtime_error{"already spinning"};
  }
  this->spin_thread = std::thread{[this]()
    {
      this->executor->spin();
    }};
}

template<typename NodeT, typename ExecutorT>
void RosFixture<NodeT, ExecutorT>::stop_spinning()
{
  this->executor->cancel();
  if (this->spin_thread.joinable())
  {
    this->spin_thread.join();
  }
}

}

#endif
