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

#ifndef NEXUS_COMMON__NODE_THREAD_HPP_
#define NEXUS_COMMON__NODE_THREAD_HPP_

#include "nexus_common_export.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace nexus::common {

/// \class nexus::common::NodeThread
/// \brief A background thread to process node/executor callbacks

class NEXUS_COMMON_EXPORT NodeThread
{
public:
  /// \brief A background thread to process node callbacks constructor
  /// \param[in] node_base Interface to Node to spin in thread
  explicit NodeThread(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /// \brief A background thread to process executor's callbacks constructor
  /// \param[in] executor Interface to executor to spin in thread
  explicit NodeThread(
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /// \brief A background thread to process node callbacks constructor
  /// \param[in] node Node pointer to spin in thread
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  /// \brief A destructor
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};

}  // namespace nexus::common

#endif  // NEXUS_COMMON__NODE_THREAD_HPP_
