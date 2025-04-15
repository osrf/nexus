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

#ifndef NEXUS_COMMON__LOGGING
#define NEXUS_COMMON__LOGGING

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "nexus_common_export.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace nexus::common {

template<typename NodePtrT>
class RclcppInfoLogF
{
  // no explicit to support implicit conversion
public: RclcppInfoLogF(NodePtrT node)
  : _node(node)
  {}

public: void operator()(const std::string& s) const
  {
    RCLCPP_INFO_STREAM(this->_node->get_logger(), s);
  }

private: NodePtrT _node;
};

/**
 * Registers status change callbacks on all nodes in a BT to
 * print node statuses using RCLCPP_* macros.
 *
 * @tparam LogFunctor A functor that should perform the logging.
 */
class BtLogging
{
public: class LogReport
  {
  public: struct NodeRecord
    {
      uint16_t node_id;
      std::string node_name;
    };

  public: struct ElapsedTimeRecord : public NodeRecord
    {
      BT::Duration elapsed_time;
    };

  public: std::vector<ElapsedTimeRecord> total_elasped_time;
  };

private: struct _NodeState
  {
    BT::TimePoint start_time;
    BT::Duration total_elapsed = std::chrono::seconds(0);
  };

public: template<typename LogFunctor,
    std::enable_if_t<std::is_invocable_v<LogFunctor,
    const std::string&>, bool> = true>
  BtLogging(
    // We don't really need a non-const `BT::Tree` but specifying it anyway to make it
    // more clear that it does not take ownership.
    BT::Tree& bt,
    LogFunctor log_f,
    const std::vector<std::string>& blocklist_nodes = {})
  : _bt(bt)
  {
    this->_node_states.reserve(bt.nodes.size());
    std::unordered_set<std::string> blocklist_set;
    for (const std::string& bn : blocklist_nodes)
    {
      blocklist_set.insert(bn);
    }

    this->_bt_subs.reserve(bt.nodes.size());
    for (const auto& n : bt.nodes)
    {
      if (blocklist_set.count(n->name()) > 0)
      {
        // If the node name is part of the blocklist, don't register a cb
        // to log status changes.
        continue;
      }
      this->_bt_subs.emplace_back(n->subscribeToStatusChange([this,
        log_f](BT::TimePoint now,
        const BT::TreeNode& bt_node,
        BT::NodeStatus prev_status, BT::NodeStatus new_status)
        {
          std::ostringstream oss;

          if (prev_status == BT::NodeStatus::IDLE &&
          new_status == BT::NodeStatus::RUNNING)
          {
            this->_node_states[bt_node.UID()].start_time = now;
            oss << "Started [" << bt_node.name() << "]";
            log_f(oss.str());
            return;
          }

          if (prev_status == BT::NodeStatus::RUNNING &&
          new_status != BT::NodeStatus::RUNNING)
          {
            auto elapsed = BT::TimePoint::clock::now() -
            this->_node_states[bt_node.UID()].start_time;
            auto& state = this->_node_states[bt_node.UID()];
            state.total_elapsed += elapsed;
            std::string verb = "Finished";
            if (new_status == BT::NodeStatus::FAILURE)
            {
              verb = "Failed";
            }
            oss << verb << " [" << bt_node.name() << "] elapsed_time: " <<
              std::fixed << std::setprecision(3) <<
              std::chrono::duration<double>(elapsed).count() << "s";
            log_f(oss.str());
            return;
          }
        }));
    }
  }

/**
 * @copydoc BtLogging::BtLogging
 */
public: template<typename NodePtrT,
    std::enable_if_t<!std::is_invocable_v<NodePtrT,
    const std::string&>, bool> = true>
  BtLogging(
    BT::Tree& bt, NodePtrT node,
    const std::vector<std::string>& blocklist_nodes = {})
  : BtLogging(bt, RclcppInfoLogF<NodePtrT>(node), blocklist_nodes)
  {}

/**
 * Returns a report containing the elapsed time of each node, keyed by their name.
 *   ```
 *   node_1: 1.0 # in secs
 *   node_2: 2.0
 *   node_3: 3.0
 *   ```
 */
public: LogReport generate_report()
  {
    LogReport report;
    for (const auto& n : this->_bt.nodes)
    {
      const auto& state = this->_node_states[n->UID()];
      report.total_elasped_time.emplace_back(
        LogReport::ElapsedTimeRecord{{n->UID(),
          n->name()}, state.total_elapsed});
    }
    return report;
  }

private: const BT::Tree& _bt;
private: std::unordered_map<uint16_t, _NodeState> _node_states;
private: std::vector<BT::TreeNode::StatusChangeSubscriber> _bt_subs;
};

class NEXUS_COMMON_EXPORT ReportConverter
{
  /**
   * Returns a formatted reported.
   */
public: static std::string to_string(const BtLogging::LogReport& report);
};


/**
 * Configures a node's logger.
 *
 * @param log_level The log level to set to, if not provided it will be read from
 * NEXUS_LOG_LEVEL environment variable. If the log level is invalid, it will be ignored.
 *
 * Note: ROS Iron only supports externally setting log level using the
 * `--log-level` CLI flag.
 * This function allows it to be set via an environment variable
 * `NEXUS_LOG_LEVEl`.
 */
template<typename NodePtrT>
void configure_logging(NodePtrT node,
  const std::optional<std::string>& log_level = std::nullopt)
{
  std::string log_level_str;
  if (!log_level.has_value())
  {
    const char* log_level_cstr = std::getenv("NEXUS_LOG_LEVEL");
    if (log_level_cstr != nullptr)
    {
      log_level_str = log_level_cstr;
    }
  }
  else
  {
    log_level_str = *log_level;
  }

  if (log_level_str == "")
  {
    // use the default
    return;
  }
  std::transform(log_level_str.begin(), log_level_str.end(),
    log_level_str.begin(), [](char c)
    {
      return std::tolower(c);
    });

  if (log_level_str == "debug")
  {
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
  else if (log_level_str == "info")
  {
    node->get_logger().set_level(rclcpp::Logger::Level::Info);
  }
  else if (log_level_str == "warn")
  {
    node->get_logger().set_level(rclcpp::Logger::Level::Warn);
  }
  else if (log_level_str == "error")
  {
    node->get_logger().set_level(rclcpp::Logger::Level::Error);
  }
  else if (log_level_str == "fatal")
  {
    node->get_logger().set_level(rclcpp::Logger::Level::Fatal);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Unknown log level [%s]",
      log_level->c_str());
  }

  return;
}

}

#endif
