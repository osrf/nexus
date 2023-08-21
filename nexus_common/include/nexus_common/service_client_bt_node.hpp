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


#ifndef NEXUS_COMMON__SERVICE_CLIENT_BT_NODE_HPP
#define NEXUS_COMMON__SERVICE_CLIENT_BT_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <future>
#include <optional>
#include <memory>
#include <string>

namespace nexus::common {

/**
 * A BT service node that sends a goal to an service server.
 *
 * @tparam NodeT The type of the ROS node.
 * @tparam ServiceT Type of the service.
 *
 * IMPORTANT: Be careful when halting a behavior tree that contains this node on the ros thread.
 * Because a halt must be synchronous, this will wait for the result of the service.
 * This may result in a deadlock if the ros executor and client is not properly configured.
 */
template<typename ServiceT>
class ServiceClientBtNode : public BT::StatefulActionNode
{
  /**
   * @param name The name of the BT node.
   * @param logger rclcpp logger to use for logging.
   * @param timeout The amount of time to wait for a response.
   * @param discovery_timeout The amount of time to wait to discover the service.
   */
public: ServiceClientBtNode(const std::string& name,
    const BT::NodeConfiguration& config, rclcpp::Logger logger,
    std::chrono::milliseconds timeout = std::chrono::milliseconds{5000},
    std::chrono::milliseconds discovery_timeout = std::chrono::milliseconds{5000});

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override;

protected: rclcpp::Logger _logger;

/**
 * Returns the client to use.
 */
protected: virtual typename rclcpp::Client<ServiceT>::SharedPtr client() = 0;

/**
 * Returns the request to send to the server.
 */
protected: virtual typename ServiceT::Request::SharedPtr make_request() = 0;

/**
 * Returns `true` if the response is a success.
 */
protected: virtual bool on_response(
    typename rclcpp::Client<ServiceT>::SharedResponse) = 0;

private: typename rclcpp::Client<ServiceT>::SharedFuture _fut;
private: std::chrono::milliseconds _timeout;
private: std::chrono::milliseconds _discovery_timeout;
private: typename rclcpp::Client<ServiceT>::SharedPtr _client;
private: std::chrono::steady_clock::time_point _timeout_limit;
private: int64_t _ongoing_request_id;

private: void _cleanup();
};

/******************
 * Implementation *
 ******************/

template<typename ServiceType>
ServiceClientBtNode<ServiceType>::ServiceClientBtNode(
  const std::string& name, const BT::NodeConfiguration& config,
  rclcpp::Logger logger,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds discovery_timeout)
: BT::StatefulActionNode{name, config}, _logger{logger},
  _timeout{timeout}, _discovery_timeout{discovery_timeout}
{
}

template<typename ServiceType>
BT::NodeStatus ServiceClientBtNode<ServiceType>::onStart()
{
  this->_client = this->client();
  if (!this->_client)
  {
    RCLCPP_ERROR(
      this->_logger, "%s: Invalid service client!",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!this->_client->wait_for_service(this->_discovery_timeout))
  {
    RCLCPP_ERROR(
      this->_logger, "%s: Timeout discovering server!",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto req = this->make_request();
  if (!req)
  {
    RCLCPP_ERROR(this->_logger, "%s: Failed to create request",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto fut = this->_client->async_send_request(this->make_request());
  this->_ongoing_request_id = fut.request_id;
  this->_fut = fut.share();
  this->_timeout_limit = std::chrono::steady_clock::now() + this->_timeout;

  return BT::NodeStatus::RUNNING;
}

template<typename ServiceType>
BT::NodeStatus ServiceClientBtNode<ServiceType>::onRunning()
{
  if (std::chrono::steady_clock::now() > this->_timeout_limit)
  {
    RCLCPP_ERROR(
      this->_logger, "%s: Timeout waiting for response",
      this->name().c_str());
    this->_cleanup();
    return BT::NodeStatus::FAILURE;
  }

  if (this->_fut.wait_for(std::chrono::seconds{0}) == std::future_status::ready)
  {
    RCLCPP_DEBUG(
      this->_logger, "%s: Service call finished successfully",
      this->name().c_str());

    auto resp = this->_fut.get();
    this->_cleanup();
    if (!this->on_response(resp))
    {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

template<typename ServiceType>
void ServiceClientBtNode<ServiceType>::onHalted()
{
  RCLCPP_INFO(this->_logger, "%s: Halting", this->name().c_str());
  this->_cleanup();
}

template<typename ServiceType>
void ServiceClientBtNode<ServiceType>::_cleanup()
{
  if (this->_fut.valid())
  {
    this->_client->remove_pending_request(this->_ongoing_request_id);
    this->_fut = decltype(this->_fut){};
  }
}

}

#endif
