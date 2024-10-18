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

#ifndef NEXUS_COMMON__SYNC_SERVICE_CLIENT_HPP
#define NEXUS_COMMON__SYNC_SERVICE_CLIENT_HPP

#include "exceptions.hpp"
#include "node_utils.hpp"

#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rmw/qos_profiles.h>

namespace nexus::common {

template<typename ServiceT>
class SyncServiceClient
{
public: template<typename NodePtrT>
  SyncServiceClient(NodePtrT node, const std::string& service_name)
  : _executor(create_executor(node)),
    _cb_group(node->create_callback_group(rclcpp::CallbackGroupType::
      MutuallyExclusive, false)),
    _client(node->template create_client<ServiceT>(
        service_name, rclcpp::ServicesQoS(), this->_cb_group))
  {
    this->_executor.add_callback_group(this->_cb_group,
      node->get_node_base_interface());
  }

  /**
   * Synchronously calls a service.
   * @param timeout The amount of time to wait for a service response,
   *   this includes time spent in discovery.
   * @throws TimeoutError if timeout is reached while waiting for response
   */
public: typename ServiceT::Response::SharedPtr send_request(
    const typename ServiceT::Request::SharedPtr& request,
    const std::chrono::nanoseconds& timeout = std::chrono::milliseconds(1000))
  {
    const auto start = std::chrono::steady_clock::now();
    if (!this->_client->wait_for_service(timeout))
    {
      throw TimeoutError();
    }
    const auto remaining_timeout = start + timeout -
      std::chrono::steady_clock::now();
    auto fut = this->_client->async_send_request(request);
    if (this->_executor.spin_until_future_complete(fut,
      remaining_timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
      throw TimeoutError();
    }
    return fut.get();
  }

  /// Wait for a service to be ready.
  /**
   * \param timeout maximum time to wait
   * \return `true` if the service is ready and the timeout is not over, `false` otherwise
   */
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT,
    RatioT>(-1))
  {
    return this->_client->wait_for_service(std::move(timeout));
  }

private: rclcpp::executors::SingleThreadedExecutor _executor;
private: rclcpp::CallbackGroup::SharedPtr _cb_group;
private: typename rclcpp::Client<ServiceT>::SharedPtr _client;
};

}

#endif
