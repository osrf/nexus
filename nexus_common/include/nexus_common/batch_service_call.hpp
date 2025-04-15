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

#ifndef NEXUS_COMMON__BATCH_SERVICE_CALL_HPP
#define NEXUS_COMMON__BATCH_SERVICE_CALL_HPP

#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace nexus::common {

class CancellationToken
{
public: CancellationToken(std::function<void()> on_cancel)
  : _on_cancel{std::move(on_cancel)} {}

public: void cancel()
  {
    this->_on_cancel();
  }

private: std::function<void()> _on_cancel;
};

template<typename ServiceT>
struct BatchServiceReq
{
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  typename ServiceT::Request::SharedPtr req;
};

template<typename ServiceT>
struct BatchServiceResult
{
  /**
   * Indicates if a response was received within the timeout.
   */
  bool success;
  typename ServiceT::Response::SharedPtr resp;
};

/**
 * Sends multiple service calls in parallel and wait for their response.
 * Precondition: All clients must be using the same callback group.
 * @param node The node that is used to create a timer to trigger the timeout.
 * @param batch_reqs Map of a key to identify the request to the request and client used to send that request.
 * @param timeout Timeout for each requests.
 * @param on_done Callback when all the requests have finished.
 */
template<typename NodePtrT, typename ServiceT, typename KeyT,
  typename DoneCallbackT = std::function<void(const std::unordered_map<KeyT,
  BatchServiceResult<ServiceT>>& )>>
CancellationToken batch_service_call(
  NodePtrT node,
  const std::unordered_map<KeyT, BatchServiceReq<ServiceT>>& batch_reqs,
  std::chrono::milliseconds timeout = std::chrono::milliseconds{5000},
  DoneCallbackT on_done = [] (const std::unordered_map<KeyT,
  BatchServiceResult<ServiceT>>&) {})
{
  struct _Session
  {
    size_t done_count = 0;
    std::unordered_map<KeyT, BatchServiceResult<ServiceT>> results;
    std::unordered_map<KeyT,
      typename rclcpp::Client<ServiceT>::SharedPtr> clients;
    std::unordered_map<KeyT,
      typename rclcpp::Client<ServiceT>::SharedFutureAndRequestId>
    pending_requests;
    rclcpp::TimerBase::SharedPtr timeout_timer;
  };
  auto session = std::make_shared<_Session>();

  for (const auto& p : batch_reqs)
  {
    const auto& k = p.first;
    const auto& req = p.second;
    session->clients.emplace(k, req.client);
    session->pending_requests.emplace(k,
      req.client->async_send_request(req.
      req,
      [on_done, session, k,
      batch_size =
      batch_reqs.size()](typename rclcpp::Client<ServiceT>::SharedFuture fut)
      {
        auto& result = session->results[k];
        result.success = true;
        result.resp = fut.get();
        session->pending_requests.erase(k);
        ++session->done_count;
        if (session->done_count >= batch_size)
        {
          if (session->timeout_timer)
          {
            session->timeout_timer->cancel();
          }
          on_done(session->results);
        }
      }));
  }

  session->timeout_timer = node->create_wall_timer(timeout, [session, on_done]()
      {
        session->timeout_timer->cancel();
        for (const auto& [k, req] : session->pending_requests)
        {
          session->clients[k]->remove_pending_request(req.request_id);
          auto& result = session->results[k];
          result.success = false;
        }
        on_done(session->results);
      });

  return CancellationToken{[session]()
    {
      for (const auto& [k, req] : session->pending_requests)
      {
        session->clients[k]->remove_pending_request(req.request_id);
      }
    }};
}

}

#endif
