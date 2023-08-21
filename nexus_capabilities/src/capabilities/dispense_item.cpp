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

#include "dispense_item.hpp"

namespace nexus::capabilities {

//==============================================================================
rclcpp::Client<endpoints::DispenserService::ServiceType>::SharedPtr
DispenseItem::client()
{
  auto item = this->getInput<std::string>("item");
  if (!item)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [item] is required",
      this->name().c_str());
    return nullptr;
  }
  auto dispenser_session = this->_dispensers.at(0);
  auto maybe_dispenser = this->getInput<std::string>("dispenser");
  if (maybe_dispenser)
  {
    const auto it = std::find_if(
      this->_dispensers.begin(), this->_dispensers.end(),
      [&maybe_dispenser](const DispenserSession& sess)
      {
        return sess.dispenser_id == *maybe_dispenser;
      });
    if (it == this->_dispensers.end())
    {
      RCLCPP_ERROR(this->_logger, "%s: dispenser [%s] is not registered",
        this->name().c_str(), maybe_dispenser->c_str());
      return nullptr;
    }
  }
  return dispenser_session.client;
}

//==============================================================================
endpoints::DispenserService::ServiceType::Request::SharedPtr DispenseItem::
make_request()
{
  auto node = this->_w_node.lock();
  auto req =
    std::make_shared<endpoints::DispenserService::ServiceType::Request>();
  req->requester = node->get_name();
  // TODO(koonpeng): Fill in data and payload
  return req;
}

//==============================================================================
bool DispenseItem::on_response(
  rclcpp::Client<endpoints::DispenserService::ServiceType>::SharedResponse resp)
{
  return resp->success;
}

} // namespace nexus::capabilities
