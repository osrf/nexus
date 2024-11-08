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

#include <nexus_transporter/Itinerary.hpp>

//==============================================================================
namespace nexus_transporter {

//==============================================================================
class Itinerary::Implementation
{
public:
  // Documentation same as constructor input params
  std::string id;
  std::string destination;
  std::string transporter_name;
  rclcpp::Time finish_time;
  rclcpp::Time expiration_time;
  std::string source;
};

//==============================================================================
const std::string& Itinerary::id() const
{
  return _pimpl->id;
}

//==============================================================================
Itinerary& Itinerary::id(std::string id)
{
  _pimpl->id = std::move(id);
  return *this;
}

//==============================================================================
const std::string& Itinerary::source() const
{
  return _pimpl->source;
}

//==============================================================================
Itinerary& Itinerary::source(std::string source)
{
  _pimpl->source= std::move(source);
  return *this;
}

//==============================================================================
const std::string& Itinerary::destination() const
{
  return _pimpl->destination;
}

//==============================================================================
Itinerary& Itinerary::destination(std::string destination)
{
  _pimpl->destination = std::move(destination);
  return *this;
}

//==============================================================================
const std::string& Itinerary::transporter_name() const
{
  return _pimpl->transporter_name;
}

//==============================================================================
Itinerary& Itinerary::transporter_name(std::string transporter_name)
{
  _pimpl->transporter_name = std::move(transporter_name);
  return *this;
}

//==============================================================================
const rclcpp::Time& Itinerary::estimated_finish_time() const
{
  return _pimpl->finish_time;
}

//==============================================================================
Itinerary& Itinerary::estimated_finish_time(rclcpp::Time time)
{
  _pimpl->finish_time = std::move(time);
  return *this;
}

//==============================================================================
const rclcpp::Time& Itinerary::expiration_time() const
{
  return _pimpl->expiration_time;
}

//==============================================================================
Itinerary& Itinerary::expiration_time(rclcpp::Time time)
{
  _pimpl->expiration_time = std::move(time);
  return *this;
}

//==============================================================================
Itinerary::Itinerary(
  std::string id,
  std::string destination,
  std::string transporter_name,
  rclcpp::Time estimated_finish_time,
  rclcpp::Time expiration_time,
  std::string source)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(id),
        std::move(destination),
        std::move(transporter_name),
        std::move(estimated_finish_time),
        std::move(expiration_time),
        std::move(source),
      }))
{
  // Do nothing
}

} // namespace nexus_transporter
