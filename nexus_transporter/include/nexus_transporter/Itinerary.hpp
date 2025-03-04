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

#ifndef NEXUS_TRANSPORTER__ITINERARY_HPP
#define NEXUS_TRANSPORTER__ITINERARY_HPP

#include <vector>

#include <nexus_transporter_msgs/msg/destination.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <yaml-cpp/yaml.h>

//==============================================================================
namespace nexus_transporter {

using Destination = nexus_transporter_msgs::msg::Destination;

/// An itinerary that is generated by the Transporter.
class Itinerary
{
public:

  /// Constructor
  /// \param[in] id
  ///   A unique id for this itinerary
  ///
  /// \param[in] destinations
  ///   The destinations for this itinerary
  ///
  /// \param[in] transporter_name
  ///   The name of the transporter that is assigned to this itinerary
  ///
  /// \param[in] estimated_finish_time
  ///   The estimated finish time for this itinerary
  ///
  /// \param[in] expiration_time
  ///   The time until when this itinerary is valid
  Itinerary(
    std::string id,
    std::vector<Destination> destinations,
    std::string transporter_name,
    rclcpp::Time estimated_finish_time,
    rclcpp::Time expiration_time
  );

  /// Get the id
  const std::string& id() const;

  /// Set the id
  Itinerary& id(std::string id);

  /// Get the destinations
  const std::vector<Destination>& destinations() const;

  /// Set the destinations
  Itinerary& destinations(std::vector<Destination> destination);

  /// Get the name of the transporter
  const std::string& transporter_name() const;

  /// Set the name of the transporter
  Itinerary& transporter_name(std::string name);

  /// Get the estimated finish time
  const rclcpp::Time& estimated_finish_time() const;

  /// Set the estimated finish time
  Itinerary& estimated_finish_time(rclcpp::Time time);

  /// Get the expiration time
  const rclcpp::Time& expiration_time() const;

  /// Set the expiration time
  Itinerary& expiration_time(rclcpp::Time time);

  class Implementation;
private:
  Itinerary();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace nexus_transporter

#endif // NEXUS_TRANSPORTER__ITINERARY_HPP
