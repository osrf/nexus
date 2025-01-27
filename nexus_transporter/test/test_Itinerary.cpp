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

#include <nexus_transporter_msgs/msg/destination.hpp>

#include <rmf_utils/catch.hpp>

using Destination = nexus_transporter::Destination;
//==============================================================================
SCENARIO("Test Itinerary")
{
  const std::string id = "A123";
  std::vector<Destination> destinations;
  destinations.emplace_back(
    nexus_transporter_msgs::build<Destination>()
    .name("workcell_1")
    .action(Destination::ACTION_PICKUP)
    .params("")
  );
  const std::string transporter_name = "pallet_1";
  const auto& now = rclcpp::Clock().now();
  const rclcpp::Time finish_time =
    now + rclcpp::Duration::from_seconds(10.0);
  const rclcpp::Time expiry_time =
    now + rclcpp::Duration::from_seconds(5.0);
  auto itinerary = nexus_transporter::Itinerary(
    id,
    std::move(destinations),
    transporter_name,
    finish_time,
    expiry_time
  );

  CHECK(itinerary.id() == id);
  const auto & destinations_ = itinerary.destinations();
  CHECK(destinations_.size() == 1);
  CHECK(destinations_[0].name == "pallet_1");
  CHECK(destinations_[0].action == Destination::ACTION_PICKUP);
  CHECK(destinations_[0].params == "");
  CHECK(itinerary.transporter_name() == transporter_name);
  CHECK(itinerary.estimated_finish_time() == finish_time);
  CHECK(itinerary.expiration_time() == expiry_time);

  WHEN("Setting new id")
  {
    const std::string new_id = "B123";
    itinerary.id(new_id);
    CHECK(itinerary.id() == new_id);
  }
  WHEN("Setting new destination")
  {
    std::vector<Destination> new_destinations;
    new_destinations.emplace_back(
      nexus_transporter_msgs::build<Destination>()
      .name("workcell_2")
      .action(Destination::ACTION_DROPOFF)
      .params("")
    );
    itinerary.destinations(std::move(new_destinations));
    const auto & new_destinations_ = itinerary.destinations();
    CHECK(new_destinations_.size() == 1);
    CHECK(new_destinations_[0].name == "pallet_1");
    CHECK(new_destinations_[0].action == Destination::ACTION_PICKUP);
    CHECK(new_destinations_[0].params == "");
  }
  WHEN("Setting new transporter_name")
  {
    const std::string new_transporter_name = "pallet_2";
    itinerary.transporter_name(new_transporter_name);
    CHECK(itinerary.transporter_name() == new_transporter_name);
  }
  WHEN("Setting new finish time")
  {
    const rclcpp::Time new_time = now + rclcpp::Duration::from_seconds(30.0);
    itinerary.estimated_finish_time(new_time);
    CHECK(itinerary.estimated_finish_time() == new_time);
  }
  WHEN("Setting new expiration time")
  {
    const rclcpp::Time new_time = now + rclcpp::Duration::from_seconds(3.0);
    itinerary.expiration_time(new_time);
    CHECK(itinerary.expiration_time() == new_time);
  }
}
