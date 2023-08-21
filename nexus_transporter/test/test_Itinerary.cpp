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

#include <rmf_utils/catch.hpp>

//==============================================================================
SCENARIO("Test Itinerary")
{
  const std::string id = "A123";
  const std::string destination = "workcell_1";
  const std::string transporter_name = "pallet_1";
  const auto& now = rclcpp::Clock().now();
  const rclcpp::Time finish_time =
    now + rclcpp::Duration::from_seconds(10.0);
  const rclcpp::Time expiry_time =
    now + rclcpp::Duration::from_seconds(5.0);
  auto itinerary = nexus_transporter::Itinerary(
    id,
    destination,
    transporter_name,
    finish_time,
    expiry_time
  );

  CHECK(itinerary.id() == id);
  CHECK(itinerary.destination() == destination);
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
    const std::string new_destination = "workcell_2";
    itinerary.destination(new_destination);
    CHECK(itinerary.destination() == new_destination);
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
