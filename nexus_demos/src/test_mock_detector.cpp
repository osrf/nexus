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

#include <filesystem>

#include <rmf_utils/catch.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mock_detector.hpp"

//==============================================================================
SCENARIO("Test Detection Mock")
{
  rclcpp::init(0, nullptr);

  auto config_file = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nexus_demos")) / "config" /
    "mock_detector_config.yaml";

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("config_file", config_file);
  auto node = std::make_shared<nexus_demos::MockDetector>(std::move(node_options));

  node->configure();
  node->activate();

  auto client =
    node->create_client<nexus::endpoints::DetectorService::ServiceType>(
    nexus::endpoints::DetectorService::service_name(node->get_name()));
  if (!client->wait_for_service(20s))
  {
    REQUIRE(false);
  }

  auto request =
    std::make_shared<nexus::endpoints::DetectorService::ServiceType::Request>();
  request->id = "bad";

  auto result = client->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  CHECK(ret == rclcpp::FutureReturnCode::SUCCESS);
  auto response = result.get();
  CHECK(false == response->success);
  CHECK(0 == response->detections.detections.size());

  auto request2 =
    std::make_shared<nexus::endpoints::DetectorService::ServiceType::Request>();
  request2->id = "sku_id1";
  auto result2 = client->async_send_request(request2);
  auto ret2 = rclcpp::spin_until_future_complete(node, result2, 5s);  // Wait for the result.
  CHECK(ret2 == rclcpp::FutureReturnCode::SUCCESS);
  response = result2.get();

  CHECK(true == response->success);
  CHECK(2 == response->detections.detections.size());
  CHECK("camera" == response->detections.header.frame_id);
  CHECK("sku_id1" == response->detections.detections[0].id);
  CHECK(2.0 ==
    response->detections.detections[0].bbox.center.position.x);
  CHECK(2.1 ==
    response->detections.detections[0].bbox.center.position.y);
  CHECK(2.2 ==
    response->detections.detections[0].bbox.center.position.z);
  CHECK(0.707 ==
    response->detections.detections[0].bbox.center.orientation.x);
  CHECK(0.0 ==
    response->detections.detections[0].bbox.center.orientation.y);
  CHECK(0.707 ==
    response->detections.detections[0].bbox.center.orientation.z);
  CHECK(1.0 ==
    response->detections.detections[0].bbox.center.orientation.w);

  CHECK(4.5 ==
    response->detections.detections[1].bbox.center.position.x);
  CHECK(5.4 ==
    response->detections.detections[1].bbox.center.position.y);
  CHECK(3.7 ==
    response->detections.detections[1].bbox.center.position.z);
  CHECK(0.0 ==
    response->detections.detections[1].bbox.center.orientation.x);
  CHECK(0.0 ==
    response->detections.detections[1].bbox.center.orientation.y);
  CHECK(0.0 ==
    response->detections.detections[1].bbox.center.orientation.z);
  CHECK(1.0 ==
    response->detections.detections[1].bbox.center.orientation.w);

  rclcpp::shutdown();
}
