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

#include <rmf_utils/catch.hpp>

#include "mock_detection.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <rcpputils/filesystem_helper.hpp>

//==============================================================================
SCENARIO("Test Detection Mock")
{
  rclcpp::init(0, nullptr);

  std::string package_prefix;
  try
  {
    package_prefix =
      ament_index_cpp::get_package_prefix("nexus_demos");
  }
  catch (ament_index_cpp::PackageNotFoundError& e)
  {
    throw std::runtime_error(e.what());
  }

  auto path = rcpputils::fs::path(package_prefix) / "share" /
    "nexus_demos" / "config" / "detection.yaml";

  auto node = std::make_shared<DetectorNode>(
    rclcpp::NodeOptions(), path.string());

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

  CHECK(0 == result.get()->detections.detections.size());

  auto request2 =
    std::make_shared<nexus::endpoints::DetectorService::ServiceType::Request>();
  request2->id = "sku_id1";

  auto result2 = client->async_send_request(request2);

  auto ret2 = rclcpp::spin_until_future_complete(node, result2, 5s);  // Wait for the result.
  CHECK(ret2 == rclcpp::FutureReturnCode::SUCCESS);

  auto response = result2.get();

  CHECK(1 == response->detections.detections.size());
  CHECK("camera_color_frame" == response->detections.header.frame_id);
  CHECK("sku_id1" == response->detections.detections[0].id);
  CHECK(2 == response->detections.detections[0].results.size());
  CHECK("id1" ==
    response->detections.detections[0].results[0].hypothesis.class_id);
  CHECK(0.2 ==
    response->detections.detections[0].results[0].hypothesis.score);
  CHECK(2.0 ==
    response->detections.detections[0].results[0].pose.pose.position.x);
  CHECK(2.1 ==
    response->detections.detections[0].results[0].pose.pose.position.y);
  CHECK(2.2 ==
    response->detections.detections[0].results[0].pose.pose.position.z);
  CHECK(0.707 ==
    response->detections.detections[0].results[0].pose.pose.orientation.x);
  CHECK(0.0 ==
    response->detections.detections[0].results[0].pose.pose.orientation.y);
  CHECK(0.707 ==
    response->detections.detections[0].results[0].pose.pose.orientation.z);
  CHECK(1.0 ==
    response->detections.detections[0].results[0].pose.pose.orientation.w);

  CHECK("id1" ==
    response->detections.detections[0].results[1].hypothesis.class_id);
  CHECK(0.8 ==
    response->detections.detections[0].results[1].hypothesis.score);
  CHECK(4.5 ==
    response->detections.detections[0].results[1].pose.pose.position.x);
  CHECK(5.4 ==
    response->detections.detections[0].results[1].pose.pose.position.y);
  CHECK(3.7 ==
    response->detections.detections[0].results[1].pose.pose.position.z);
  CHECK(0.0 ==
    response->detections.detections[0].results[1].pose.pose.orientation.x);
  CHECK(0.0 ==
    response->detections.detections[0].results[1].pose.pose.orientation.y);
  CHECK(0.0 ==
    response->detections.detections[0].results[1].pose.pose.orientation.z);
  CHECK(1.0 ==
    response->detections.detections[0].results[1].pose.pose.orientation.w);

  rclcpp::shutdown();
}
