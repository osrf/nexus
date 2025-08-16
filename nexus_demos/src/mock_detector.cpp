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

#include "mock_detector.hpp"

#include <memory>

#include <yaml-cpp/yaml.h>

using namespace std::literals;

namespace nexus_demos {

//==============================================================================
MockDetector::MockDetector(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("mock_detector_node", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(
    this->get_logger(),
    "Mock Detector is running..."
  );

  config_file_ = this->declare_parameter("config_file", "");
  if (config_file_.empty())
  {
    throw std::runtime_error("Config file not provided via config_file param.");
  }

  bool autostart = this->declare_parameter<bool>("autostart", false);
  // If 'autostart' parameter is true, the node self-transitions to 'active' state upon startup
  if (autostart)
  {
    this->configure();
    this->activate();
  }
}

//==============================================================================
CallbackReturn MockDetector::on_configure(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock Detector configuring...");

  try
  {
    YAML::Node node = YAML::LoadFile(config_file_);

    this->frame_id_ = node["frame_id"].as<std::string>();

    for (YAML::const_iterator it = node["keys"].begin();
      it != node["keys"].end();
      ++it)
    {
      const std::string sku_id = it->as<std::string>();
      const std::size_t detected_units =
        node[sku_id]["detected_units"].as<std::size_t>();
      for (std::size_t i = 0; i < detected_units; ++i)
      {
        double x = node[sku_id]["position"][i][0].as<double>();
        double y = node[sku_id]["position"][i][1].as<double>();
        double z = node[sku_id]["position"][i][2].as<double>();
        double qx = node[sku_id]["rotation"][i][0].as<double>();
        double qy = node[sku_id]["rotation"][i][1].as<double>();
        double qz = node[sku_id]["rotation"][i][2].as<double>();
        double qw = node[sku_id]["rotation"][i][3].as<double>();
        update_map(sku_id, x, y, z, qx, qy, qz, qw);
      }
    }
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error while parsing config file: %s", e.what()
    );
    return CallbackReturn::FAILURE;
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create a service that will use the callback function to handle requests.
  srv_ = create_service<ServiceType>(
    DetectorService::service_name(this->get_name()),
    [this](
      const ServiceType::Request::ConstSharedPtr request,
      ServiceType::Response::SharedPtr response
    )
    {
      response->success = false;
      if (this->get_current_state().label() != "active")
      {
        RCLCPP_ERROR(this->get_logger(), "Mock Detector is not activated. "
        "Rejecting incoming request %s", request->id.c_str());
        return;
      }
      RCLCPP_INFO(
        this->get_logger(), "Incoming request %s", request->id.c_str());

      auto it = map_.find(request->id);
      if (it == map_.end())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "This detector is not configured to handle requests with id [%s]",
          request->id.c_str()
        );
        return;
      }
      response->detections = it->second->detection_array;
      response->success = true;
      publish_transforms(it->second->tf_msgs);
    });

  RCLCPP_INFO(this->get_logger(), "Mock Detector configured");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockDetector::on_activate(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock Detector activated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockDetector::on_deactivate(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock Detector deactivated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockDetector::on_cleanup(const rclcpp_lifecycle::State& previous)
{
  srv_.reset();
  tf_broadcaster_.reset();
  this->map_.clear();

  RCLCPP_INFO(this->get_logger(), "Mock Detector cleaned up.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CallbackReturn MockDetector::on_shutdown(const rclcpp_lifecycle::State& previous)
{
  RCLCPP_INFO(this->get_logger(), "Mock Detector has shutdown.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
void MockDetector::update_map(
  const std::string& id,
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  auto insertion = map_.insert({id, nullptr});
  if (insertion.second)
  {
    insertion.first->second = std::make_shared<DetectionData>();
  }
  auto& detection_array = insertion.first->second->detection_array;
  detection_array.header.frame_id = frame_id_;
  detection_array.header.stamp = this->get_clock()->now();
  vision_msgs::msg::Detection3D detection;
  detection.header = detection_array.header;
  detection.id = id;
  auto& bbox = detection.bbox;
  bbox.center.position.x = x;
  bbox.center.position.y = y;
  bbox.center.position.z = z;
  bbox.center.orientation.x = qx;
  bbox.center.orientation.y = qy;
  bbox.center.orientation.z = qz;
  bbox.center.orientation.w = qw;
  detection_array.detections.push_back(std::move(detection));
  fill_tf_msgs(detection_array, insertion.first->second->tf_msgs);
}

//==============================================================================
void MockDetector::fill_tf_msgs(
  const vision_msgs::msg::Detection3DArray& detection_array,
  std::vector<geometry_msgs::msg::TransformStamped>& tf_msgs)
{
  for (std::size_t i = 0; i < detection_array.detections.size(); ++i)
  {
    const auto& detection = detection_array.detections[i];
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = detection.id + "_" + std::to_string(i+1);
    tf.transform.translation.x = detection.bbox.center.position.x;
    tf.transform.translation.y = detection.bbox.center.position.y;
    tf.transform.translation.z = detection.bbox.center.position.z;
    tf.transform.rotation = detection.bbox.center.orientation;
    tf_msgs.push_back(std::move(tf));
  }
}

//==============================================================================
void MockDetector::publish_transforms(
  const std::vector<geometry_msgs::msg::TransformStamped>& tf_msgs)
{
  for (const auto& tf_msg : tf_msgs)
  {
    // TFs are time sensitive so we update the header stamp.
    auto _tf_msg = tf_msg;
    _tf_msg.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(std::move(_tf_msg));
  }
}

}  // namespace nexus_demos

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus_demos::MockDetector)
