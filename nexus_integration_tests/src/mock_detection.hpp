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

#include <memory>

#include <nexus_endpoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <yaml-cpp/yaml.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::literals;

class DetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using DetectorService = nexus::endpoints::DetectorService;
  using ServiceType = DetectorService::ServiceType;

  explicit DetectorNode(
    const rclcpp::NodeOptions& options,
    const std::string& _configFile = "")
  : rclcpp_lifecycle::LifecycleNode("mock_detector_node", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(
      this->get_logger(),
      "Mock Detector is running..."
    );

    configfile_ = this->declare_parameter("config_file", _configFile);
    frame_id_ = this->declare_parameter("frame_id", "camera_color_frame");

    bool autostart = this->declare_parameter<bool>("autostart", false);
    // If 'autostart' parameter is true, the node self-transitions to 'active' state upon startup
    if (autostart)
    {
      this->configure();
      this->activate();
    }
  }

  ~DetectorNode()
  {
    if (this->get_current_state().label() != "unconfigured"
      && this->get_current_state().label() != "finalized")
    {
      on_cleanup(this->get_current_state());
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override
  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Mock Detector configuring...");

    if (!configfile_.empty())
    {
      read_yaml();
    }
    else
    {
      add_entry("sku_id1", 0.1, 0.2, 1.0);
      add_entry("sku_id2", 0.1, 0.6, 1.0);
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
        if (this->get_current_state().label() != "active")
        {
          RCLCPP_ERROR(this->get_logger(), "Mock Detector is not activated. "
          "Rejecting incoming request %s", request->id.c_str());
          return;
        }
        RCLCPP_INFO(
          this->get_logger(), "Incoming request %s", request->id.c_str());

        // YV: Special case for internal testing
        if (request->id == this->get_name())
        {
          // Send all SKUs
          auto& detection_array = response->detections;
          for (const auto& [_, data] : map_)
          {
            detection_array.detections.insert(
              detection_array.detections.end(),
              data->detection_array.detections.begin(),
              data->detection_array.detections.end()
            );
          }
          std::vector<geometry_msgs::msg::TransformStamped> tf_msgs = {};
          fill_tf_msgs(detection_array, tf_msgs);
          publish_transforms(tf_msgs);
          return;
        }
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

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock Detector activating...");

    RCLCPP_INFO(this->get_logger(), "Mock Detector activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock Detector deactivating...");

    RCLCPP_INFO(this->get_logger(), "Mock Detector deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock Detector cleaning up...");

    srv_.reset();
    tf_broadcaster_.reset();
    this->map_.clear();

    RCLCPP_INFO(this->get_logger(), "Mock Detector cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock Detector shutting down...");

    RCLCPP_INFO(this->get_logger(), "Mock Detector has shut down");
    return CallbackReturn::SUCCESS;
  }

public:
  void read_yaml()
  {
    try
    {
      YAML::Node node = YAML::LoadFile(configfile_);

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
          add_entry(sku_id, x, y, z, qx, qy, qz, qw);
        }
      }
    }
    catch (...)
    {
      RCLCPP_WARN(
        this->get_logger(), "Error reading config file [%s]",
        configfile_.c_str());
    }
  }

private:

  struct DetectionData
  {
    vision_msgs::msg::Detection3DArray detection_array;
    std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
  };

  rclcpp::Service<ServiceType>::SharedPtr srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unordered_map<std::string, std::shared_ptr<DetectionData>> map_;
  std::string frame_id_;
  std::string configfile_;

  void add_entry(
    const std::string& id,
    double x, double y, double z,
    double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
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

  void fill_tf_msgs(
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
      tf.transform.translation.x = detection.bbox.center.position.x;
      tf.transform.translation.y = detection.bbox.center.position.y;
      tf.transform.translation.z = detection.bbox.center.position.z;
      tf.transform.rotation = detection.bbox.center.orientation;
      tf_msgs.push_back(std::move(tf));
    }
  }

  void publish_transforms(
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
};
