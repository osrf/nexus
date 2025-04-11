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

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::literals;

namespace nexus_demos {

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
//==============================================================================
class MockDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
  using DetectorService = nexus::endpoints::DetectorService;
  using ServiceType = DetectorService::ServiceType;

  explicit MockDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override;

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
  std::string config_file_;

  void update_map(
    const std::string& id,
    double x, double y, double z,
    double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);

  void fill_tf_msgs(
    const vision_msgs::msg::Detection3DArray& detection_array,
    std::vector<geometry_msgs::msg::TransformStamped>& tf_msgs);

  void publish_transforms(
    const std::vector<geometry_msgs::msg::TransformStamped>& tf_msgs);
};
}  // namespace nexus_demos
