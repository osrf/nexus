/*
 * Copyright (C) 2023 Johnson & Johnson
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

#ifndef NEXUS_COMMON__MODELS__VISION_HPP
#define NEXUS_COMMON__MODELS__VISION_HPP

#include "../yaml_helpers.hpp"

#include <vision_msgs/msg/detection3_d_array.hpp>


namespace YAML {

template<>
struct convert<vision_msgs::msg::Detection3DArray>
{
  static bool decode(const Node& node, vision_msgs::msg::Detection3DArray& data)
  {
    using nexus::common::YamlHelpers;
    {
      const auto& s_header = node["header"];
      YamlHelpers::write(s_header["stamp"]["sec"], data.header.stamp.sec);
      YamlHelpers::write(s_header["stamp"]["nanosec"],
        data.header.stamp.nanosec);
      YamlHelpers::write(s_header["frame_id"], data.header.frame_id);
    }
    const auto& s_detections = node["detections"];
    for (const auto& s_detection : s_detections)
    {
      auto& detection = data.detections.emplace_back();
      const auto& s_header = s_detection["header"];
      YamlHelpers::write(s_header["stamp"]["sec"], detection.header.stamp.sec);
      YamlHelpers::write(s_header["stamp"]["nanosec"],
        detection.header.stamp.nanosec);
      YamlHelpers::write(s_header["frame_id"], detection.header.frame_id);
      for (const auto s_result : s_detection["results"])
      {
        auto& hypothesis = detection.results.emplace_back();
        YamlHelpers::write(s_result["hypothesis"]["class_id"],
          hypothesis.hypothesis.class_id);
        YamlHelpers::write(s_result["hypothesis"]["score"],
          hypothesis.hypothesis.score);
        auto& pose = hypothesis.pose;
        YamlHelpers::write(s_result["pose"]["pose"]["position"]["x"],
          pose.pose.position.x);
        YamlHelpers::write(s_result["pose"]["pose"]["position"]["y"],
          pose.pose.position.y);
        YamlHelpers::write(s_result["pose"]["pose"]["position"]["z"],
          pose.pose.position.z);
        YamlHelpers::write(s_result["pose"]["pose"]["orientation"]["x"],
          pose.pose.orientation.x);
        YamlHelpers::write(s_result["pose"]["pose"]["orientation"]["y"],
          pose.pose.orientation.y);
        YamlHelpers::write(s_result["pose"]["pose"]["orientation"]["z"],
          pose.pose.orientation.z);
        YamlHelpers::write(s_result["pose"]["pose"]["orientation"]["w"],
          pose.pose.orientation.w);
        YamlHelpers::write(s_result["pose"]["covariance"], pose.covariance);
      }
      const auto& s_bbox = s_detection["bbox"];
      auto& bbox = detection.bbox;
      YamlHelpers::write(s_bbox["center"]["position"]["x"],
        bbox.center.position.x);
      YamlHelpers::write(s_bbox["center"]["position"]["y"],
        bbox.center.position.y);
      YamlHelpers::write(s_bbox["center"]["position"]["z"],
        bbox.center.position.z);
      YamlHelpers::write(s_bbox["center"]["orientation"]["x"],
        bbox.center.orientation.x);
      YamlHelpers::write(s_bbox["center"]["orientation"]["y"],
        bbox.center.orientation.y);
      YamlHelpers::write(s_bbox["center"]["orientation"]["z"],
        bbox.center.orientation.z);
      YamlHelpers::write(s_bbox["center"]["orientation"]["w"],
        bbox.center.orientation.w);
      YamlHelpers::write(s_bbox["size"]["x"], bbox.size.x);
      YamlHelpers::write(s_bbox["size"]["y"], bbox.size.y);
      YamlHelpers::write(s_bbox["size"]["z"], bbox.size.z);
      YamlHelpers::write(s_detection["id"], detection.id);
    }
    return true;
  }
};

}

#endif // NEXUS_COMMON__MODELS__VISION_HPP
