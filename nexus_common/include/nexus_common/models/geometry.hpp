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

#ifndef NEXUS_COMMON__MODELS__GEOMETRY_HPP
#define NEXUS_COMMON__MODELS__GEOMETRY_HPP

#include "../yaml_helpers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace nexus::common {

struct Point2D
{
  double x;
  double y;
};

using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Pose = geometry_msgs::msg::Pose;

}

namespace YAML {

template<>
struct convert<nexus::common::Point2D>
{
  static Node encode(const nexus::common::Point2D& point)
  {
    Node node;
    node["x"] = point.x;
    node["y"] = point.y;
    return node;
  }

  static bool decode(const Node& node, nexus::common::Point2D& point)
  {
    using nexus::common::YamlHelpers;
    YamlHelpers::write(node["x"], point.x);
    YamlHelpers::write(node["y"], point.y);
    return true;
  }
};

template<>
struct convert<nexus::common::Point>
{
  static Node encode(const nexus::common::Point& point)
  {
    Node node;
    node["x"] = point.x;
    node["y"] = point.y;
    node["z"] = point.z;
    return node;
  }

  static bool decode(const Node& node, nexus::common::Point& point)
  {
    using nexus::common::YamlHelpers;
    YamlHelpers::write(node["x"], point.x);
    YamlHelpers::write(node["y"], point.y);
    YamlHelpers::write(node["z"], point.z);
    return true;
  }
};

template<>
struct convert<geometry_msgs::msg::Quaternion>
{
  static Node encode(const geometry_msgs::msg::Quaternion& quat)
  {
    Node node;
    node["x"] = quat.x;
    node["y"] = quat.y;
    node["z"] = quat.z;
    node["w"] = quat.w;
    return node;
  }

  static bool decode(const Node& node, geometry_msgs::msg::Quaternion& quat)
  {
    using nexus::common::YamlHelpers;
    YamlHelpers::write(node["x"], quat.x);
    YamlHelpers::write(node["y"], quat.y);
    YamlHelpers::write(node["z"], quat.z);
    YamlHelpers::write(node["w"], quat.w);
    return true;
  }
};

template<>
struct convert<nexus::common::Pose>
{
  static Node encode(const nexus::common::Pose& data)
  {
    Node node;
    node["x"] = data.position.x;
    node["y"] = data.position.y;
    node["z"] = data.position.z;
    node["qx"] = data .orientation.x;
    node["qy"] = data .orientation.y;
    node["qz"] = data .orientation.z;
    node["qw"] = data .orientation.w;
    return node;
  }

  static bool decode(const Node& node, nexus::common::Pose& data)
  {
    using nexus::common::YamlHelpers;
    YamlHelpers::write(node["x"], data.position.x);
    YamlHelpers::write(node["y"], data.position.y);
    YamlHelpers::write(node["z"], data.position.z);
    YamlHelpers::write(node["qx"], data.orientation.x);
    YamlHelpers::write(node["qy"], data.orientation.y);
    YamlHelpers::write(node["qz"], data.orientation.z);
    YamlHelpers::write(node["qw"], data.orientation.w);
    return true;
  }
};

}

#endif
