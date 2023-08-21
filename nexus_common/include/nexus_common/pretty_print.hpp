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

#ifndef NEXUS_COMMON__PRETTY_PRINT_HPP
#define NEXUS_COMMON__PRETTY_PRINT_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.h>

#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

namespace nexus {

// inline std::ostream& operator<<(std::ostream& output,
//   const geometry_msgs::msg::Pose& pose);

// inline std::ostream& operator<<(std::ostream& output,
//   const tf2::Transform& tf);

inline std::ostream& operator<<(std::ostream& out,
  const geometry_msgs::msg::Pose& pose)
{
  tf2::Quaternion quat{pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w};
  auto axis = quat.getAxis();
  out << "[ position: (" << pose.position.x << "," << pose.position.y <<
    "," <<
    pose.position.z << ") orientation_quat: (" << pose.orientation.x << "," <<
    pose.orientation.y << "," << pose.orientation.z << "," <<
    pose.orientation.w << ") orientation_axis: (" << axis.x() << "," <<
    axis.y() << "," << axis.z() <<
    ") orientation_angle: " << quat.getAngle() << "rad ]";
  return out;
}

//==============================================================================
inline std::ostream& operator<<(std::ostream& out,
  const tf2::Transform& tf)
{
  auto translation = tf.getOrigin();
  auto rot = tf.getRotation();
  auto axis = rot.getAxis();
  out << "[ translation: (" << translation.x() << "," << translation.y() <<
    "," <<
    translation.z() << ") rotation_quat: (" << rot.x() << "," <<
    rot.y() << "," << rot.z() << "," <<
    rot.w() << ") rotation_axis: (" << axis.x() << "," <<
    axis.y() << "," << axis.z() <<
    ") rotation_angle: " << rot.getAngle() << "rad ]";
  return out;
}

} // namespace nexus

#endif // NEXUS_COMMON__PRETTY_PRINT_HPP
