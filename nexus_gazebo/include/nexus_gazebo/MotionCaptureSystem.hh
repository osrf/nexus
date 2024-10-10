// Copyright 2023 Johnson & Johnson
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

#ifndef NEXUS_GAZEBO__MOTIONCAPTURESYSTEM_HH_
#define NEXUS_GAZEBO__MOTIONCAPTURESYSTEM_HH_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

#include "vrpn_Connection.h"
#include "vrpn_ConnectionPtr.h"
#include "vrpn_Tracker.h"

namespace nexus_gazebo {

using Entity = gz::sim::Entity;
using EntityComponentManager = gz::sim::EntityComponentManager;
using EventManager = gz::sim::EventManager;
using ISystemConfigure = gz::sim::ISystemConfigure;
using ISystemPostUpdate = gz::sim::ISystemPostUpdate;
using Pose3d = gz::math::Pose3d;
using System = gz::sim::System;
using UpdateInfo = gz::sim::UpdateInfo;

class RigidBodyTracker : public vrpn_Tracker
{
public:
  explicit RigidBodyTracker(
    const std::string& _tracker_name,
    vrpn_Connection* _c = nullptr);

  ~RigidBodyTracker() override = default;

  void mainloop() override;

  void updatePose(
    const std::chrono::steady_clock::duration& _sim_time,
    const Pose3d& pose);

private:
  struct timeval timestamp;
  Pose3d pose;
};

/// \class MotionCaptureSystem
class MotionCaptureSystem :
  public System,
  public ISystemConfigure,
  public ISystemPostUpdate
{
public:
  /// \brief Constructor
  MotionCaptureSystem();

  /// \brief Destructor
  ~MotionCaptureSystem() override;

  /// Configure the system
  void Configure(
    const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& _eventMgr) override;

  /// Post-update callback
  void PostUpdate(
    const UpdateInfo& _info,
    const EntityComponentManager& _ecm) override;

private:
  int port {3883};
  bool stream_rigid_bodies {true};
  bool stream_labeled_markers {true};
  bool stream_unlabeled_markers {true};
  double update_frequency {50.0};

  /// Pointer to the VRPN connection
  vrpn_ConnectionPtr connection {nullptr};

  Entity entity {0};

  std::chrono::steady_clock::duration last_pose_pub_time {0};

  /// Collection of VRPN trackers
  std::unordered_map<Entity,
    std::unique_ptr<RigidBodyTracker>> trackers;
};

}  // namespace nexus_gazebo
#endif  // NEXUS_GAZEBO__MOTIONCAPTURESYSTEM_HH_
