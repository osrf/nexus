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

#ifndef NEXUS_GAZEBO__MOTIONCAPTURERIGIDBODY_HH_
#define NEXUS_GAZEBO__MOTIONCAPTURERIGIDBODY_HH_

#include <memory>
#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace nexus_gazebo {

using Entity = gz::sim::Entity;
using EntityComponentManager = gz::sim::EntityComponentManager;
using EventManager = gz::sim::EventManager;
using ISystemConfigure = gz::sim::ISystemConfigure;
using System = gz::sim::System;

/// \class MotionCaptureRigidBody
class MotionCaptureRigidBody :
  public System,
  public ISystemConfigure
{
public:
  MotionCaptureRigidBody();

  void Configure(
    const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& _eventMgr) override;

private:
  std::string rigid_body_label;
};
}  // namespace nexus_gazebo

#endif  // NEXUS_GAZEBO__MOTIONCAPTURERIGIDBODY_HH_
