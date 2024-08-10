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

#include "nexus_gazebo/MotionCaptureRigidBody.hh"
#include "nexus_gazebo/Components.hh"

#include <ignition/plugin/RegisterMore.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>

constexpr const char* kRigidBodyLabel = "rigid_body_label";

namespace nexus_gazebo {

//////////////////////////////////////////////////
MotionCaptureRigidBody::MotionCaptureRigidBody() = default;

void MotionCaptureRigidBody::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& /*_eventMgr*/)
{
  // Read SDF Configuration
  this->rigid_body_label = _sdf->Get<std::string>(
    kRigidBodyLabel, this->rigid_body_label).first;

  if (this->rigid_body_label.empty())
  {
    // In the case that the user hasn't overridden the label, then use
    // the name of the entity we are attached to.
    this->rigid_body_label =
      _ecm.Component<ignition::gazebo::components::Name>(_entity)->Data();
  }

  _ecm.CreateComponent<components::MotionCaptureRigidBody>(_entity,
    components::MotionCaptureRigidBody(this->rigid_body_label));
}
}  // namespace nexus_gazebo

IGNITION_ADD_PLUGIN(
  nexus_gazebo::MotionCaptureRigidBody,
  nexus_gazebo::System,
  nexus_gazebo::MotionCaptureRigidBody::ISystemConfigure);
