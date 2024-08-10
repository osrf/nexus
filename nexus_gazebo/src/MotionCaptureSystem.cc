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

#include "nexus_gazebo/MotionCaptureSystem.hh"
#include "nexus_gazebo/Components.hh"

#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

constexpr const char* kPort = "port";
constexpr const char* kSdfStreamRigid = "stream_rigid_bodies";
constexpr const char* kSdfStreamLabeled = "stream_labeled_markers";
constexpr const char* kSdfStreamUnlabeled = "stream_unlabeled_markers";
constexpr const char* kUpdateFrequency = "update_frequency";

namespace nexus_gazebo {

RigidBodyTracker::RigidBodyTracker(const std::string& _tracker_name,
  vrpn_Connection* c)
: vrpn_Tracker(_tracker_name.c_str(), c)
{
}

void RigidBodyTracker::mainloop()
{
  this->pos[0] = this->pose.X();
  this->pos[1] = this->pose.Y();
  this->pos[2] = this->pose.Z();

  this->d_quat[0] = this->pose.Rot().X();
  this->d_quat[1] = this->pose.Rot().Y();
  this->d_quat[2] = this->pose.Rot().Z();
  this->d_quat[3] = this->pose.Rot().W();

  std::array<char, 1000> msgbuf {};
  int len = vrpn_Tracker::encode_to(msgbuf.data());

  if (!static_cast<bool>(this->d_connection->connected()))
    return;

  if (0 > this->d_connection->pack_message(
      len,
      this->timestamp,
      position_m_id,
      d_sender_id, msgbuf.data(),
      vrpn_CONNECTION_LOW_LATENCY))
  {
    return;
  }

  server_mainloop();
}

void RigidBodyTracker::updatePose(
  const std::chrono::steady_clock::duration& _sim_time,
  const Pose3d& _pose)
{
  this->pose = _pose;
  this->timestamp.tv_sec = _sim_time.count() / 1000;
  this->timestamp.tv_usec = (_sim_time.count() % 1000) * 1000;
}

//////////////////////////////////////////////////
MotionCaptureSystem::MotionCaptureSystem() = default;

//////////////////////////////////////////////////
MotionCaptureSystem::~MotionCaptureSystem() = default;

//////////////////////////////////////////////////
void MotionCaptureSystem::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& /*_ecm*/,
  EventManager& /*_eventMgr*/)
{
  this->entity = _entity;

  // Read SDF configuration
  this->port = _sdf->Get<int>(kPort, this->port).first;
  this->stream_rigid_bodies =
    _sdf->Get<bool>(kSdfStreamRigid, this->stream_rigid_bodies).first;
  this->stream_labeled_markers =
    _sdf->Get<bool>(kSdfStreamLabeled, this->stream_labeled_markers).first;
  this->stream_unlabeled_markers =
    _sdf->Get<bool>(kSdfStreamUnlabeled, this->stream_unlabeled_markers).first;
  this->update_frequency =
    _sdf->Get<double>(kUpdateFrequency, this->update_frequency).first;

  // Configure the VRPN server
  this->connection = vrpn_ConnectionPtr::create_server_connection(this->port);
}

//////////////////////////////////////////////////
void MotionCaptureSystem::PostUpdate(
  const UpdateInfo& _info,
  const EntityComponentManager& _ecm)
{
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time" << std::endl;
  }

  if (_info.paused)
  {
    return;
  }

  auto diff = _info.simTime - this->last_pose_pub_time;
  if ((diff > std::chrono::steady_clock::duration::zero()) &&
    (diff < std::chrono::duration<double> {1.0 / this->update_frequency}))
  {
    return;
  }

  // Tracker pose in world frame
  const auto pose_WT = ignition::gazebo::worldPose(this->entity, _ecm);

  if (this->stream_rigid_bodies)
  {
    _ecm.Each<components::MotionCaptureRigidBody>(
      [&](const Entity& _entity,
      const components::MotionCaptureRigidBody* _rigid_body)->bool
      {
        // Rigid body pose in world frame
        const auto pose_WB = ignition::gazebo::worldPose(_entity, _ecm);

        // Rigid body pose in tracker frame (pose_TB = pose_TW * pose_WB)
        const auto pose_TB = pose_WT.Inverse() * pose_WB;

        if (this->trackers.count(_entity) == 0)
        {
          ignmsg << "Creating Tracker: " << _rigid_body->Data() << std::endl;
          this->trackers.insert(
            {
              _entity,
              std::make_unique<RigidBodyTracker>(
                _rigid_body->Data(),
                this->connection.get())
            });
        }
        this->trackers[_entity]->updatePose(_info.simTime, pose_TB);
        return true;
      });
  }

  for (auto&[_, tracker] : this->trackers)
  {
    tracker->mainloop();
  }
  this->connection->mainloop();
}

}  // namespace nexus_gazebo

IGNITION_ADD_PLUGIN(
  nexus_gazebo::MotionCaptureSystem,
  nexus_gazebo::System,
  nexus_gazebo::MotionCaptureSystem::ISystemConfigure,
  nexus_gazebo::MotionCaptureSystem::ISystemPostUpdate)
