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

#include "calibration_node.hpp"

namespace nexus {

//==============================================================================
struct CalibrationNode::Tracker
{
  std::string name;
  std::string vrpn_frame_id;
  std::unique_ptr<vrpn_Tracker_Remote> vrpn_tracker;
  std::shared_ptr<tf2::BufferCore> buffer;

  Tracker(
    const std::string& name_,
    const std::string& vrpn_frame_id_,
    std::shared_ptr<vrpn_Connection> connection,
    std::shared_ptr<tf2::BufferCore> buffer_)
  : name(name_),
    vrpn_frame_id(vrpn_frame_id_),
    buffer(buffer_)
  {
    vrpn_tracker =
      std::make_unique<vrpn_Tracker_Remote>(name.c_str(), connection.get());
    vrpn_tracker->register_change_handler(this, &Tracker::handle_pose);
  }

  // This callback will write pose information as a static transform into
  // the buffer.
  static void VRPN_CALLBACK handle_pose(
    void* userData,
    const vrpn_TRACKERCB tracker_pose)
  {
    Tracker* tracker = static_cast<Tracker*>(userData);

    TransformStamped transform;
    transform.header.frame_id = tracker->vrpn_frame_id;
    transform.header.stamp.sec = tracker_pose.msg_time.tv_sec;
    transform.header.stamp.nanosec = tracker_pose.msg_time.tv_usec * 1000;
    transform.child_frame_id = tracker->name;
    auto& translation = transform.transform.translation;
    translation.x = tracker_pose.pos[0];
    translation.y = tracker_pose.pos[1];
    translation.z = tracker_pose.pos[2];
    auto& quat = transform.transform.rotation;
    quat.x = tracker_pose.quat[0];
    quat.y = tracker_pose.quat[1];
    quat.z = tracker_pose.quat[2];
    quat.w = tracker_pose.quat[3];

    tracker->buffer->setTransform(
      std::move(transform),
      tracker->name,
      true // static
    );
  }

  ~Tracker()
  {
    vrpn_tracker->unregister_change_handler(this, &Tracker::handle_pose);
  }
};

//==============================================================================
CalibrationNode::CalibrationNode(const rclcpp::NodeOptions& options)
: LifecycleNode("nexus_calibration_node", options),
  _vrpn_connection(nullptr)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Running nexus_calibration_node..."
  );

  _vrpn_frame_id = this->declare_parameter("vrpn_frame_id", "world");
  _tracker_names = this->declare_parameter("tracker_names", _tracker_names);
  {
    ParameterDescriptor desc;
    desc.description = "The vrpn server address as ip_address:port.";
    _server_address =
      this->declare_parameter<std::string>("vrpn_server_address", desc);
  }
  _update_rate = this->declare_parameter("update_rate", 2.0);
  _workcell_id = this->declare_parameter<std::string>("workcell_id");
}

//==============================================================================
auto CalibrationNode::on_configure(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  try
  {
    // The vrpn connection will attempt to connect when its mainloop is run.
    _vrpn_connection = std::shared_ptr<vrpn_Connection>(
      vrpn_get_connection_by_name(_server_address.c_str()));

    _tf_buffer = std::make_shared<tf2::BufferCore>();

    // Create trackers
    for (const auto& t : _tracker_names)
    {
      auto it = _trackers.insert({t, nullptr});
      if (!it.second)
      {
        // Skip duplicates.
        RCLCPP_WARN(
          this->get_logger(),
          "Duplicate tracker [%s] provided. Ignoring...",
          t.c_str()
        );
        continue;
      }
      it.first->second = std::make_shared<Tracker>(
        t,
        this->_vrpn_frame_id,
        this->_vrpn_connection,
        this->_tf_buffer
      );
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error while setting up vrpn connection: %s",
      e.what()
    );
    return CallbackReturn::FAILURE;
  }

  if (_workcell_id.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter workcell_id cannot be empty."
    );
    return CallbackReturn::FAILURE;
  }

  if (_vrpn_frame_id.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter vrpn_frame_id cannot be empty."
    );
    return CallbackReturn::FAILURE;
  }

  _extrinsic_service = this->create_service<ExtrinsicCalibrationServiceType>(
    ExtrinsicCalibrationService::service_name(_workcell_id),
    [this](
      ExtrinsicCalibrationServiceType::Request::ConstSharedPtr req,
      ExtrinsicCalibrationServiceType::Response::SharedPtr res)
    {
      res->success = false;
      if (this->get_current_state().label() != "active")
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "ExtrinsicCalibrationService called before node activation!"
        );
        return;
      }
      try
      {
        if (req->frame_id.empty())
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "frame_id cannot be empty in ExtrinsicCalibration request."
          );
          return;
        }
        for (const auto& [tracker_name, _] : _trackers)
        {
          res->results.emplace_back(_tf_buffer->lookupTransform(
            req->frame_id,
            tracker_name,
            tf2::TimePointZero
          ));
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Error when looking up transform. Details: %s",
          e.what()
        );
        return;
      }

      res->success = true;
    }
  );

  // Start update timer.
  const auto timer_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(1.0 / _update_rate));
  this->_timer = this->create_wall_timer(
    timer_period,
    [this]()
    {
      if (this->get_current_state().label() != "active")
      {
        return;
      }

      this->_vrpn_connection->mainloop();
      RCLCPP_DEBUG(
        this->get_logger(),
        "Updated connection mainLoop."
      );
      if (!this->_vrpn_connection->doing_okay())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "VRPN connection to server %s is not doing okay...",
          _server_address.c_str()
        );
        return;
      }
      for (const auto& [t, tracker] : _trackers)
      {
        tracker->vrpn_tracker->mainloop();
        RCLCPP_DEBUG(
          this->get_logger(),
          "Updated mainloop for tracker %s.",
          t.c_str()
        );
      }
    });

  RCLCPP_INFO(this->get_logger(), "Successfully configured.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto CalibrationNode::on_cleanup(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  _trackers.clear();
  _tf_buffer.reset();
  _vrpn_connection.reset();
  _timer.reset();
  _extrinsic_service.reset();
  RCLCPP_INFO(this->get_logger(), "Successfully cleaned up.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto CalibrationNode::on_shutdown(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto CalibrationNode::on_activate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Activating...");
  RCLCPP_INFO(this->get_logger(), "Successfully activated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto CalibrationNode::on_deactivate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  RCLCPP_INFO(this->get_logger(), "Successfully deactivated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto CalibrationNode::on_error(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_ERROR(this->get_logger(), "Error!");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
CalibrationNode::~CalibrationNode()
{
  // Delete all references within Tracker.
  _trackers.clear();
}

} // namespace nexus

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus::CalibrationNode)
