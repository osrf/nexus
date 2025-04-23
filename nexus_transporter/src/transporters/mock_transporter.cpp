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

#include <nexus_transporter/Transporter.hpp>

#include <mutex>
#include <thread>
#include <unordered_set>

namespace nexus_transporter {

//==============================================================================
struct Location
{
  double pose;
  std::optional<std::string> name;

  Location(
    double pose_,
    std::optional<std::string> name_)
  : pose(std::move(pose_)),
    name(std::move(name_))
  {}
};

//==============================================================================
struct MockTransporter3000
{
  std::string name;
  Location current_location;
  std::optional<Itinerary> itinerary; // If idle and not at `unloading_station` this should be nullopt.

  MockTransporter3000(
    const std::string& name_,
    Location current_location_,
    std::optional<Itinerary> itinerary_)
  : name(name_),
    current_location(current_location_),
    itinerary(itinerary_)
  {}
};

//==============================================================================
// MockTransporter that can only process once request at a time.
class MockTransporter : public Transporter
{
public:
  bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) final
  {
    auto n = node.lock();
    if (!n)
    {
      return false;
    }
    RCLCPP_INFO(
      n->get_logger(),
      "Configuring MockTransporter...");

    // Set the x-increment (m)
    _increment = n->declare_parameter("x_increment", 2.0);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter x_increment set to [%.2f].",
      _increment
    );

    std::vector<std::string> destinations;
    destinations = n->declare_parameter(
      "destinations",
      std::vector<std::string>(
        {"loading", "workcell_1", "workcell_2", "unloading"}));

    if (!destinations.empty())
    {
      double x_pose = 0.0;
      for (const auto& dest : destinations)
      {
        _destinations.insert({dest, x_pose});
        x_pose += _increment;
      }
    }

    std::stringstream ss;
    for (const auto& d : _destinations)
      ss << d.first << ", ";
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter configured with the following destinations: %s",
      ss.str().c_str()
    );

    // Set the initial location of all added transporters as first destination in the vector
    _initial_location = destinations.front();

    // Set the unloading station
    _unloading_station = n->declare_parameter("unloading_station", "unloading");
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter unloading station set to [%s].",
      _unloading_station.c_str()
    );

    // Set the speed of transporter (m/s)
    _speed = n->declare_parameter("speed", 1.0);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter speed set to [%.2f].",
      _speed
    );

    _node = node;
    _ready = true;
    RCLCPP_INFO(
      n->get_logger(),
      "Finished configuring MockTransporter!"
    );
    return true;
  }

  bool ready() const final
  {
    return _ready;
  }

  void get_itinerary(
    const std::string& id,
    const std::vector<Destination>& destinations,
    Transporter::ItineraryQueryCompleted completed_cb)
  {
    if (destinations.empty())
    {
      completed_cb(std::nullopt);
      return;
    }
    // This transporter can only go to one destination at a time.
    const auto& destination = destinations[0];
    if (_destinations.find(destination.name) == _destinations.end())
    {
      completed_cb(std::nullopt);
      return;
    }

    auto n = _node.lock();
    const rclcpp::Time now = n ? n->get_clock()->now() : rclcpp::Clock().now();

    // Assign transporter
    const std::string transporter_name = "mock_transporter_" + id;
    auto it = _transporters.insert({transporter_name, nullptr});
    if (it.second)
    {
      // New insertion
      it.first->second = std::make_shared<MockTransporter3000>(
        transporter_name,
        Location(0.0, _initial_location),
        std::nullopt);
    }

    const auto& transporter_location = it.first->second->current_location;
    const auto& dest_pose = _destinations.find(destination.name)->second;

    double travel_duration;
    if (transporter_location.name.has_value() &&
      destination.name == transporter_location.name)
    {
      travel_duration = 0.0;
    }
    else
    {
      travel_duration = abs(dest_pose - transporter_location.pose)/_speed;
    }

    completed_cb(Itinerary{
        id,
        destinations,
        transporter_name,
        now + rclcpp::Duration::from_seconds(travel_duration),
        now + rclcpp::Duration::from_seconds(60.0)
      });
  }

  void transport_to_destination(
    const Itinerary& itinerary,
    Transporter::TransportFeedback feedback_cb,
    Transporter::TransportCompleted completed_cb) final
  {
    const auto& current_transporter = itinerary.transporter_name();
    const auto& current_location =
      _transporters.at(current_transporter)->current_location;
    auto n = _node.lock();
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter %s starting at pose %.2f meters",
      current_transporter.c_str(), current_location.pose);
    {
      std::lock_guard<std::mutex> lock(_mutex);

      if (!_transporters.empty() &&
        (_transporters.find(current_transporter) != _transporters.end()) &&
        (_transporters.at(current_transporter)->itinerary.has_value()))
      {
        completed_cb(false);
        return;
      }
    }

    const auto& destinations = itinerary.destinations();
    if (destinations.empty())
    {
      completed_cb(true);
      return;
    }
    const auto& destination = destinations[0];
    if (current_location.name.has_value() &&
      destination.name == current_location.name)
    {
      completed_cb(true);
      return;
    }

    if (_thread.joinable())
      _thread.join();

    RCLCPP_INFO(n->get_logger(),
      "Received request for transporter [%s] to %s",
      current_transporter.c_str(), destination.name.c_str());

    // TODO(YV): Capture a data_ptr to avoid reference captures
    _thread = std::thread(
      [this, feedback_cb = feedback_cb, completed_cb = completed_cb](
        const Itinerary& itinerary)
      {
        const auto& current_transporter = itinerary.transporter_name();
        {
          std::lock_guard<std::mutex> lock(_mutex);

          _transporters.at(current_transporter)->itinerary = itinerary;
        }
        const auto& destination = itinerary.destinations()[0];
        const auto& dest_pose =
        _destinations.find(destination.name)->second;
        const auto& current_pose = _transporters.at(
          current_transporter)->current_location.pose;
        const auto& dist = abs(dest_pose - current_pose);
        const auto now = rclcpp::Clock().now();
        // TODO(YV): Make duration a param
        const auto finish_time =
        now + rclcpp::Duration::from_seconds(dist/_speed);
        Transporter::TransporterState state;
        state.transporter = current_transporter;
        state.model = "MockTransporter3000";
        state.task_id = itinerary.id();
        state.location.header.frame_id = "world";
        state.location.header.stamp = now;
        state.location.pose.position.x = current_pose;
        state.state = state.STATE_IDLE;
        feedback_cb(state);

        state.state = state.STATE_MOVING;
        _stop = false;
        double dist_traveled = 0.0;
        while (rclcpp::Clock().now() < finish_time && !_stop)
        {
          std::this_thread::sleep_for(std::chrono::seconds(1));

          double x_increment;
          if (dist - dist_traveled < _speed &&
          (finish_time - rclcpp::Clock().now()).seconds() <= 0.0)
          {
            auto gap = dist - dist_traveled;
            x_increment = dest_pose < current_pose ?
            -1*gap : gap;
          }
          else
            x_increment = dest_pose < current_pose ?
            -1*_speed : _speed;

          state.location.pose.position.x += x_increment;
          dist_traveled += abs(x_increment);
          _transporters.at(
            current_transporter)->current_location.pose =
          state.location.pose.position.x;
          feedback_cb(state);
        }

        completed_cb(true);
        std::lock_guard<std::mutex> lock(_mutex);

        // Check if the transporter is at the unloading station
        if (destination.name == _unloading_station)
        {
          // Work order completed, ok to remove transporter from list
          _transporters.erase(
            current_transporter);
        }
        else
        {
          // Update the transporter location
          _transporters.at(
            current_transporter)->current_location.name = std::nullopt;
          for (const auto& it : _destinations)
          {
            if (it.second ==
            _transporters.at(current_transporter)->current_location.pose)
            {
              _transporters.at(
                current_transporter)->current_location.name = it.first;
              break;
            }
          }
          _transporters.at(current_transporter)->itinerary = std::nullopt;
        }
      }, itinerary);
  }

  bool cancel(const Itinerary& itinerary) final
  {
    std::unique_lock lock(_mutex);
    const auto& transporter = itinerary.transporter_name();

    if (_transporters.find(transporter) == _transporters.end())
    {
      return true;
    }

    if (_transporters.at(transporter)->itinerary.has_value())
    {
      _stop = true;
      // need to unlock first to avoid deadlock
      lock.unlock();
      if (_thread.joinable())
        _thread.join();
      return true;
    }

    return false;
  }

  ~MockTransporter()
  {
    _stop = true;
    if (_thread.joinable())
      _thread.join();
  }

private:
  /// A weak_ptr to a lifecycle node
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;
  /// A boolean that is set true when the system is initialized and without
  /// errors
  bool _ready = false;
  /// The MockTransporter system offers multiple transporters capable of transporting items
  std::unordered_map<std::string,
    std::shared_ptr<MockTransporter3000>> _transporters;
  /// First stop in the list of destinations where the transporter starts
  std::string _initial_location;
  /// A set of destinations that _transporter is capable to visiting
  std::unordered_map<std::string, double> _destinations;
  /// The final location of each transporter request
  std::string _unloading_station;
  /// The speed of the transporter in m/s
  double _speed;
  /// Increment along X axis for transporter destinations
  double _increment;

  std::thread _thread;
  std::mutex _mutex;
  std::atomic_bool _stop = false;
};
} // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::MockTransporter, nexus_transporter::Transporter)
