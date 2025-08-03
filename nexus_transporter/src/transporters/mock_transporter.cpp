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

#include <rclcpp/rclcpp.hpp>

#include <nexus_transporter/Transporter.hpp>

#include <rmf_building_map_msgs/msg/building_map.hpp>

#include <mutex>
#include <thread>
#include <unordered_set>


namespace nexus_transporter {

using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;

//==============================================================================
struct Location
{
  double x;
  double y;
  std::optional<std::string> name;

  Location(
    double x_,
    double y_,
    std::optional<std::string> name_)
  : x(x_),
    y(y_),
    name(std::move(name_))
  {}
};

//==============================================================================
struct MockTransporter3000
{
  std::string name;
  Location current_location;
  std::unordered_map<std::string, Location> destinations_map;
  std::optional<Itinerary> itinerary; // If idle this should be nullopt.

  MockTransporter3000(
    const std::string& name_,
    Location current_location_,
    const std::unordered_map<std::string, Location>& destinations_map_,
    std::optional<Itinerary> itinerary_)
  : name(name_),
    current_location(current_location_),
    destinations_map(destinations_map_),
    itinerary(itinerary_)
  {}
};

//==============================================================================
std::string make_mock_transporter_name(const std::string& index_str)
{
  return "mock_transporter_" + index_str;
}

//==============================================================================
double get_2d_distance(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

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

    _nav_graph_names = n->declare_parameter(
      "nav_graph_names",
      std::vector<std::string>({"1"}));
    if (_nav_graph_names.empty())
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "nav_graph_names is required"
      );
      return false;
    }
    else
    {
      std::stringstream ss;
      for (const auto& i : _nav_graph_names)
      {
        ss << i << ", ";
      }
      RCLCPP_INFO(
        n->get_logger(),
        "MockTransporter nav_graph_names set to [%s]",
        ss.str().c_str());
    }

    // Set the speed of transporter (m/s)
    _speed = n->declare_parameter("speed", 1.0);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter speed set to [%.2f].",
      _speed
    );

    _node = node;

    const auto transient_qos =
      rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();

    _building_map_sub = n->create_subscription<BuildingMap>(
      "/map",
      transient_qos,
      [&](BuildingMap::SharedPtr msg)
      {
        _building_map = msg;

        std::unordered_map<std::string, std::unordered_map<std::string, Location>>
          nav_graph_to_destinations_map;
        for (const auto& n : _nav_graph_names)
        {
          nav_graph_to_destinations_map.insert({n, {}});
        }

        for (const auto& level : _building_map->levels)
        {
          for (const auto& graph : level.nav_graphs)
          {
            if (nav_graph_to_destinations_map.find(graph.name) ==
              nav_graph_to_destinations_map.end())
            {
              continue;
            }

            for (const auto& v : graph.vertices)
            {
              if (!v.name.empty())
              {
                nav_graph_to_destinations_map[graph.name].insert({
                  v.name,
                  Location(v.x, v.y, v.name)});
              }
            }
          }
        }

        std::unordered_map<std::string, std::shared_ptr<MockTransporter3000>>
          transporters;
        for (const auto& nd : nav_graph_to_destinations_map)
        {
          if (nd.second.empty())
          {
            continue;
          }

          const std::string transporter_name =
            make_mock_transporter_name(nd.first);
          transporters.insert({
            transporter_name,
            std::make_shared<MockTransporter3000>(
              transporter_name,
              nd.second.begin()->second,
              nd.second,
              std::nullopt)});
        }

        std::lock_guard<std::mutex> lock(_mutex);
        _transporters = transporters;
      });

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

    auto n = _node.lock();

    // This transporter can only go to one destination at a time.
    if (destinations.size() > 1)
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "MockTransporter currently only supports 1 destination at a time");
      completed_cb(std::nullopt);
      return;
    }

    const rclcpp::Time now = n ? n->get_clock()->now() : rclcpp::Clock().now();

    const auto& destination = destinations[0];
    std::lock_guard<std::mutex> lock(_mutex);
    for (const auto& t : _transporters)
    {
      const auto dest = t.second->destinations_map.find(destination.name);
      if (dest != t.second->destinations_map.end())
      {
        // Assign transporter
        const auto& transporter_location = t.second->current_location;

        double travel_duration;
        if (transporter_location.name.has_value() &&
          dest->second.name == transporter_location.name)
        {
          travel_duration = 0.0;
        }
        else
        {
          travel_duration = get_2d_distance(
            dest->second.x, dest->second.y, transporter_location.x,
            transporter_location.y) / _speed;
        }

        completed_cb(Itinerary{
            id,
            destinations,
            t.second->name,
            now + rclcpp::Duration::from_seconds(travel_duration),
            now + rclcpp::Duration::from_seconds(60.0)
          });
      }
    }

    completed_cb(std::nullopt);
    return;

  }

  void transport_to_destination(
    Itinerary itinerary,
    Transporter::TransportFeedback feedback_cb,
    Transporter::TransportCompleted completed_cb) final
  {
    const auto& current_transporter = itinerary.transporter_name();
    const auto& current_location =
      _transporters.at(current_transporter)->current_location;
    auto n = _node.lock();
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter %s starting at pose [%.2f, %.2f]",
      current_transporter.c_str(), current_location.x, current_location.y);
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

    // This transporter can only go to one destination at a time.
    if (destinations.size() > 1)
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "MockTransporter currently only supports 1 destination at a time");
      completed_cb(false);
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
    {
      _thread.join();
    }

    RCLCPP_INFO(n->get_logger(),
      "Received request for transporter [%s] to %s",
      current_transporter.c_str(), destination.name.c_str());

    // TODO(YV): Capture a data_ptr to avoid reference captures
    _thread = std::thread(
      [this, feedback_cb = feedback_cb, completed_cb = completed_cb](
        Itinerary itinerary)
      {
        const auto& current_transporter = itinerary.transporter_name();
        {
          std::lock_guard<std::mutex> lock(_mutex);
          _transporters.at(current_transporter)->itinerary = itinerary;
        }
        const auto& destination = itinerary.destinations()[0];
        const auto& destinations_map =
          _transporters.at(current_transporter)->destinations_map;
        const auto loc_it = destinations_map.find(destination.name);
        if (loc_it == destinations_map.end())
        {
          completed_cb(false);
        }
        
        const auto& current_loc =
          _transporters.at(current_transporter)->current_location;
        const auto& dest_loc = loc_it->second;
        const auto& dist = get_2d_distance(
          dest_loc.x, dest_loc.y, current_loc.x, current_loc.y);
        const auto now = rclcpp::Clock().now();
        // TODO(YV): Make duration a param
        const auto finish_time =
          now + rclcpp::Duration::from_seconds(dist/_speed);
        
        // Calculate direction vector components
        double dx = dest_loc.x - current_loc.x;
        double dy = dest_loc.y - current_loc.y;
        
        // Normalize the direction vector
        double direction_magnitude = std::sqrt(dx * dx + dy * dy);
        if (direction_magnitude > 0.0) {
          dx /= direction_magnitude;
          dy /= direction_magnitude;
        }
        
        Transporter::TransporterState state;
        state.transporter = current_transporter;
        state.model = "MockTransporter3000";
        state.task_id = itinerary.id();
        state.location.header.frame_id = "world";
        state.location.header.stamp = now;
        state.location.pose.position.x = current_loc.x;
        state.location.pose.position.y = current_loc.y;
        state.state = state.STATE_IDLE;
        feedback_cb(state);

        state.state = state.STATE_MOVING;
        _stop = false;
        double dist_traveled = 0.0;

        

        while (rclcpp::Clock().now() < finish_time && !_stop)
        {
          std::this_thread::sleep_for(std::chrono::seconds(1));

          double x_increment;
          double y_increment;
          
          // Calculate remaining distance to travel
          double remaining_dist = dist - dist_traveled;
          
          if (remaining_dist < _speed)
          {
            // Final step - travel the remaining distance
            x_increment = dx * remaining_dist;
            y_increment = dy * remaining_dist;
          }
          else
          {
            // Normal step - travel at speed
            x_increment = dx * _speed;
            y_increment = dy * _speed;
          }

          state.location.pose.position.x += x_increment;
          state.location.pose.position.y += y_increment;
          dist_traveled += std::sqrt(x_increment * x_increment + y_increment * y_increment);
          
          // Update transporter's current location
          _transporters.at(current_transporter)->current_location.x = state.location.pose.position.x;
          _transporters.at(current_transporter)->current_location.y = state.location.pose.position.y;
          feedback_cb(state);
        }

        // completed_cb(true);
        // std::lock_guard<std::mutex> lock(_mutex);

        // // Check if the transporter is at the unloading station
        // if (destination.name == _unloading_station)
        // {
        //   // Work order completed, ok to remove transporter from list
        //   _transporters.erase(
        //     current_transporter);
        // }
        // else
        // {
        //   // Update the transporter location
        //   _transporters.at(
        //     current_transporter)->current_location.name = std::nullopt;
        //   for (const auto& it : _destinations)
        //   {
        //     if (it.second ==
        //     _transporters.at(current_transporter)->current_location.pose)
        //     {
        //       _transporters.at(
        //         current_transporter)->current_location.name = it.first;
        //       break;
        //     }
        //   }
        //   _transporters.at(current_transporter)->itinerary = std::nullopt;
        // }
      }, itinerary);
  }

  bool cancel(Itinerary itinerary) final
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
  /// The nav graphs that represent mock transporters
  std::vector<std::string> _nav_graph_names;
  /// The speed of the transporter in m/s
  double _speed;
  /// Increment along X axis for transporter destinations
  double _increment;
  /// RMF building map
  BuildingMap::SharedPtr _building_map = nullptr;
  /// All the instances of MockTransporter, based on the navigation graph names
  std::unordered_map<std::string,
    std::shared_ptr<MockTransporter3000>> _transporters;

  /// Building map subscription to retrieve navigation graphs
  rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
    _building_map_sub = nullptr;

  std::thread _thread;
  std::mutex _mutex;
  std::atomic_bool _stop = false;
};
} // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::MockTransporter, nexus_transporter::Transporter)
