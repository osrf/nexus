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

    _travel_duration_seconds =
      n->declare_parameter("travel_duration_seconds", 2);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter travel_duration_seconds set to [%d].",
      _travel_duration_seconds);

    _node = node;

    const auto transient_qos =
      rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();

    _building_map_sub = n->create_subscription<BuildingMap>(
      "/map",
      transient_qos,
      [&](BuildingMap::SharedPtr msg)
      {
        auto n = _node.lock();
        if (!n)
        {
          return;
        }
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

          std::stringstream ss;
          for (const auto& it : nd.second)
          {
            ss << it.first << ", ";
          }
          RCLCPP_INFO(
            n->get_logger(),
            "Adding a new MockTransporter3000 with destinations [%s]",
            ss.str().c_str());

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
    auto n = this->_node.lock();
    if (!n)
    {
      std::cerr << "MockTransporter::get_itinerary - invalid node" << std::endl;
      completed_cb(std::nullopt);
      return;
    }

    if (destinations.empty())
    {
      RCLCPP_ERROR(n->get_logger(), "Itinerary has no destinations");
      completed_cb(std::nullopt);
      return;
    }

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
    std::unique_lock<std::mutex> lock(_mutex);
    for (const auto& t : _transporters)
    {
      const auto dest = t.second->destinations_map.find(destination.name);
      if (dest != t.second->destinations_map.end())
      {
        // Assign transporter
        const auto& transporter_location = t.second->current_location;

        int travel_duration;
        if (transporter_location.name.has_value() &&
          dest->second.name == transporter_location.name)
        {
          travel_duration = 0;
        }
        else
        {
          travel_duration = _travel_duration_seconds;
        }

        lock.unlock();
        completed_cb(Itinerary{
            id,
            destinations,
            t.second->name,
            now + rclcpp::Duration::from_seconds(travel_duration),
            now + rclcpp::Duration::from_seconds(60.0)
          });
        return;
      }
    }

    completed_cb(std::nullopt);
  }

  void transport_to_destination(
    Itinerary itinerary,
    Transporter::TransportFeedback feedback_cb,
    Transporter::TransportCompleted completed_cb) final
  {
    auto n = this->_node.lock();
    if (!n)
    {
      std::cerr
        << "MockTransporter::transport_to_destination - invalid node"
        << std::endl;
      completed_cb(false);
      return;
    }

    const auto& destinations = itinerary.destinations();
    if (destinations.empty())
    {
      RCLCPP_ERROR(n->get_logger(), "Itinerary has no destinations");
      completed_cb(false);
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

    const auto& desired_transporter_name = itinerary.transporter_name();

    {
      std::unique_lock<std::mutex> lock(_mutex);

      // The transporter needs to exist
      const auto& transporter_it = _transporters.find(desired_transporter_name);
      if (transporter_it == _transporters.end())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "MockTransporter::transport_to_destination: no such transporter [%s]",
          desired_transporter_name.c_str());
        lock.unlock();
        completed_cb(false);
        return;
      }

      // The transporter needs to be idle
      if (transporter_it->second->itinerary.has_value())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "MockTransporter::transport_to_destination: transporter [%s] is "
          "already performing itinerary [%s]",
          desired_transporter_name.c_str(),
          transporter_it->second->itinerary->id().c_str());
        lock.unlock();
        completed_cb(false);
        return;
      }

      // The transporter needs to have this destination name available
      const auto& destination = destinations[0];
      const auto& d_it =
        transporter_it->second->destinations_map.find(destination.name);
      if (d_it == transporter_it->second->destinations_map.end())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "MockTransporter %s does not support destination [%s]",
          desired_transporter_name.c_str(),
          destination.name.c_str());
        lock.unlock();
        completed_cb(false);
        return;
      }

      const auto& current_location = transporter_it->second->current_location;
      RCLCPP_INFO(
        n->get_logger(),
        "MockTransporter %s starting at pose [%.2f, %.2f]",
        desired_transporter_name.c_str(),
        current_location.x,
        current_location.y);
    }

    if (_thread.joinable())
    {
      _thread.join();
    }

    RCLCPP_INFO(n->get_logger(),
      "Received request for transporter [%s] to %s",
      itinerary.transporter_name().c_str(),
      destinations[0].name.c_str());

    // TODO(YV): Capture a data_ptr to avoid reference captures
    _thread = std::thread(
      [this, feedback_cb = feedback_cb, completed_cb = completed_cb](
        Itinerary itinerary)
      {
        const auto& selected_transporter = itinerary.transporter_name();
        const auto& destination = itinerary.destinations()[0];

        double dx, dy;

        Transporter::TransporterState state;
        state.transporter = itinerary.transporter_name();
        state.model = "MockTransporter3000";
        state.task_id = itinerary.id();
        state.location.header.frame_id = "world";

        {
          std::lock_guard<std::mutex> lock(_mutex);
          const auto& transporter = _transporters.at(selected_transporter);
          transporter->itinerary = itinerary;

          const auto dest_it =
            transporter->destinations_map.find(destination.name);
          if (dest_it == transporter->destinations_map.end())
          {
            completed_cb(false);
          }

          // distance travel per second
          dx =
            (dest_it->second.x - transporter->current_location.x)
            / _travel_duration_seconds;
          dy =
            (dest_it->second.y - transporter->current_location.y)
            / _travel_duration_seconds;

          state.location.header.stamp = rclcpp::Clock().now();
          state.location.pose.position.x = transporter->current_location.x;
          state.location.pose.position.y = transporter->current_location.y;
          state.state = state.STATE_IDLE;
          feedback_cb(state);
        }

        state.state = state.STATE_MOVING;
        _stop = false;

        for (int i = 0; i < _travel_duration_seconds; ++i)
        {
          // TODO(ac): use a stop flag for each mock transporter
          if (_stop)
          {
            break;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));

          state.location.pose.position.x += dx;
          state.location.pose.position.y += dy;

          // Update transporter's current location
          std::lock_guard<std::mutex> lock(_mutex);
          _transporters.at(selected_transporter)->current_location.x =
            state.location.pose.position.x;
          _transporters.at(selected_transporter)->current_location.y =
            state.location.pose.position.y;
          feedback_cb(state);
        }

        completed_cb(true);

        std::lock_guard<std::mutex> lock(_mutex);
        _transporters.at(selected_transporter)->current_location.name =
          destination.name;
        _transporters.at(selected_transporter)->itinerary = std::nullopt;
      },
      itinerary);
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
  /// The mocked travel duration of each transport
  int _travel_duration_seconds;
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
