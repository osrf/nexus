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
#include <yaml-cpp/yaml.h>

#include <nexus_transporter/Transporter.hpp>

#include <filesystem>
#include <mutex>
#include <thread>


namespace nexus_transporter {

//==============================================================================
struct Location
{
  double x;
  double y;
  std::string name;

  Location(
    double x_,
    double y_,
    std::string name_)
  : x(x_),
    y(y_),
    name(name_)
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
std::string make_mock_transporter_name(
  const std::string& graph_name,
  const std::string& level_name)
{
  std::stringstream ss;
  ss << "mock_transporter_"  << graph_name << "_" << level_name;
  return ss.str();
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

    _nav_graph_files = n->declare_parameter(
      "nav_graph_files",
      std::vector<std::string>({}));
    if (_nav_graph_files.empty())
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "nav_graph_files is required"
      );
      return false;
    }

    std::unordered_map<std::string, std::unordered_map<std::string, Location>>
      mock_transporter_name_to_destinations_map;

    std::stringstream ss;
    for (const auto& f : _nav_graph_files)
    {
      const YAML::Node graph_config = YAML::LoadFile(f);
      if (!graph_config)
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "failed to load graph from [%s], skipping...",
          f.c_str());
        continue;
      }
      const YAML::Node levels = graph_config["levels"];
      if (!levels || !levels.IsMap())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "graph [%s] has invalid levels, skipping...",
          f.c_str());
        continue;
      }
      for (const auto& level : levels)
      {
        const std::string mock_transporter_name =
          make_mock_transporter_name(
            std::filesystem::path(f).stem().string(),
            level.first.as<std::string>());
        mock_transporter_name_to_destinations_map.insert(
          {mock_transporter_name, {}});

        const YAML::Node vertices = level.second["vertices"];
        if (!vertices || !vertices.IsSequence())
        {
          continue;
        }

        for (const auto& vertex : vertices)
        {
          if (!vertex.IsSequence() || vertex.size() < 3 || !vertex[2].IsMap() ||
            !vertex[2]["name"])
          {
            continue;
          }
          const std::string wp_name = vertex[2]["name"].as<std::string>();
          if (wp_name.empty())
          {
            continue;
          }
          mock_transporter_name_to_destinations_map[mock_transporter_name]
          .insert({
            wp_name,
            Location(
              vertex[0].as<double>(), vertex[1].as<double>(), wp_name)
          });
        }
      }

      ss << f << ", ";
    }
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter nav_graph_files set to [%s]",
      ss.str().c_str());

    {
      std::lock_guard<std::mutex> lock(_mutex);
      for (const auto& it : mock_transporter_name_to_destinations_map)
      {
        if (it.second.empty())
        {
          continue;
        }

        std::stringstream dest_ss;
        for (const auto& dest_it : it.second)
        {
          dest_ss << dest_it.first << ", ";
        }
        RCLCPP_INFO(
          n->get_logger(),
          "Adding a new MockTransporter3000 [%s] with destinations [%s]",
          it.first.c_str(), dest_ss.str().c_str());

        _transporters.insert({
          it.first,
          MockTransporter3000(
            it.first,
            it.second.begin()->second,
            it.second,
            std::nullopt)});
      }
    }

    _travel_duration_seconds_per_destination =
      n->declare_parameter("travel_duration_seconds_per_destination", 2);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter travel_duration_seconds set to [%d].",
      _travel_duration_seconds_per_destination);

    _world_frame =
      n->declare_parameter("world_frame", "world");
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter world_frame set to [%s].",
      _world_frame.c_str());

    _conveyor_mode =
      n->declare_parameter("conveyor_mode", false);
    RCLCPP_INFO(
      n->get_logger(),
      "MockTransporter conveyor_mode set to [%s].",
      _conveyor_mode ? "true" : "false");

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

    const rclcpp::Time now = n ? n->get_clock()->now() : rclcpp::Clock().now();

    std::unique_lock<std::mutex> lock(_mutex);
    for (const auto& t : _transporters)
    {
      bool found_transporter = true;

      for (const auto& d : destinations)
      {
        if (t.second.destinations_map.find(d.name) ==
          t.second.destinations_map.end())
        {
          found_transporter = false;
          break;
        }
      }

      if (!found_transporter)
      {
        continue;
      }

      lock.unlock();

      const int destination_multiplier =
        _conveyor_mode ? destinations.size() - 1 : destinations.size();
      completed_cb(Itinerary{
        id,
        destinations,
        t.second.name,
        now + rclcpp::Duration::from_seconds(
          _travel_duration_seconds_per_destination * destination_multiplier),
        now + rclcpp::Duration::from_seconds(60.0)});
      return;
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
      if (transporter_it->second.itinerary.has_value())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "MockTransporter::transport_to_destination: transporter [%s] is "
          "already performing itinerary [%s]",
          desired_transporter_name.c_str(),
          transporter_it->second.itinerary->id().c_str());
        lock.unlock();
        completed_cb(false);
        return;
      }

      // The transporter needs to have the destinations available
      for (const auto& d : destinations)
      {
        if (transporter_it->second.destinations_map.find(d.name) ==
          transporter_it->second.destinations_map.end())
        {
          RCLCPP_ERROR(
            n->get_logger(),
            "MockTransporter %s does not support destination [%s]",
            desired_transporter_name.c_str(),
            d.name.c_str());
          lock.unlock();
          completed_cb(false);
          return;
        }
      }

      const auto& current_location = transporter_it->second.current_location;
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

    std::stringstream ss;
    for (const auto& d : destinations)
    {
      ss << d.name << ", ";
    }
    RCLCPP_INFO(n->get_logger(),
      "Received request for transporter [%s] to [%s]",
      itinerary.transporter_name().c_str(),
      ss.str().c_str());

    // TODO(YV): Capture a data_ptr to avoid reference captures
    _thread = std::thread(
      [this, feedback_cb = feedback_cb, completed_cb = completed_cb](
        Itinerary itinerary)
      {
        const auto& selected_transporter = itinerary.transporter_name();
        double curr_x, curr_y;

        Transporter::TransporterState state;
        state.transporter = itinerary.transporter_name();
        state.model = "MockTransporter3000";
        state.task_id = itinerary.id();
        state.location.header.frame_id = _world_frame;

        {
          std::lock_guard<std::mutex> lock(_mutex);
          auto& transporter = _transporters.at(selected_transporter);
          transporter.itinerary = itinerary;

          if (_conveyor_mode)
          {
            const auto& dest =
              transporter.destinations_map.find(
                itinerary.destinations().at(0).name);
            transporter.current_location.x = dest->second.x;
            transporter.current_location.y = dest->second.y;
            transporter.current_location.name =
              itinerary.destinations().at(0).name;
          }

          curr_x = transporter.current_location.x;
          curr_y = transporter.current_location.y;

          state.location.header.stamp = rclcpp::Clock().now();
          state.location.pose.position.x = transporter.current_location.x;
          state.location.pose.position.y = transporter.current_location.y;
          state.state = state.STATE_IDLE;
          feedback_cb(state);
        }

        state.state = state.STATE_MOVING;
        _stop = false;

        std::size_t destination_index = _conveyor_mode ? 1 : 0;
        for (; destination_index < itinerary.destinations().size();
          ++destination_index)
        {
          const auto& d = itinerary.destinations().at(destination_index);
          double dx, dy;
          {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto& dest =
              _transporters.at(selected_transporter).destinations_map.find(
                d.name);
            dx = (dest->second.x - curr_x)
              / _travel_duration_seconds_per_destination;
            dy = (dest->second.y - curr_y)
              / _travel_duration_seconds_per_destination;
          }

          for (int i = 0; i < _travel_duration_seconds_per_destination; ++i)
          {
            // TODO(ac): use a stop flag for each mock transporter
            if (_stop)
            {
              break;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            curr_x += dx;
            curr_y += dy;
            state.location.pose.position.x = curr_x;
            state.location.pose.position.y = curr_y;
            feedback_cb(state);
          }

          std::lock_guard<std::mutex> lock(_mutex);
          _transporters.at(selected_transporter).current_location.name =
            d.name;
        }

        completed_cb(true);

        std::lock_guard<std::mutex> lock(_mutex);
        _transporters.at(selected_transporter).itinerary = std::nullopt;
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

    if (_transporters.at(transporter).itinerary.has_value())
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
  std::vector<std::string> _nav_graph_files;
  /// The mocked travel duration to each destination
  int _travel_duration_seconds_per_destination;
  /// World frame for location in feedback
  std::string _world_frame;
  /// Whether or not the mock transporter is operating like a conveyor, which
  /// allows the transporter's location to start immediately at the first
  /// destination
  bool _conveyor_mode;
  /// All the instances of MockTransporter, based on the navigation graph names
  std::unordered_map<std::string, MockTransporter3000> _transporters;

  std::thread _thread;
  std::mutex _mutex;
  std::atomic_bool _stop = false;
};

} // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::MockTransporter, nexus_transporter::Transporter)
