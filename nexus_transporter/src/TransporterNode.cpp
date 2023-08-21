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


#include "TransporterNode.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

//==============================================================================
namespace nexus_transporter {

//==============================================================================
auto TransporterNode::on_configure(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Configuring...");

  _data->w_node = this->weak_from_this();

  _data->tf_broadcaster =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Setup service server to process IsTransportAvailable requests
  _data->availability_srv =
    this->create_service<IsTransporterAvailable>(
    IsTransporterAvailableService::service_name(this->get_name()),
    [data = _data](
      IsTransporterAvailable::Request::ConstSharedPtr request,
      IsTransporterAvailable::Response::SharedPtr response)
    {
      response->available = false;
      auto node = data->w_node.lock();
      if (node == nullptr)
      {
        return;
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Received IsTransporterAvailable request from %s with id %s. "
        "Destination: %s. Payload: [%s]",
        request->request.requester.c_str(),
        request->request.id.c_str(),
        request->request.destination.c_str(),
        request->request.payload.c_str()
      );

      if (request->request.destination.empty())
      {
        RCLCPP_WARN(
          node->get_logger(),
          "Ignoring request as destination is empty."
        );
        return;
      }

      if (node->get_current_state().label() != "active")
      {
        RCLCPP_WARN(
          node->get_logger(),
          "Node is not active. Ignoring IsTransporterAvailable request."
        );
        return;
      }

      if (data->transporter == nullptr || !data->transporter->ready())
      {
        RCLCPP_WARN(
          node->get_logger(),
          "The transporter [%s] is not yet ready. Please try again later.",
          data->transporter_plugin_name.c_str()
        );
        return;
      }

      auto itinerary = data->transporter->get_itinerary(
        request->request.id,
        request->request.destination);

      if (!itinerary.has_value())
      {
        RCLCPP_WARN(
          node->get_logger(),
          "The transporter is not configured to go to destination [%s]",
          request->request.destination.c_str()
        );

        return;
      }

      // Success
      response->available = true;
      response->transporter = itinerary->transporter_name();
      response->estimated_finish_time = itinerary->estimated_finish_time();
    },
    rmw_qos_profile_services_default,
    _data->cb_group
    );

  // Load transporter plugin
  if (_data->transporter_plugin_name.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Required parameter transporter_plugin was not provided"
    );

    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loading transporter_plugin: %s",
    _data->transporter_plugin_name.c_str()
  );

  try
  {
    _data->transporter = _data->transporter_loader.createUniqueInstance(
      _data->transporter_plugin_name);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load transporter_plugin [%s]. Detailed error: %s",
      _data->transporter_plugin_name.c_str(),
      e.what()
    );

    return CallbackReturn::FAILURE;
  }

  if (!_data->transporter->configure(shared_from_this()))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to configure transporter_plugin [%s]",
      _data->transporter_plugin_name.c_str()
    );
    return CallbackReturn::FAILURE;
  }

  //Timer for registering with the system orchestrator
  this->_register_timer = this->create_wall_timer(std::chrono::seconds{1},
      [this]()
      {
        if (this->registration_callback())
        {
          this->_register_timer.reset();
        }
      });

  // Setup Transport action server
  _data->action_srv =
    rclcpp_action::create_server<ActionType>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    TransportAction::action_name(this->get_name()),
    [data = _data](
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActionType::Goal> goal)
    -> rclcpp_action::GoalResponse
    {
      // goal_handle
      auto node = data->w_node.lock();
      if (node)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Received transport goal request from [%s] with id [%s] for destination "
          "%s and payload [%s]",
          goal->request.requester.c_str(),
          goal->request.id.c_str(),
          goal->request.destination.c_str(),
          goal->request.payload.c_str()
        );
      }

      if (data->transporter == nullptr)
      {
        if (node)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Unable to accept goal as this TransporterNode is not configured "
            "with a valid transporter plugin. Rejecting goal..."
          );
        }
        return rclcpp_action::GoalResponse::REJECT;
      }
      // TODO(YV): Book keeping
      auto itinerary =
      data->transporter->get_itinerary(
        goal->request.id,
        goal->request.destination);
      if (!itinerary.has_value())
      {
        if (node)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Unable to generate an itinerary for destination [%s]. "
            "Rejecting goal...",
            goal->request.destination.c_str()
          );
        }
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (node)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Successfully generated itinerary with id: %s which is assigned to "
          "transporter: %s",
          itinerary->id().c_str(),
          itinerary->transporter_name().c_str()
        );
      }
      data->itineraries[uuid] =
      std::make_unique<Itinerary>(std::move(itinerary.value()));
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [data = _data](const std::shared_ptr<GoalHandle> goal_handle)
    -> rclcpp_action::CancelResponse
    {
      //  handle_cancel
      auto node = data->w_node.lock();
      if (node)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Received request to cancel goal"
        );
      }
      const auto it = data->itineraries.find(goal_handle->get_goal_id());
      if (it == data->itineraries.end())
      {
        if (node)
        {
          RCLCPP_WARN(
            node->get_logger(),
            "Cancellation goal uuid not found. No goal to cancel."
          );
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      const bool ok = data->transporter->cancel(*(it->second));
      if (ok)
      {
        if (node)
        {
          RCLCPP_INFO(
            node->get_logger(),
            "Successfully cancelled transport with id [%s] to destination [%s]",
            it->second->id().c_str(),
            it->second->destination().c_str()
          );
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      else
      {
        if (node)
        {
          RCLCPP_INFO(
            node->get_logger(),
            "Unable to cancel transport with id [%s] to destination [%s]",
            it->second->id().c_str(),
            it->second->destination().c_str()
          );
        }
        return rclcpp_action::CancelResponse::REJECT;
      }
    },
    [data = _data](const std::shared_ptr<GoalHandle> goal_handle)
    {
      // handle_accepted
      auto node = data->w_node.lock();
      auto it = data->itineraries.find(goal_handle->get_goal_id());
      if (it == data->itineraries.end())
      {
        if (node)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "[handle_accepted] unable to retrieve itinerary"
          );
        }
        return;
      }
      if (node)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Executing transport request..."
        );
      }
      data->transporter->transport_to_destination(
        *(it->second),
        [handle = goal_handle,
        data = data](const Transporter::TransporterState& state)
        {
          auto feedback_msg = std::make_shared<ActionType::Feedback>();
          feedback_msg->state = state;
          handle->publish_feedback(feedback_msg);

          // Publish transporter pose to tf
          auto node = data->w_node.lock();
          geometry_msgs::msg::TransformStamped tf_msg;
          tf_msg.header.stamp =
          node ? node->get_clock()->now() : rclcpp::Clock().now();
          tf_msg.header.frame_id = "world";
          tf_msg.child_frame_id = state.transporter + "_" + state.model;
          tf_msg.transform.translation.x = state.location.pose.position.x;
          tf_msg.transform.translation.y = state.location.pose.position.y;
          tf_msg.transform.translation.z = state.location.pose.position.z;
          tf_msg.transform.rotation = state.location.pose.orientation;
          data->tf_broadcaster->sendTransform(tf_msg);
        },
        [handle = goal_handle, data = data](bool success)
        {
          auto node = data->w_node.lock();
          auto result_msg = std::make_shared<ActionType::Result>();
          result_msg->success = success;
          if (success)
          {
            if (node)
            {
              RCLCPP_INFO(
                node->get_logger(),
                "Transportation successful!"
              );
            }
            handle->succeed(result_msg);
          }
          else
          {
            if (node)
            {
              RCLCPP_ERROR(
                node->get_logger(),
                "Transportation unsuccessful!"
              );
            }
            handle->abort(result_msg);
          }
        });

    },
    rcl_action_server_get_default_options(),
    _data->cb_group
    );

  RCLCPP_INFO(this->get_logger(), "Successfully configured.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_cleanup(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  this->_register_timer.reset();
  _data->transporter.reset();
  _data->availability_srv.reset();
  _data->action_srv.reset();
  _data->itineraries.clear();
  _data->tf_broadcaster.reset();
  RCLCPP_INFO(this->get_logger(), "Successfully cleaned up.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_shutdown(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_activate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Activating...");
  RCLCPP_INFO(this->get_logger(), "Successfully activated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_deactivate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  for (auto it = _data->itineraries.begin(); it != _data->itineraries.end();
    ++it)
  {
    _data->transporter->cancel(*(it->second));
  }
  RCLCPP_INFO(this->get_logger(), "Successfully deactivated.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_error(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_ERROR(this->get_logger(), "Error!");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
TransporterNode::TransporterNode(const rclcpp::NodeOptions& options)
: LifecycleNode("transporter_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Running...");

  _data = std::make_shared<Data>();

  _data->cb_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  _data->transporter_plugin_name = this->declare_parameter(
    "transporter_plugin",
    std::string(""));
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter transporter_plugin to [%s]",
    _data->transporter_plugin_name.c_str()
  );

  const std::size_t timeout_seconds =
    this->declare_parameter("connection_timeout", 5);
  _data->connection_timeout = std::chrono::seconds(5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter connection_timeout to [%zd] seconds",
    timeout_seconds
  );
}

bool TransporterNode::registration_callback()
{
  RCLCPP_INFO(this->get_logger(), "Registering with system orchestrator...");
  auto client = this->create_client<RegisterTransporter>(
    RegisterTransporterService::service_name(),
    rmw_qos_profile_services_default,
    _data->cb_group
  );

  if (!client->wait_for_service(_data->connection_timeout))
  {
    RCLCPP_ERROR(this->get_logger(), "Could not find system orchestrator!");
    return false;
  }

  auto req = std::make_shared<RegisterTransporter::Request>();
  req->description.workcell_id = this->get_name();
  auto fut = client->async_send_request(req);

  if (fut.wait_for(_data->connection_timeout) != std::future_status::ready)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Could not register with system orchestrator");
    return false;
  }
  auto resp = fut.get();
  if (!resp->success)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to register with system orchestrator (%s)",
      resp->message.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
    "Successfully registered with system orchestrator");
  return true;
}

} // namespace nexus_transporter

RCLCPP_COMPONENTS_REGISTER_NODE(nexus_transporter::TransporterNode)
