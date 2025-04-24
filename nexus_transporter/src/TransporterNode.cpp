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

#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>

//==============================================================================
namespace nexus_transporter {

//==============================================================================
TransporterNode::Data::Data()
: transporter_loader("nexus_transporter", "nexus_transporter::Transporter"),
  transporter(nullptr),
  availability_srv(nullptr),
  action_srv(nullptr),
  tf_broadcaster(nullptr),
  ongoing_register(std::nullopt)
{
  // Do nothing.
}

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
        "Received IsTransporterAvailable request from %s with id %s. ",
        request->request.requester.c_str(),
        request->request.id.c_str()
      );

      if (request->request.destinations.empty())
      {
        RCLCPP_WARN(
          node->get_logger(),
          "Ignoring request as destinations is empty."
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

      auto shared_promise =
      std::make_shared<std::promise<std::optional<Itinerary>>>();
      auto future = shared_promise->get_future();
      data->transporter->get_itinerary(
        request->request.id,
        request->request.destinations,
        [promise = shared_promise](
          std::optional<Itinerary> itinerary)
        {
          promise->set_value(itinerary);
          return;
        }
      );
      // parameter
      if (future.wait_for(data->wait_for_itinerary_timeout)
      == std::future_status::ready)
      {
        const auto itinerary = future.get();
        if (!itinerary.has_value())
        {
          RCLCPP_WARN(
            node->get_logger(),
            "The transporter is not configured to go to destinations."
          );
          return;
        }

        // Success
        response->available = true;
        response->transporter = itinerary->transporter_name();
        response->estimated_finish_time = itinerary->estimated_finish_time();
      }

      // Timed out
      return;
    });

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

  // Create the client for registration.
  this->_data->register_client = this->create_client<RegisterTransporter>(
    RegisterTransporterService::service_name());

  //Timer for registering with the system orchestrator
  this->_data->register_timer = this->create_wall_timer(std::chrono::seconds{1},
      [this]()
      {
        this->_data->_register();
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
          "Received transport goal request from [%s] with id [%s].",
          goal->request.requester.c_str(),
          goal->request.id.c_str()
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

      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
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
            "Successfully cancelled transport with id [%s].",
            it->second->id().c_str()
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
            "Unable to cancel transport with id [%s]",
            it->second->id().c_str()
          );
        }
        return rclcpp_action::CancelResponse::REJECT;
      }
    },
    [data = _data](const std::shared_ptr<GoalHandle> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      data->transporter->get_itinerary(
        goal->request.id,
        goal->request.destinations,
        [handle = goal_handle, data = data](std::optional<Itinerary> itinerary)
        {
          auto result_msg = std::make_shared<ActionType::Result>();
          auto node = data->w_node.lock();
          if (!node)
          {
            result_msg->success = false;
            handle->abort(result_msg);
            return;
          }

          if (!itinerary.has_value())
          {
            RCLCPP_ERROR(
              node->get_logger(),
              "[handle_accepted] No valid itinerary available. Aborting goal..."
            );
            result_msg->success = false;
            handle->abort(result_msg);
            return;
          }

          auto it_pair = data->itineraries.insert(
            handle->get_goal_id(),
            std::make_unique<Itinerary>(std::move(itinerary.value()))
          );
          if (!it_pair.second)
          {
            RCLCPP_ERROR(
              node->get_logger(),
              "[handle_accepted] Found existing itinerary with the same UUID. "
              "Aborting goal..."
            )
          }

          handle->execute();
          RCLCPP_INFO(
            node->get_logger(),
            "Executing transport request..."
          );

          data->transporter->transport_to_destination(
            *(it_pair.first->second),
            [handle = handle, data = data](
              const Transporter::TransporterState& state)
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
            [handle = handle, data = data](bool success)
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

          return;
        }
      );
    });

  RCLCPP_INFO(this->get_logger(), "Successfully configured.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto TransporterNode::on_cleanup(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  _data->register_timer.reset();
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
  _data->connection_timeout = std::chrono::seconds(timeout_seconds);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter connection_timeout to [%zd] seconds",
    timeout_seconds
  );

  const std::size_t wait_for_itinerary_timeout =
    this->declare_parameter("wait_for_itinerary_timeout", 5);
  _data->wait_for_itinerary_timeout =
    std::chrono::seconds(wait_for_itinerary_timeout);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter wait_for_itinerary_timeout to [%zd] seconds",
    wait_for_itinerary_timeout
  );
}

void TransporterNode::Data::_register()
{
  auto node = this->w_node.lock();
  if (node == nullptr)
  {
    return;
  }

  if (this->ongoing_register)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to register: No response from system orchestrator.");
    if (!this->register_client->remove_pending_request(*this->
      ongoing_register))
    {
      RCLCPP_WARN(node->get_logger(),
        "Unable to remove pending request during transporter registration.");
    }
  }

  RCLCPP_INFO(node->get_logger(), "Registering with system orchestrator...");
  auto register_cb =
    [this](rclcpp::Client<RegisterTransporter>::SharedFuture future)
    {
      auto node = this->w_node.lock();
      if (node == nullptr)
      {
        return;
      }
      this->ongoing_register = std::nullopt;
      auto resp = future.get();
      if (!resp->success)
      {
        switch (resp->error_code)
        {
          case RegisterTransporter::Response::ERROR_NOT_READY:
            RCLCPP_ERROR(
              node->get_logger(),
              "Error while registering with system orchestrator, retrying again... [%s]",
              resp->message.c_str());
            break;
          default:
            RCLCPP_FATAL(node->get_logger(),
              "Failed to register with system orchestrator! [%s]",
              resp->message.c_str());
            throw std::runtime_error(resp->message);
        }
        return;
      }
      RCLCPP_INFO(node->get_logger(),
        "Successfully registered with system orchestrator");
      this->register_timer->cancel();
      this->register_timer.reset();
    };

  if (!this->register_client->wait_for_service(connection_timeout))
  {
    RCLCPP_ERROR(node->get_logger(), "Could not find system orchestrator!");
    // timer is not cancelled so it will run again.
    return;
  }

  auto req = std::make_shared<RegisterTransporter::Request>();
  req->description.workcell_id = node->get_name();
  this->ongoing_register =
    this->register_client->async_send_request(req, register_cb);
}

} // namespace nexus_transporter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus_transporter::TransporterNode)
