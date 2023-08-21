// Copyright 2022 Johnson & Johnson
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

#include <nexus_endpoints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

//==============================================================================
class MockPrinter : public rclcpp_lifecycle::LifecycleNode
{
public:
  using DispenserService = nexus::endpoints::DispenserService;
  using ServiceType = DispenserService::ServiceType;

  MockPrinter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("mock_printer", options)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mock printer is running..."
    );

    bool autostart;
    this->declare_parameter<bool>("autostart", false);
    this->get_parameter<bool>("autostart", autostart);
    // If 'autostart' parameter is true, the node self-transitions to 'active' state upon startup
    if (autostart)
    {
      this->configure();
      this->activate();
    }
  }

  ~MockPrinter()
  {
    if (this->get_current_state().label() != "unconfigured"
      && this->get_current_state().label() != "finalized")
    {
      on_cleanup(this->get_current_state());
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock printer configuring...");

    _srv = this->create_service<ServiceType>(
      DispenserService::service_name(this->get_name()),
      [this](
        ServiceType::Request::ConstSharedPtr request,
        ServiceType::Response::SharedPtr response)
      {
        if (this->get_current_state().label() != "active")
        {
          RCLCPP_ERROR(this->get_logger(), "Mock printer is not activated. "
          "Rejecting request!");
          return;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "Received request to print label from requester [%s] with payload "
          "[%s].",
          request->requester.c_str(),
          request->payload.c_str()
        );

        // Always return success
        response->success = true;
      }
    );

    RCLCPP_INFO(this->get_logger(), "Mock printer configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock printer activating...");

    RCLCPP_INFO(this->get_logger(), "Mock printer activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock printer deactivating...");

    RCLCPP_INFO(this->get_logger(), "Mock printer deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock printer cleaning up...");

    _srv.reset();

    RCLCPP_INFO(this->get_logger(), "Mock printer cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override
  {
    RCLCPP_INFO(this->get_logger(), "Mock printer shutting down...");

    RCLCPP_INFO(this->get_logger(), "Mock printer has shut down");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Service<ServiceType>::SharedPtr _srv;
}; // class MockPrinter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MockPrinter)
