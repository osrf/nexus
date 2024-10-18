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

#include <thread>
#include <memory>

#include <rmf_utils/catch.hpp>

#include <nexus_lifecycle_manager/lifecycle_manager.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>

class LifecycleNodeExample : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// LifecycleNodeExample constructor
  explicit LifecycleNodeExample(const std::string& node_name)
  : rclcpp_lifecycle::LifecycleNode(
      node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    LifecycleNode::on_activate(state);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    LifecycleNode::on_deactivate(state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
  }
};

static const std::chrono::nanoseconds DEFAULT_EVENT_TIMEOUT =
  std::chrono::seconds(3);
static const std::chrono::nanoseconds DEFAULT_EVENT_SLEEP_PERIOD =
  std::chrono::milliseconds(100);

static
bool wait_for_event(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  std::function<bool()> predicate,
  std::chrono::nanoseconds timeout,
  std::chrono::nanoseconds sleep_period)
{
  auto start = std::chrono::steady_clock::now();
  std::chrono::microseconds time_slept(0);

  bool predicate_result;
  while (!(predicate_result = predicate()) &&
    time_slept < std::chrono::duration_cast<std::chrono::microseconds>(timeout))
  {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - start);
  }
  return predicate_result;
}

static
bool wait_for_service_by_node(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string& node_name,
  const std::string& service,
  std::chrono::nanoseconds timeout = DEFAULT_EVENT_TIMEOUT,
  std::chrono::nanoseconds sleep_period = DEFAULT_EVENT_SLEEP_PERIOD)
{
  return wait_for_event(
    node,
    [node, node_name,
    service]()
    {
      auto service_names_and_types_by_node = node->get_service_names_and_types_by_node(
        node_name,
        "");
      return service_names_and_types_by_node.end() != service_names_and_types_by_node.find(
        service);
    },
    timeout,
    sleep_period);
}

std::mutex shutdown_mutex;
class SpinNode
{
public: SpinNode(std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& _node)
  : node_(_node) {}

public: void spin()
  {
    while (true)
    {
      {
        std::lock_guard<std::mutex> guard(shutdown_mutex);
        if (!rclcpp::ok())
        {
          break;
        }
        rclcpp::spin_some(this->node_->get_node_base_interface());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

//==============================================================================
SCENARIO("Test Lifecycle Manager")
{
  const char* argv[] = {"test", "--ros-args", "--log-level", "info"};
  rclcpp::init(4, argv);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node =
    std::make_shared<LifecycleNodeExample>("lifecycle_manager");

  std::vector<std::string> node_names;

  auto lifecycle_manager =
    std::make_shared<nexus::lifecycle_manager::LifecycleManager<rclcpp::Node>>(
    "lifecycle_manager",
    node_names);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node1 =
    std::make_shared<LifecycleNodeExample>("node1");
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node2 =
    std::make_shared<LifecycleNodeExample>("node2");
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node3 =
    std::make_shared<LifecycleNodeExample>("node3");

  // These are specific to lifecycle nodes, other services are provided by rclcpp::Node
  CHECK(wait_for_service_by_node(node1, "node1", "/node1/change_state"));
  CHECK(wait_for_service_by_node(node1, "node1",
    "/node1/get_available_states"));
  CHECK(
    wait_for_service_by_node(node1, "node1",
    "/node1/get_available_transitions"));
  CHECK(wait_for_service_by_node(node1, "node1", "/node1/get_state"));
  CHECK(wait_for_service_by_node(node1, "node1",
    "/node1/get_transition_graph"));

  CHECK(wait_for_service_by_node(node2, "node2", "/node2/change_state"));
  CHECK(wait_for_service_by_node(node2, "node2",
    "/node2/get_available_states"));
  CHECK(
    wait_for_service_by_node(node2, "node2",
    "/node2/get_available_transitions"));
  CHECK(wait_for_service_by_node(node2, "node2", "/node2/get_state"));
  CHECK(wait_for_service_by_node(node2, "node2",
    "/node2/get_transition_graph"));

  CHECK(wait_for_service_by_node(node3, "node3", "/node3/change_state"));
  CHECK(wait_for_service_by_node(node3, "node3",
    "/node3/get_available_states"));
  CHECK(
    wait_for_service_by_node(node3, "node3",
    "/node3/get_available_transitions"));
  CHECK(wait_for_service_by_node(node3, "node3", "/node3/get_state"));
  CHECK(wait_for_service_by_node(node3, "node3",
    "/node3/get_transition_graph"));

  RCLCPP_INFO(node->get_logger(), "Checked topics in node1, node2 and node3");

  SpinNode spinNode(node);
  SpinNode spinNode1(node1);
  SpinNode spinNode2(node2);
  SpinNode spinNode3(node3);

  std::thread t(&SpinNode::spin, &spinNode);
  std::thread t1(&SpinNode::spin, &spinNode1);
  std::thread t2(&SpinNode::spin, &spinNode2);
  std::thread t3(&SpinNode::spin, &spinNode3);

  CHECK(lifecycle_manager->addNodeName("node1"));
  RCLCPP_INFO(node->get_logger(), "Added node 1");

  CHECK(lifecycle_manager->addNodeName("node2"));
  RCLCPP_INFO(node->get_logger(), "Added node 2");

  CHECK(static_cast<int>(node1->get_current_state().id()) ==
    static_cast<int>(node2->get_current_state().id()));

  lifecycle_manager->changeStateForAllNodes(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  lifecycle_manager->changeStateForAllNodes(
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

  CHECK(static_cast<int>(node1->get_current_state().id()) ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  CHECK(static_cast<int>(node1->get_current_state().id()) ==
    static_cast<int>(node2->get_current_state().id()));

  node3->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node3->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  CHECK(lifecycle_manager->addNodeName("node3"));
  RCLCPP_INFO(node->get_logger(), "Added node 3");

  CHECK(static_cast<int>(node3->get_current_state().id()) ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  CHECK(static_cast<int>(node1->get_current_state().id()) ==
    static_cast<int>(node3->get_current_state().id()));

  lifecycle_manager->changeStateForAllNodes(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  CHECK(static_cast<int>(node3->get_current_state().id()) ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  CHECK(static_cast<int>(node1->get_current_state().id()) ==
    static_cast<int>(node3->get_current_state().id()));

  {
    std::lock_guard<std::mutex> guard(shutdown_mutex);
    rclcpp::shutdown();
  }
  t.join();
  t1.join();
  t2.join();
  t3.join();
}
