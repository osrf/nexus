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

#ifndef SRC__CAPABILITIES__DETECTION_HPP
#define SRC__CAPABILITIES__DETECTION_HPP

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common/service_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <string>
#include <functional>
#include <memory>

namespace nexus::capabilities {

/**
 * Detect offsets of an item.
 *
 * Input Ports:
 *   detector |std::string| Id of the detector.
 *   item |std::string| The item to detect.
 * Output Ports:
 *   result |geometry_msgs::msg::Pose| Detected offset.
 */
class DetectOffset : public nexus::common::
  ServiceClientBtNode<endpoints::DetectorService::ServiceType>
{
public:
  using GetClientCallback = std::function<rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedPtr(
        const std::string& detector)>;

  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts();

/**
 * @param name name of the BT node.
 * @param config config of the BT node.
 * @param logger rclcpp logger to use.
 * @param ctx_mgr context manager.
 * @param get_client_cb Callback that returns the client to use.
 */
public: inline DetectOffset(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Logger logger,
    std::shared_ptr<const ContextManager> ctx_mgr,
    GetClientCallback get_client_cb
)
  : ServiceClientBtNode<endpoints::DetectorService::ServiceType>(name, config,
      logger), _ctx_mgr(ctx_mgr), _get_client_cb(get_client_cb) {}

protected: rclcpp::Client<endpoints::DetectorService::ServiceType>::
  SharedPtr client() override;

protected: endpoints::DetectorService::ServiceType::Request::SharedPtr
  make_request() override;

protected: bool on_response(
    rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedResponse resp)
  override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
private: GetClientCallback _get_client_cb;
};

/**
 * Detect all items.
 *
 * Input Ports:
 *   detector |std::string| Id of the detector.
 *   items |std::vector<std::string>| Items to detect.
 * Output Ports:
 *   result |vision_msgs::msg::Detection3DArray| Detected items.
 */
class DetectAllItems : public BT::StatefulActionNode
{
public:
  using GetClientCallback = std::function<rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedPtr(
        const std::string& detector)>;

  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts();

  /**
   * @param name name of the BT node.
   * @param config config of the BT node.
   * @param node rclcpp lifecycle node.
   * @param client_factory a callable that returns the rclcpp client to use.
   */
public: inline DetectAllItems(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<const ContextManager> ctx_mgr,
    GetClientCallback get_client_cb
)
  : BT::StatefulActionNode(name, config), _node(node), _ctx_mgr(ctx_mgr),
    _get_client_cb(get_client_cb) {}

public: BT::NodeStatus onStart() override;

public: BT::NodeStatus onRunning() override;

public: void onHalted() override;

private: rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
private: std::shared_ptr<const ContextManager> _ctx_mgr;
private: GetClientCallback _get_client_cb;
private: std::unordered_set<std::string> _items;
private: decltype(_items)::const_iterator _cur_it;
private: rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedPtr
    _client;
private: std::optional<rclcpp::Client<endpoints::DetectorService::ServiceType>::
    FutureAndRequestId> _current_req;
private: vision_msgs::msg::Detection3DArray _results;

private: void _send_next_request();
};

/**
 * Get or find a detection from a detections array.
 *
 * Input Ports:
 *   detections |vision_msgs::msg::Detection3DArray| Array of detections.
 *   idx |size_t| Index of the detection to get. Ignored if `id` is also provided.
 *   id |std::string| Id of the detection to find. Takes precedence over `idx`.
 * Output Ports:
 *   result |vision_msgs::msg::Detection3D| The detection found.
 */
class GetDetection : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<vision_msgs::msg::Detection3DArray>("detections"),
      BT::InputPort<size_t>("idx",
        "Index of the detection to get. Ignored if `id` is also provided."),
      BT::InputPort<std::string>("id",
        "Id of the detection to find. Takes precedence over `idx`."),
      BT::OutputPort<vision_msgs::msg::Detection3D>("result") };
  }

public: GetDetection(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr)
  : BT::SyncActionNode(name, config), _ctx_mgr(std::move(ctx_mgr)) {}

protected: BT::NodeStatus tick() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

/**
 * Get the pose stamped of a detection.
 *
 * Input Ports:
 *   detection |vision_msgs::msg::Detection3D|
 * Output Ports:
 *   result |geometry_msgs::msg::PoseStamped| The detection found.
 */
class GetDetectionPose : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<vision_msgs::msg::Detection3D>("detection"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("result") };
  }

public: GetDetectionPose(const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr)
  : BT::SyncActionNode(name, config), _ctx_mgr(std::move(ctx_mgr)) {}

protected: BT::NodeStatus tick() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
};

}

#endif
