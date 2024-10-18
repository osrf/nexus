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

#include "detection.hpp"

#include <nexus_common/pretty_print.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace nexus::capabilities {

BT::PortsList DetectOffset::providedPorts()
{
  return { BT::InputPort<std::string>("detector"), BT::InputPort<std::string>(
      "item"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("result") };
}

rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedPtr
DetectOffset::client()
{
  auto detector = this->getInput<std::string>("detector");
  if (!detector)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [detector] is required",
      this->name().c_str());
    return nullptr;
  }
  return this->_get_client_cb(*detector);
}

endpoints::DetectorService::ServiceType::Request::SharedPtr DetectOffset::
make_request()
{
  auto req =
    std::make_shared<endpoints::DetectorService::ServiceType::Request>();
  auto id = this->getInput<std::string>("item");
  if (!id)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [id] is required",
      this->name().c_str());
    return nullptr;
  }
  req->id = *id;
  req->payload = YAML::Dump(this->_ctx_mgr->current_context()->task.data);
  return req;
}

bool DetectOffset::on_response(
  rclcpp::Client<endpoints::DetectorService::ServiceType>::SharedResponse resp)
{
  if (!resp->success)
  {
    return false;
  }

  auto item = this->getInput<std::string>("item");
  if (!item)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [item] is required",
      this->name().c_str());
    return false;
  }
  auto found_detection = std::find_if(
    resp->detections.detections.begin(), resp->detections.detections.end(),
    [&item](
      const vision_msgs::msg::Detection3D& i)
    {
      return i.id == item;
    });
  if (found_detection == resp->detections.detections.end())
  {
    // cannot find item
    return false;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = found_detection->header;
  pose.pose = found_detection->bbox.center;
  RCLCPP_DEBUG_STREAM(this->_logger,
    *item << " is at " << pose.pose << " frame [" << pose.header.frame_id <<
      "]");
  this->setOutput("result", pose);
  return true;
}

BT::PortsList DetectAllItems::providedPorts()
{
  return { BT::InputPort<std::string>("detector"),
    BT::InputPort<std::vector<std::string>>("items"),
    BT::OutputPort<vision_msgs::msg::Detection3DArray>("result") };
}

BT::NodeStatus DetectAllItems::onStart()
{
  this->onHalted();
  auto input_items = this->getInput<std::vector<std::string>>("items");
  if (!input_items)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [items] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_items = std::unordered_set<std::string>(
    input_items->cbegin(), input_items->cend());

  if (this->_items.empty())
  {
    RCLCPP_WARN(this->_node->get_logger(), "no input items given to detect");
    return BT::NodeStatus::SUCCESS;
  }
  auto detector = this->getInput<std::string>("detector");
  if (!detector)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [detector] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_client = this->_get_client_cb(*detector);
  this->_cur_it = this->_items.cbegin();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectAllItems::onRunning()
{
  if (this->_cur_it == this->_items.cend())
  {
    this->setOutput("result", this->_results);
    return BT::NodeStatus::SUCCESS;
  }

  if (!this->_current_req.has_value())
  {
    this->_send_next_request();
    return BT::NodeStatus::RUNNING;
  }

  if (this->_current_req->wait_for(std::chrono::seconds{0}) !=
    std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto resp = this->_current_req->get();
  if (!resp->success)
  {
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG(
    this->_node->get_logger(), "detecting [%s]", this->_cur_it->c_str());
  for (const auto& d : resp->detections.detections)
  {
    if (d.id != *this->_cur_it)
    {
      RCLCPP_WARN(
        this->_node->get_logger(), "detector requested to detect [%s] but it responded with [%s], ignoring the result",
        this->_cur_it->c_str(), d.id.c_str());
      continue;
    }
    RCLCPP_DEBUG_STREAM(this->_node->get_logger(),
      "found [" << *this->_cur_it << "] at " << d.bbox.center);
    if (this->_results.header.frame_id.empty())
    {
      this->_results.header = d.header;
    }
    else if (this->_results.header.frame_id != d.header.frame_id)
    {
      RCLCPP_WARN(
        this->_node->get_logger(),
        "detector returned different frame id from previous result");
    }
    this->_results.detections.emplace_back(d);
  }
  ++this->_cur_it;
  this->_current_req.reset();
  return BT::NodeStatus::RUNNING;
}

void DetectAllItems::onHalted()
{
  this->_cur_it = this->_items.cend();
  this->_client.reset();
  this->_results = vision_msgs::msg::Detection3DArray{};
  if (this->_current_req.has_value())
  {
    // cancelling is not supported, wait for the ongoing request to finish.
    this->_current_req->get();
  }
  this->_current_req.reset();
}

void DetectAllItems::_send_next_request()
{
  auto ctx = this->_ctx_mgr->current_context();
  auto req =
    std::make_shared<endpoints::DetectorService::ServiceType::Request>();
  req->id = *this->_cur_it;
  req->payload = YAML::Dump(ctx->task.data);
  this->_current_req = this->_client->async_send_request(req);
}

BT::NodeStatus GetDetection::tick()
{
  auto ctx = this->_ctx_mgr->current_context();
  auto detections = this->getInput<vision_msgs::msg::Detection3DArray>(
    "detections");
  if (!detections)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: port [detections] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto idx = this->getInput<size_t>("idx");
  auto id = this->getInput<std::string>("id");
  if (!idx && !id)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: port [idx] or [id] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (id)
  {
    const auto it = std::find_if(detections->detections.cbegin(),
        detections->detections.cend(),
        [&id](const vision_msgs::msg::Detection3D& i)
        {
          return i.id == *id;
        });
    if (it == detections->detections.cend())
    {
      RCLCPP_ERROR(
        ctx->node.get_logger(), "%s: cannot find detection with id [%s]",
        this->name().c_str(), id->c_str());
      return BT::NodeStatus::FAILURE;
    }
    this->setOutput("result", *it);
  }
  else
  {
    this->setOutput("result", detections->detections.at(*idx));
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetDetectionPose::tick()
{
  auto ctx = this->_ctx_mgr->current_context();
  auto detection = this->getInput<vision_msgs::msg::Detection3D>("detection");
  if (!detection)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: port [detection] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped result;
  result.header = detection->header;
  result.pose = detection->bbox.center;
  this->setOutput("result", result);
  return BT::NodeStatus::SUCCESS;
}

}
