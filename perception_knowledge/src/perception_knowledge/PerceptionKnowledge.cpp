// Copyright 2021 Intelligent Robotics Lab
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

#include <iostream>
#include "perception_knowledge/PerceptionKnowledge.hpp"

using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace perception_knowledge
{

PerceptionKnowledge::PerceptionKnowledge(std::string node_name)
: LifecycleNode(node_name)
{
  // Declare the parameters

  this->declare_parameter<std::string>("messages_type", "vision_msgs");
  this->declare_parameter<std::string>(
    "detections_topic", "/vision_msgs_topic");
}

CallbackReturnT
PerceptionKnowledge::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_WARN(
    this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  // Get the Parameters

  getParams();

  // Set the Subscribers and Publishers

  setSubscribers();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PerceptionKnowledge::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_WARN(
    this->get_logger(), "[%s] Activating from [%s] state...",
    this->get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void
PerceptionKnowledge::update()
{
  for (auto det : visionDetections_)
  {
    printf("%s\n", det.object_name);
  }
  printf("\n----------\n");
}

void
PerceptionKnowledge::setSubscribers()
{
  if (msgsType_ == "vision_msgs") {
    visionMsgsSubscriber_ =
      this->create_subscription<vision_msgs::msg::Detection3DArray>(
        detectionsTopic_, rclcpp::SensorDataQoS(),
        std::bind(&PerceptionKnowledge::visionsMsgsCallback, this, std::placeholders::_1));
  }
  else if (msgsType_ == "gb_visual_detection_3d_msgs") {
    // Subscribe to gb messages
  }
}

void
PerceptionKnowledge::visionsMsgsCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  // Remove the old objects

  visionDetections_.clear();

  // Save the frame id

  fameId_ = msg->header.frame_id;

  // Set de new objects detected

  ObjectType obj;
  for (auto detection : msg->detections)
  {
    obj.object_name = detection.tracking_id;
    obj.position = detection.bbox.center.position;
    obj.orientation = detection.bbox.center.orientation;

    visionDetections_.push_back(obj);
  }
}

void
PerceptionKnowledge::getParams()
{
  this->get_parameter("messages_type", msgsType_);
  this->get_parameter("detections_topic", detectionsTopic_);
}
} // namespace perception_knowledge
