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

#ifndef PERCEPCION_KNOWLEDGE__PERCEPTIONKNOWLEDGE_HPP_
#define PERCEPCION_KNOWLEDGE__PERCEPTIONKNOWLEDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <string>
#include <memory>

namespace perception_knowledge
{

class PerceptionKnowledge : public rclcpp_lifecycle::LifecycleNode
{
public:
  PerceptionKnowledge(std::string node_name);
  void update();
private:
  void getParams();

  rclcpp::Subscription<vision_msgs::msg::BoundingBox3D>::SharedPtr
    visionMsgsSubscriber;

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  // CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  // CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  // CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  // CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  // CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
};

} // namespace percepcion_knowledge

#endif