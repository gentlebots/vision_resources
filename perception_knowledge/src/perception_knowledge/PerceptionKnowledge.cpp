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

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace perception_knowledge
{

PerceptionKnowledge::PerceptionKnowledge(std::string node_name)
: LifecycleNode(node_name)
{
  std::cout << "Hello World" << std::endl;
}

CallbackReturnT
PerceptionKnowledge::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_WARN(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  // Get the Parameters

  getParams();

  // Set the Subscribers and Publishers

  return CallbackReturnT::SUCCESS;
}

void
PerceptionKnowledge::update()
{
  RCLCPP_INFO(this->get_logger(), "Update!");
}

void
PerceptionKnowledge::getParams()
{

}

} // namespace perception_knowledge