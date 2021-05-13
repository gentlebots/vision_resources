# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os

params_file = "config/config.yaml"

def generate_launch_description():
  # Load Parameters

  pkg_dir = get_package_share_directory('perception_knowledge')
  config_file_path = os.path.join(pkg_dir, params_file)
  print(pkg_dir)
  print(config_file_path)

  stdout_use_envvar = SetEnvironmentVariable(
    'RCUTILS_LOGGING_USE_STDOUT', '1')

  stdout_buffer_envvar = SetEnvironmentVariable(
    'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

  # Create the node

  perception_knowledge_node = Node(
    package='perception_knowledge',
    executable='perception_knowledge_node',
    name='perception_knowledge_node',
    output='screen',
    parameters=[config_file_path]
  )

  ld = LaunchDescription()
  ld.add_action(stdout_use_envvar)
  ld.add_action(stdout_buffer_envvar)
  ld.add_action(perception_knowledge_node)

  return ld