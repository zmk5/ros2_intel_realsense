# Copyright (c) 2019 Intel Corporation
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

# /* Author: Gary Liu */
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launch simple RealSense camera node."""
    rs_param_dir = LaunchConfiguration(
        'rs_param_dir',
        default=os.path.join(
            get_package_share_directory('realsense_examples'),
            'config',
            'd435.yaml')
    )
    rgbd_node = Node(
        package='realsense_node',
        executable='realsense_node',
        namespace='',
        output='screen',
        parameters=[rs_param_dir],
        arguments=['/d435'],
        )
    return LaunchDescription([rgbd_node])
