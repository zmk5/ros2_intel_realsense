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
    """Launch multiple RealSense camera devices."""
    # config the serial number and base frame id of each camera
    camera1_base_frame_id = LaunchConfiguration(
        'base_frame_id', default='camera1_link')
    camera2_base_frame_id = LaunchConfiguration(
        'base_frame_id', default='camera2_link')
    camera3_base_frame_id = LaunchConfiguration(
        'base_frame_id', default='camera3_link')
    camera1_serial_no = LaunchConfiguration(
        'serial_no', default='837212070294')
    camera2_serial_no = LaunchConfiguration(
        'serial_no', default='819312071869')
    camera3_serial_no = LaunchConfiguration(
        'serial_no', default='845412111144')


    camera1_node = Node(
        package='realsense_node',
        executable='realsense_node',
        namespace="/camera1",
        output='screen',
        parameters=[{'serial_no':camera1_serial_no,
                    'base_frame_id': camera1_base_frame_id}]
        )
    camera2_node = Node(
        package='realsense_node',
        executable='realsense_node',
        namespace="/camera2",
        output='screen',
        parameters=[{'serial_no':camera2_serial_no, 
                    'base_frame_id': camera2_base_frame_id}]
        )
    camera3_node = Node(
        package='realsense_node',
        executable='realsense_node',
        namespace="/camera3",
        output='screen',
        parameters=[{'serial_no':camera3_serial_no,
                    'base_frame_id': camera3_base_frame_id}]
        )
    return LaunchDescription([camera1_node, camera2_node, camera3_node])
