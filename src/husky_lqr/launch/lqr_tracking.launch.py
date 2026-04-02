# Copyright 2026 wt
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bow_path = Node(
        package='husky_lqr',
        executable='bow_coverage_path_publisher',
        name='bow_coverage_path_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'lane_spacing': 0.8,
            'length': 12.0,
            'width': 4.0,
            'turn_radius': 1.0,
            'resolution': 0.05,
            'publish_rate': 1.0,
            'origin_x': 0.0,
            'origin_y': 0.0,
        }],
    )

    lqr_tracker = Node(
        package='husky_lqr',
        executable='lqr_tracker_node',
        name='lqr_tracker_node',
        output='screen',
        parameters=[{
            'control_rate': 30.0,
            'lookahead_dist': 0.2,
            'goal_tolerance': 0.25,
            'min_lqr_speed': 0.10,
            'kappa_speed_eps': 1e-3,
            'base_v_ref': 0.2,
            'v_min': 0.0,
            'v_max': 1.0,
            'w_max': 1.5,
            'accel_limit': 0.6,
            'decel_limit': 1.2,
            'w_accel_limit': 1.8,
            'q_ex': 2.0,
            'q_ey': 4.0,
            'q_etheta': 3.0,
            'r_v': 1.0,
            'r_w': 1.5,
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/wt/lqr_ws/src/husky_lqr/rviz/lqr_tracking.rviz'],
        output='screen',
    )

    return LaunchDescription([
        bow_path,
        lqr_tracker,
        rviz2,
    ])
