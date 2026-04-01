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
    rviz_path_builder = Node(
        package='husky_lqr',
        executable='rviz_path_builder_node',
        name='rviz_path_builder_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'min_point_distance': 0.10,
            'interpolation_per_segment': 10,
            'resample_step': 0.10,
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
            'base_v_ref': 0.6,
            'v_min': 0.0,
            'v_max': 1.0,
            'w_max': 1.5,
            'accel_limit': 0.6,
            'decel_limit': 0.8,
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
        rviz_path_builder,
        lqr_tracker,
        rviz2,
    ])
