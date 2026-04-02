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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_metrics = LaunchConfiguration('use_metrics')

    bow_path = Node(
        package='husky_lqr',
        executable='bow_coverage_path_publisher',
        name='bow_coverage_path_publisher',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'lane_spacing': LaunchConfiguration('lane_spacing'),
            'length': LaunchConfiguration('length'),
            'width': LaunchConfiguration('width'),
            'turn_radius': LaunchConfiguration('turn_radius'),
            'resolution': LaunchConfiguration('resolution'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'pass_count': LaunchConfiguration('pass_count'),
            'closed_loop': LaunchConfiguration('closed_loop'),
            'origin_x': LaunchConfiguration('origin_x'),
            'origin_y': LaunchConfiguration('origin_y'),
        }],
    )

    rviz_config = os.path.join(
        get_package_share_directory('husky_lqr'),
        'rviz',
        'lqr_tracking.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    lqr_tracker = Node(
        package='husky_lqr',
        executable='lqr_tracker_node',
        name='lqr_tracker_node',
        output='screen',
        parameters=[{
            'control_rate': 30.0,
            'lookahead_dist': 0.4,
            'goal_tolerance': 0.1,
            'min_lqr_speed': 0.10,
            'kappa_speed_eps': 1e-3,
            'base_v_ref': 0.5,
            'v_min': 0.0,
            'v_max': 1.0,
            'w_max': 1.5,
            'accel_limit': 0.1,
            'decel_limit': 0.8,
            'w_accel_limit': 1.8,
            'q_ex': 2.0,
            'q_ey': 4.0,
            'q_etheta': 3.0,
            'r_v': 1.0,
            'r_w': 1.8,
        }],
    )

    metrics = Node(
        package='husky_lqr',
        executable='tracking_metrics_node',
        name='tracking_metrics_node',
        output='screen',
        condition=IfCondition(use_metrics),
        parameters=[{'report_rate': 1.0}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('lane_spacing', default_value='0.25'),
        DeclareLaunchArgument('length', default_value='12.0'),
        DeclareLaunchArgument('width', default_value='10.0'),
        DeclareLaunchArgument('turn_radius', default_value='1.0'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_rate', default_value='1.0'),
        DeclareLaunchArgument('pass_count', default_value='0'),
        DeclareLaunchArgument('closed_loop', default_value='false'),
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='0.0'),
        DeclareLaunchArgument('use_metrics', default_value='true'),
        bow_path,
        lqr_tracker,
        metrics,
        rviz2,
    ])
