from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'diff_bot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'diff_drive_sim.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description_content = urdf_file.read()

    sim_node = Node(
        package='diff_drive_sim',
        executable='diff_drive_sim_node',
        name='diff_drive_sim_node',
        output='screen',
        parameters=[{
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'update_rate': LaunchConfiguration('update_rate'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'left_wheel_joint_name': 'left_wheel_joint',
            'right_wheel_joint_name': 'right_wheel_joint',
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('wheel_base', default_value='0.5'),
        DeclareLaunchArgument('wheel_radius', default_value='0.1'),
        DeclareLaunchArgument('update_rate', default_value='50.0'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        sim_node,
        robot_state_publisher,
        rviz_node,
    ])
