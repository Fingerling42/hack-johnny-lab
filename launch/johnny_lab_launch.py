from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Launch arguments
launch_args = [
    DeclareLaunchArgument(
        name='pubsub_params_path',
        description='Path to pubsub config file with parameters',
    ),
    DeclareLaunchArgument(
        name='navigator_params_path',
        description='Path to navigator config file with parameters',
    ),
    DeclareLaunchArgument(
        name='namespace',
        description='Robot namespace',
        default_value=''
    ),
    DeclareLaunchArgument(
        name='seeds_file_path',
        description='Path to file that contains all seeds',
    ),
]


def generate_launch_description():
    ld = LaunchDescription(launch_args)

    # Prepare config files
    config_localization = os.path.join(
        get_package_share_directory('turtlebot4_johnny_lab'),
        'config',
        'turtlebot4_localization.yaml'
    )

    config_navigation = os.path.join(
        get_package_share_directory('turtlebot4_johnny_lab'),
        'config',
        'turtlebot4_nav2.yaml'
    )

    # Robonomics pubsub node with param path
    robonomics_pubsub_node = Node(
        package='robonomics_ros2_pubsub',
        executable='robonomics_ros2_pubsub',
        emulate_tty=True,
        parameters=[{
            'pubsub_params_path': LaunchConfiguration('pubsub_params_path')
        }],
    )

    # Robonomics handler for Johnny Lab
    johnny_lab_robonomics = Node(
        package='turtlebot4_johnny_lab',
        executable='johnny_lab_robonomics',
        emulate_tty=True,
    )

    # Navigator node for Johnny Lab
    johnny_lab_navigator = Node(
        package='turtlebot4_johnny_lab',
        executable='johnny_lab_navigator',
        emulate_tty=True,
        parameters=[{
            'navigator_params_path': LaunchConfiguration('navigator_params_path'),
            'seeds_file_path': LaunchConfiguration('seeds_file_path'),
        }],
    )

    namespace_launch_action = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            robonomics_pubsub_node,
            johnny_lab_robonomics
        ]
    )

    # Adding launch files from Turtlebot 4 Navigation stack
    turtlebot4_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/localization.launch.py']),
        launch_arguments={
            'params': config_localization,
        }.items()
    )

    turtlebot4_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/nav2.launch.py']),
        launch_arguments={
            'params_file': config_navigation,
        }.items()
    )

    nav2_timer = TimerAction(
        period=20.0,
        actions=[turtlebot4_navigation]
    )

    # ld.add_action(turtlebot4_localization)
    # ld.add_action(nav2_timer)
    ld.add_action(namespace_launch_action)
    ld.add_action(johnny_lab_navigator)

    return ld
