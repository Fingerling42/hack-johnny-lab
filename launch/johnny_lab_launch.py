from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

from launch.actions import GroupAction, DeclareLaunchArgument
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
]


def generate_launch_description():
    ld = LaunchDescription(launch_args)

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
        }],
    )

    namespace_launch_action = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            robonomics_pubsub_node,
            johnny_lab_robonomics
        ]
    )

    ld.add_action(namespace_launch_action)
    ld.add_action(johnny_lab_navigator)

    return ld
