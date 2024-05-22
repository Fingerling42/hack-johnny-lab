import math
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, LifecycleState, Publisher, State, TransitionCallbackReturn
from typing_extensions import Self, Any, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions

from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from rclpy.executors import MultiThreadedExecutor


class JohnnyLabNavigator(Node):
    def __init__(self) -> None:
        super().__init__('johnny_lab_navigator')

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigator_params_path',
                 rclpy.Parameter.Type.STRING,
                 ParameterDescriptor(description='Path to config file with parameters')),
            ]
        )

        navigator_params_path = self.get_parameter('navigator_params_path').value
        with open(navigator_params_path, 'r') as navigator_config_file:
            navigator_params_dict = yaml.load(navigator_config_file, Loader=yaml.SafeLoader)

        # Load all params
        self.init_position = navigator_params_dict['init']['position']
        self.init_rotation = navigator_params_dict['init']['rotation']

        self.publisher_initial_pose = None
        self.undock_action_client = None
        self.dock_action_client = None
        self.nav_to_pose_action_client = None
        self.subscriber_dock_status = None

        self.action_status_undock = None
        self.action_status_dock = None
        self.action_status_nav_to_pose = None
        self.is_docked = None
        self.initial_pose = None
        self.action_feedback = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested. This callback is being called when
        the lifecycle node enters the configuring state.
        :return: The state machine either invokes a transition to the inactive state or stays
        in "unconfigured" depending on the return value.
        """

        self.get_logger().info('Configuring navigator')

        main_callback_group = MutuallyExclusiveCallbackGroup()

        try:
            self.initial_pose = self.get_pose_stamped(
                self.init_position,
                self.init_rotation * 180 / math.pi)
        except Exception as e:
            self.get_logger().error('Error: %s' % str(e))

        self.publisher_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            qos_profile=qos_profile_system_default,
            callback_group=main_callback_group,
        )

        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_callback,
            qos_profile_sensor_data
        )

        self.undock_action_client = ActionClient(
            self,
            Undock,
            'undock',
            callback_group=main_callback_group
        )

        self.dock_action_client = ActionClient(
            self,
            Dock,
            'dock',
            callback_group=main_callback_group
        )

        self.nav_to_pose_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=main_callback_group
        )

        self.get_logger().info('Configuring is done')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the inactive and
        # enabled state and vice versa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks, and we don't
        # need to write on_activate()/on_deactivate() callbacks.

        self.get_logger().info('Navigator in active state')

        # Wait for dock status
        while self.is_docked is None:
            pass

        # Undock if docked
        if self.is_docked is True:
            self.send_goal_undock()
            while self.action_status_undock != GoalStatus.STATUS_SUCCEEDED:
                if self.action_status_undock == GoalStatus.STATUS_ABORTED:
                    self.action_status_undock = None
                    self.send_goal_undock()
                pass

        # Set the initial pose
        self.set_initial_pose(self.initial_pose)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Navigator in deactivate state')

        # Dock the robot, until success
        self.send_goal_dock()
        while self.action_status_dock != GoalStatus.STATUS_SUCCEEDED:
            if self.action_status_dock == GoalStatus.STATUS_ABORTED:
                self.action_status_dock = None
                self.send_goal_dock()
            pass

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested. on_cleanup callback is being called when
        the lifecycle node enters the "cleaning up" state.
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        """
        self.get_logger().warn('Cleaning up navigator state')

        self.destroy_publisher(self.publisher_initial_pose)
        self.destroy_client(self.undock_action_client)
        self.destroy_client(self.dock_action_client)
        self.destroy_subscription(self.subscriber_dock_status)

        self.is_docked = None
        self.action_status_undock = None
        self.action_status_dock = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested. The callback is being called when the
        lifecycle node enters the "shutting down" state.
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        """
        self.get_logger().warn('Destroying the navigator')

        return TransitionCallbackReturn.SUCCESS

    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """

    def get_pose_stamped(self, position, rotation):
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = math.sin(math.radians(rotation) / 2)
        pose.pose.orientation.w = math.cos(math.radians(rotation) / 2)

        return pose

    def set_initial_pose(self, initial_pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = initial_pose.pose
        msg.header.frame_id = initial_pose.header.frame_id
        msg.header.stamp = initial_pose.header.stamp
        self.get_logger().info('Publishing initial pose')

        self.publisher_initial_pose.publish(msg)

    def dock_status_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def send_goal_undock(self):
        """
        A function for sending goal to undock action
        :return: None
        """

        while not self.undock_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Undock action server not available, waiting...")

        goal_msg = Undock.Goal()
        send_goal_future = self.undock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Cannot undock, please check')
            return
        self.get_logger().info('Going to undock robot from station')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        result = future.result().result.is_docked
        if result is False:
            self.get_logger().info('Undocking is done')
            self.action_status_undock = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().warn('Undocking is failed, trying again')
            self.action_status_undock = GoalStatus.STATUS_ABORTED

    def send_goal_dock(self):
        """
        A function for sending goal to dock action
        :return: None
        """

        while not self.dock_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Dock action server not available, waiting...")

        goal_msg = Dock.Goal()
        send_goal_future = self.dock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.dock_response_callback)

    def dock_response_callback(self, future):
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Cannot dock, please check')
            return
        self.get_logger().info('Going to dock robot to station')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future):
        result = future.result().result.is_docked
        if result is True:
            self.get_logger().info('Docking is done')
            self.action_status_dock = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().warn('Docking is failed, trying again')
            self.action_status_dock = GoalStatus.STATUS_ABORTED

    def send_goal_nav_to_pose(self,
                              pose: PoseStamped,
                              behavior_tree: str = ''):

        while not self.nav_to_pose_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("NavigateToPose action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ', ' + str(pose.pose.position.y))
        send_goal_future = self.nav_to_pose_action_client.send_goal_async(goal_msg, self.action_feedback_callback)
        send_goal_future.add_done_callback(self.nav_to_pose_response_callback)

    def nav_to_pose_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal to was rejected!')
            return

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_to_pose_result_callback)

    def nav_to_pose_result_callback(self, future):
        result = future.result().result.error_code
        if result == NavigateToPose.NONE:
            self.get_logger().info('Navigation to goal is done')
            self.action_status_nav_to_pose = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().warn('Navigation to goal is failed, trying again')
            self.action_status_nav_to_pose = GoalStatus.STATUS_ABORTED

    def action_feedback_callback(self, msg):
        self.get_logger().debug('Received action feedback message')
        self.action_feedback = msg.feedback
        return


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with JohnnyLabNavigator() as johnny_lab_navigator:
        try:
            executor.add_node(johnny_lab_navigator)
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            johnny_lab_navigator.get_logger().warn("Killing the navigator node...")
            executor.remove_node(johnny_lab_navigator)
            executor.shutdown()


if __name__ == '__main__':
    main()
