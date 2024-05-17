import math

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, LifecycleState, Publisher, State, TransitionCallbackReturn
from typing_extensions import Self, Any, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from action_msgs.msg import GoalStatus

from rclpy.qos import qos_profile_sensor_data

from rclpy.executors import MultiThreadedExecutor


class JohnnyLabNavigator(Node):
    def __init__(self) -> None:
        super().__init__('johnny_lab_navigator')

        self.publisher_initial_pose = None
        self.undock_action_client = None
        self.dock_action_client = None
        self.subscriber_dock_status = None

        self.action_status_undock = None
        self.action_status_dock = None
        self.is_docked = None
        self.initial_pose = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested. This callback is being called when
        the lifecycle node enters the configuring state.
        :return: The state machine either invokes a transition to the inactive state or stays
        in "unconfigured" depending on the return value.
        """

        self.get_logger().info('Configuring navigator')

        callback_group = MutuallyExclusiveCallbackGroup()

        self.initial_pose = self.get_pose_stamped(
            [0.0, 0.0],
            171.8)

        self.publisher_initial_pose = self.create_lifecycle_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10,
            callback_group=callback_group
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
            callback_group=callback_group
        )

        self.dock_action_client = ActionClient(
            self,
            Dock,
            'dock',
            callback_group=callback_group
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
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
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
        goal_msg = Dock.Goal()
        self.dock_action_client.wait_for_server()
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
