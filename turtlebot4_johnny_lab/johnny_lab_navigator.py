import math
import yaml
import json
import os
import time
import random
from datetime import datetime
from zipfile import ZipFile

import depthai as dai
import av
from fractions import Fraction

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, LifecycleState, Publisher, State, TransitionCallbackReturn
from rclpy.node import Subscription
from rclpy import Future

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from typing_extensions import Self, Any, Optional, List, Dict
from io import TextIOWrapper

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from rclpy.executors import MultiThreadedExecutor


class JohnnyLabNavigator(Node):
    """
    A class controlling Turtlebot in Johnny Lab
    """

    def __init__(self) -> None:
        super().__init__('johnny_lab_navigator')

        # Callback groups
        self.main_callback_group = MutuallyExclusiveCallbackGroup()
        self.free_callback_group = ReentrantCallbackGroup()

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigator_params_path',
                 rclpy.Parameter.Type.STRING,
                 ParameterDescriptor(description='Path to navigator config file with parameters')),
            ]
        )

        # Open navigator param file and load all coordinates
        navigator_params_path = self.get_parameter('navigator_params_path').value
        with open(navigator_params_path, 'r') as navigator_config_file:
            navigator_params_dict = yaml.load(navigator_config_file, Loader=yaml.SafeLoader)

        # Get initial pose and its reverse pose
        try:
            self.initial_pose = self.get_pose_stamped(
                navigator_params_dict['init']['position'],
                navigator_params_dict['init']['rotation'])

            self.initial_pose_back = self.get_pose_stamped(
                navigator_params_dict['init']['position'],
                0.0)
        except Exception as e:
            self.get_logger().error('Error: %s' % str(e))

        # Get goal poses for path and put it to list of dicts
        self.goal_poses = []
        for point in navigator_params_dict['points']:
            try:
                pose = self.get_pose_stamped(
                    navigator_params_dict['points'][point]['position'],
                    navigator_params_dict['points'][point]['rotation'])
                pose_dict = {
                    'label':    str(point),
                    'pose':     pose,
                }
                self.goal_poses.append(pose_dict)
            except Exception as e:
                self.get_logger().error('Error in getting goal poses: %s' % str(e))

        # Service for getting IPFS dir from pubsub
        self.get_pubsub_parameter_client = self.create_client(
            GetParameters,
            'johnny_lab_navigator/robonomics_ros2_pubsub/get_parameters',
            callback_group=self.main_callback_group
        )

        # Creating publisher for archive file name after all work is done
        self.publisher_archive_name = self.create_publisher(
            String,
            'johnny_lab_navigator/archive_name',
            10,
            callback_group=self.free_callback_group
        )

        # Video init
        self.video_fps = 10
        self.video_pipeline = dai.Pipeline()
        oakd_camera = self.video_pipeline.create(dai.node.ColorCamera)
        oakd_camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        oakd_camera.setIspScale(1, 3)
        oakd_camera.setVideoSize(352, 288)
        oakd_camera.setFps(self.video_fps)

        video_encoder = self.video_pipeline.create(dai.node.VideoEncoder)
        video_encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
        oakd_camera.video.link(video_encoder.input)

        xout = self.video_pipeline.create(dai.node.XLinkOut)
        xout.setStreamName('enc')
        video_encoder.bitstream.link(xout.input)

        # Variable init
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_status_undock: Optional[int] = None
        self.action_status_dock: Optional[int] = None
        self.action_status_nav_to_pose: Optional[int] = None
        self.is_docked: Optional[bool] = None
        self.action_feedback: Optional[NavigateToPose.Feedback()] = None
        self.goal_random_poses_words: Optional[List[Dict]] = None
        self.ipfs_dir_path: Optional[str] = None
        self.data_path: Optional[str] = None
        self.video_path: Optional[str] = None
        self.data_file: Optional[TextIOWrapper] = None
        self.archive_path: Optional[str] = None
        self.seeds_file_path: Optional[str] = None
        self.video_record_status: Optional[bool] = None

        # Creating publisher for robot initial pose
        self.publisher_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            qos_profile=qos_profile_system_default,
            callback_group=self.free_callback_group,
        )

        # Creating subscriber to get dock status of robot
        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_callback,
            qos_profile_sensor_data,
            callback_group=self.free_callback_group
        )

        # Creating action client for undocking
        self.undock_action_client = ActionClient(
            self,
            Undock,
            'undock',
            callback_group=self.free_callback_group
        )

        # Creating action client for docking
        self.dock_action_client = ActionClient(
            self,
            Dock,
            'dock',
            callback_group=self.free_callback_group
        )

        # Creating action for navigation to pose
        self.nav_to_pose_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.free_callback_group
        )

        self.get_logger().info('Navigator init is done')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested. This callback is being called when
        the lifecycle node enters the configuring state.
        :return: The state machine either invokes a transition to the inactive state or stays
        in "unconfigured" depending on the return value.
        """

        self.get_logger().info('Configuring navigator')

        # Make request to get pubsub parameter with IPFS path if it not exists
        if self.ipfs_dir_path is None:
            self.ipfs_dir_path = self.get_pubsub_params_request()
            self.seeds_file_path = os.path.join(self.ipfs_dir_path, 'johnny_lab_launch.json')
            self.video_path = os.path.join(self.ipfs_dir_path, 'johnny_lab_record.mp4')
            self.data_path = os.path.join(self.ipfs_dir_path, 'data.json')

        # Open file and get seed phrase
        with open(self.seeds_file_path, 'r') as seeds_file:
            seed_data = json.load(seeds_file)
            seed_phrase = seed_data['seed']

        os.remove(self.seeds_file_path)

        # Split seed phrase into 2 words in pairs
        words_list = list(seed_phrase.split(' '))
        two_words_list = []

        for i in range(0, len(words_list), 2):
            two_words = words_list[i] + ' ' + words_list[i + 1]
            two_words_list.append(two_words)

        # Add pairs of words to new list with goal poses dicts
        self.goal_random_poses_words = self.goal_poses.copy()
        for i in range(0, len(two_words_list)):
            self.goal_random_poses_words[i].update({'words': two_words_list[i]})

        # Randomize poses with words
        random.shuffle(self.goal_random_poses_words)

        # Prepare files for archive
        current_time = datetime.now()
        self.archive_path = os.path.join(self.ipfs_dir_path, 'johnny_lab_archive_'
                                         + current_time.strftime("%d-%m-%Y-%H-%M-%S") + '.zip')

        self.data_file = open(self.data_path, 'w')
        self.data_file.write('[\n')

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

        time.sleep(10)

        # Set the initial pose
        self.set_initial_pose(self.initial_pose)

        time.sleep(5)

        # Activate video
        self.video_record_status = True
        self.executor.create_task(self.video_routine)
        self.get_logger().info('Created video routine')

        time.sleep(15)

        try:
            point_num = 0
            for goal_pose in self.goal_random_poses_words:

                self.send_goal_nav_to_pose(goal_pose['pose'])
                while self.action_status_nav_to_pose != GoalStatus.STATUS_SUCCEEDED:
                    if self.action_status_nav_to_pose == GoalStatus.STATUS_ABORTED:
                        self.get_logger().warn('Navigation to goal is failed, trying again')
                        self.action_status_nav_to_pose = None
                        self.send_goal_nav_to_pose(goal_pose['pose'])
                    pass

                # Getting pose from transform
                try:
                    coord_transform = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time())

                    robot_position_x = float(coord_transform.transform.translation.x)
                    robot_position_y = float(coord_transform.transform.translation.y)

                except TransformException:
                    self.get_logger().warn('Could not make pose transform')
                    robot_position_x = "NaN"
                    robot_position_y = "NaN"

                # Prepare dict with data and write in to file
                data_json_dict = {
                    'point_num': point_num,
                    'two_words': goal_pose['words'],
                    'robot_position_x': robot_position_x,
                    'robot_position_y': robot_position_y,
                }
                json_data_string = json.dumps(data_json_dict, indent=4)
                self.data_file.write(json_data_string + ',\n')

                self.get_logger().info('Navigation to goal is done')
                self.action_status_nav_to_pose = None

                point_num += 1

                time.sleep(5)

            self.data_file.write(']')
        except Exception as e:
            self.get_logger().error('Error in active state: %s' % str(e))

        # Deactivate video
        self.video_record_status = False

        self.get_logger().info('Navigator is finished active work')

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Navigator in deactivate state')

        # Return to initial point
        self.send_goal_nav_to_pose(self.initial_pose_back)
        while self.action_status_nav_to_pose != GoalStatus.STATUS_SUCCEEDED:
            if self.action_status_nav_to_pose == GoalStatus.STATUS_ABORTED:
                self.action_status_nav_to_pose = None
                self.send_goal_nav_to_pose(self.initial_pose_back)
            pass

        # Dock the robot, until success
        self.send_goal_dock()
        while self.action_status_dock != GoalStatus.STATUS_SUCCEEDED:
            if self.action_status_dock == GoalStatus.STATUS_ABORTED:
                self.action_status_dock = None
                self.send_goal_dock()
            pass

        self.get_logger().info('Navigator is deactivated')

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested. on_cleanup callback is being called when
        the lifecycle node enters the "cleaning up" state.
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        """
        self.get_logger().warn('Cleaning up navigator state')

        # Close file
        self.data_file.close()

        # Create resulting archive
        with ZipFile(self.archive_path, 'w') as zip_file:
            zip_file.write(self.data_path, os.path.basename(self.data_path))
            zip_file.write(self.video_path, os.path.basename(self.video_path))

        # Garbage removal routine
        os.remove(self.data_path)
        os.remove(self.video_path)

        # Publish last message with archive name
        archive_name_msg = String()
        archive_name_msg.data = str(os.path.basename(self.archive_path))
        self.publisher_archive_name.publish(archive_name_msg)
        #self.publisher_archive_name.wait_for_all_acked()

        time.sleep(15)

        # Clear variables
        self.action_status_undock = None
        self.action_status_dock = None
        self.action_status_nav_to_pose = None
        self.is_docked = None
        self.action_feedback = None
        self.goal_random_poses_words = None
        self.archive_path = None
        self.data_file = None
        self.video_record_status = None

        self.get_logger().info('All done')

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

    def get_pose_stamped(self,
                         position: List[float],
                         rotation: float) -> PoseStamped:
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

    def set_initial_pose(self, initial_pose: PoseStamped) -> None:
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = initial_pose.pose
        msg.header.frame_id = initial_pose.header.frame_id
        msg.header.stamp = initial_pose.header.stamp
        self.get_logger().info('Publishing initial pose')

        self.publisher_initial_pose.publish(msg)

    def dock_status_callback(self, msg: DockStatus) -> None:
        self.is_docked = msg.is_docked

    def send_goal_undock(self) -> None:
        """
        A function for sending goal to undock action
        :return: None
        """

        while not self.undock_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Undock action server not available, waiting...")

        goal_msg = Undock.Goal()
        send_goal_future = self.undock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future: Future) -> None:
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Cannot undock, please check')
            return
        self.get_logger().info('Going to undock robot from station')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future: Future) -> None:
        result = future.result().result.is_docked
        if result is False:
            self.get_logger().info('Undocking is done')
            self.action_status_undock = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().warn('Undocking is failed, trying again')
            self.action_status_undock = GoalStatus.STATUS_ABORTED

    def send_goal_dock(self) -> None:
        """
        A function for sending goal to dock action
        :return: None
        """

        while not self.dock_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Dock action server not available, waiting...")

        goal_msg = Dock.Goal()
        send_goal_future = self.dock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.dock_response_callback)

    def dock_response_callback(self, future: Future) -> None:
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Cannot dock, please check')
            return
        self.get_logger().info('Going to dock robot to station')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future: Future) -> None:
        result = future.result().result.is_docked
        if result is True:
            self.get_logger().info('Docking is done')
            self.action_status_dock = GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().warn('Docking is failed, trying again')
            self.action_status_dock = GoalStatus.STATUS_ABORTED

    def send_goal_nav_to_pose(self,
                              pose: PoseStamped,
                              behavior_tree: str = '') -> None:

        while not self.nav_to_pose_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("NavigateToPose action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ', ' + str(pose.pose.position.y))
        send_goal_future = self.nav_to_pose_action_client.send_goal_async(goal_msg, self.action_feedback_callback)
        send_goal_future.add_done_callback(self.nav_to_pose_response_callback)

    def nav_to_pose_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            return

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_to_pose_result_callback)

    def nav_to_pose_result_callback(self, future: Future) -> None:
        self.action_status_nav_to_pose = future.result().status

    def action_feedback_callback(self, msg: NavigateToPose.Feedback()):
        self.get_logger().debug('Received action feedback message')
        self.action_feedback = msg.feedback
        return

    def video_routine(self):

        # Activate video
        oakd_device = dai.Device(self.video_pipeline)

        device_queue = oakd_device.getOutputQueue(name="enc", maxSize=30, blocking=True)

        output_container = av.open(self.video_path, 'w')
        stream = output_container.add_stream("hevc", rate=self.video_fps)
        stream.time_base = Fraction(1, 1000 * 1000)  # Microseconds

        start_time_video = time.time()

        while self.video_record_status is True:
            try:
                data = device_queue.get().getData()  # np.array
                packet = av.Packet(data)  # Create new packet with byte array

                # Set frame timestamp
                packet.pts = int((time.time() - start_time_video) * 1000 * 1000)

                output_container.mux_one(packet)  # Mux the Packet into container
            except Exception as e:
                self.get_logger().error('Error in video recording: %s' % e)

        output_container.close()
        oakd_device.close()

    def get_pubsub_params_request(self) -> str:

        while not self.get_pubsub_parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pubsub parameter service not available, waiting again...')
        # Make request to get pubsub parameters with IPFS path

        request_ipfs = GetParameters.Request()
        request_ipfs.names = ['ipfs_dir_path']
        future_ipfs = self.get_pubsub_parameter_client.call_async(request_ipfs)

        while future_ipfs.result() is None:
            pass

        return str(future_ipfs.result().values[0].string_value)


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=4)

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
