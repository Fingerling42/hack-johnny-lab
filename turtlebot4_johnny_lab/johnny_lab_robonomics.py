import os
import json

from typing_extensions import Dict

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robonomics_ros2_interfaces.msg import RobonomicsROS2ReceivedLaunch
from rcl_interfaces.msg import ParameterDescriptor

from robonomics_ros2_robot_handler.basic_robonomics_handler import BasicRobonomicsHandler
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class JohnnyLabRobonomics(BasicRobonomicsHandler):

    def __init__(self) -> None:
        super().__init__()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('seed_file_name',
                 '',
                 ParameterDescriptor(description='Name of seed file')),
            ]
        )

        lifecycle_callback_group = MutuallyExclusiveCallbackGroup()
        # Subscription for navigator topic with archive file name
        self.subscriber_archive_name = self.create_subscription(
            String,
            '/johnny_lab_navigator/archive_name',
            self.subscriber_archive_name_callback,
            10,
        )
        self.subscriber_archive_name  # prevent unused variable warning

        self.change_navigator_state_client = self.create_client(
            ChangeState,
            '/johnny_lab_navigator/change_state',
            callback_group=lifecycle_callback_group,
        )

    def launch_file_subscriber_callback(self, msg: RobonomicsROS2ReceivedLaunch) -> None:
        super(JohnnyLabRobonomics, self).launch_file_subscriber_callback(msg)

        with open(os.path.join(self.ipfs_dir_path, self.param_file_name), 'r') as launch_file:
            try:
                data: Dict = json.load(launch_file)
                if 'seed' in data:

                    # Set IPFS path parameter
                    seed_file_name_param = Parameter(
                        'seed_file_name',
                        rclpy.Parameter.Type.STRING,
                        self.param_file_name)
                    self.set_parameters([seed_file_name_param])

                    self.change_navigator_state_request(
                        request_id=Transition.TRANSITION_CONFIGURE,
                        request_label='configure'
                    )
                    # self.change_navigator_state_request(
                    #     request_id=Transition.TRANSITION_CLEANUP,
                    #     request_label='cleanup'
                    # )
            except Exception as e:
                self.get_logger().error('Launch handling failed: %s' % str(e))

    def subscriber_archive_name_callback(self, msg: String) -> None:
        """
        Method for receiving archive file name msgs from navigator
        :param msg: msg with turtlesim/msg/String type
        :return: None
        """
        archive_file_name = str(msg.data)

        # Get new subscription users
        rws_users_list = self.get_rws_users_request()

        self.send_datalog_request(archive_file_name, encrypt_recipient_addresses=rws_users_list)

    def change_navigator_state_request(self,
                                       request_id,
                                       request_label
                                       ) -> bool:

        # Preparing a request
        request = ChangeState.Request()
        request.transition.id = request_id
        request.transition.label = request_label
        self.get_logger().info('Sending request to change state to: %s' % request_label)

        # Making a request and wait for its execution
        future = self.change_navigator_state_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        self.get_logger().info('After sending to change state to: %s' % request_label)

        return future.result().success


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with JohnnyLabRobonomics() as johnny_lab_robonomics:
        try:
            executor.add_node(johnny_lab_robonomics)
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            johnny_lab_robonomics.get_logger().warn("Killing the Johnny lab Robonomics node...")
            executor.remove_node(johnny_lab_robonomics)
            executor.shutdown()


if __name__ == '__main__':
    main()
