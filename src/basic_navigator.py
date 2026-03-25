#!/usr/bin/env python3
# go_to_pose node: navigates to a saved pose using Nav2 BasicNavigator.
# Idle by default — activated by /go_to_pose/start from state machine.
# Publishes /go_to_pose/done or /go_to_pose/failed when navigation ends.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import os

POSE_FILE = os.path.join(os.path.dirname(__file__), 'saved_pose.json')


class GoToPoseNode(Node):
    def __init__(self):
        super().__init__('go_to_pose_node')

        self.navigator = BasicNavigator()

        self.create_subscription(Empty, '/go_to_pose/start', self.start_callback, 1)

        self.pub_done   = self.create_publisher(Empty, '/go_to_pose/done',   1)
        self.pub_failed = self.create_publisher(Empty, '/go_to_pose/failed', 1)

        self.active = False

    def start_callback(self, msg):
        if self.active:
            return

        self.get_logger().info('Start signal received — waiting for Nav2...')
        self.navigator.waitUntilNav2Active()

        with open(POSE_FILE, 'r') as f:
            pose = json.load(f)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose['x']
        goal_pose.pose.position.y = pose['y']
        goal_pose.pose.orientation.z = pose['z']
        goal_pose.pose.orientation.w = pose['w']

        self.get_logger().info(f'Sending goal: {pose}')
        self.active = True
        self.task = self.navigator.goToPose(goal_pose)
        self.create_timer(0.1, self.check_navigation)

    def check_navigation(self, ):
        if not self.navigator.isTaskComplete(task=self.task):
            feedback = self.navigator.getFeedback(task=self.task)
            if feedback:
                self.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f} m',
                    throttle_duration_sec=1.0
                )
            return

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
            self.pub_done.publish(Empty())
        else:
            self.get_logger().warn(f'Goal failed with status: {result}')
            self.pub_failed.publish(Empty())

        self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()