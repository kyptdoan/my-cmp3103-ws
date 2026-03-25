#!/usr/bin/env python
# go_to_pose node: navigates to a saved pose using Nav2.
# Idle by default — activated by /go_to_pose/start from state machine.
# Publishes /go_to_pose/done when the goal is reached.

# Reference: https://api.nav2.org/actions/humble/navigatetopose.html

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import Empty
import json
import os
import math
# from ament_index_python.packages import get_package_share_directory

# Path to the pose file written by estimate_pose_node
POSE_FILE = '/tmp/saved_pose.json'

class GoToPoseNode(Node):
    def __init__(self):
        super().__init__('go_to_pose_node')
 
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )

        # subscribe to start signal from state machine
        self.create_subscription(
            Empty,
            '/go_to_pose/start',
            self.start_callback,
            1
        )

        # publish to notify state machine when goal is reached
        self.pub_done = self.create_publisher(
            Empty,
            '/go_to_pose/done',
            1
        )

        # publish to notify state machine when goal is failed
        self.pub_failed = self.create_publisher(
            Empty,
            '/go_to_pose/failed',
            1
        )

        # guard flag: only navigate when activated by the state machine
        self.active = False

    def start_callback(self, msg):
        """
        Activate the node and begin navigation.
        Blocks briefly to confirm the Nav2 action server is available.
        """

        self.get_logger().info('Start signal received — navigating to saved pose.')
        self.active = True

        # wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available.')
            self.active = False
            return
        
        self.send_goal()

    def send_goal(self):
        """
        Load the saved pose from disk and send it as a NavigationToPose goal in map frame.
        """

        if not self.active:
            return

        # load the (x, y, yaw) pose saved by estimate_pose_node
        with open(POSE_FILE, 'r') as f:
            pose = json.load(f)

        # Build a PoseStamped in map frame
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose['x']
        goal_pose.pose.position.y = pose['y']
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
        goal_pose.pose.orientation.w = math.cos(pose['yaw'] / 2.0)

        # Wrap in a NavigateToPose goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending goal: {pose}')

        # send the goal asynchronously
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Called when Nav2 accepts or rejects the goal.
        If accepted, registers a callback to handle the final result (success or failure).
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.active = False
            return

        self.get_logger().info('Goal accepted — navigating...')

        # Register callback for when navigation finishes
        goal_handle.get_result_async().add_done_callback(self.goal_feedback_callback)

    
    def goal_feedback_callback(self, future):
        """
        Called when navigation completed.
        """
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached — publishing done.')
            self.pub_done.publish(Empty())
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
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