#!/usr/bin/env python3
"""
The script used to demonstrate SLAM and navigation to a goal on the real robot.
This is a node that:
_ save the robot's initial pose.
_ sends a goal forward & navigate the robot to that without a pre-built map using slam-toolbox and nav2.
_ navigate the robot back to its initial pose.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time


class NavAndReturnNode(Node):
    def __init__(self):
        super().__init__('nav_and_return_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose = None

        # wait for Nav2 action server
        # self.get_logger().info('Waiting for Nav2 action server...')
        # self.nav_client.wait_for_server()
        # self.get_logger().info('Nav2 ready.')

    def send_goal(self, pose: PoseStamped) -> bool:
        """Send a NavigateToPose goal and block until it completes. Returns True if succeeded."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # send goal and wait for acceptance
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted — navigating...')

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached.')
            return True
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            return False

    def nav_and_return(self):

        # save initial pose in map frame
        # save initial pose in map frame — retry until TF is ready
        for _ in range(10):
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.initial_pose = t.transform
                self.get_logger().info(
                    f'Initial pose saved: '
                    f'x={self.initial_pose.translation.x:.3f} '
                    f'y={self.initial_pose.translation.y:.3f}'
                )
                break
            except Exception as e:
                self.get_logger().warn(f'TF not ready yet, retrying... ({e})')
                rclpy.spin_once(self, timeout_sec=1.0)  # allow TF callbacks to process
        else:
            self.get_logger().error('Could not get TF after retries.')
            return

        # send forward goal (1m ahead in base_link frame)
        forward_pose = PoseStamped()
        forward_pose.header.frame_id = 'base_link'
        forward_pose.header.stamp = self.get_clock().now().to_msg()
        forward_pose.pose.position.x = 2.0
        forward_pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending forward goal (1.0m in base_link)...')
        if not self.send_goal(forward_pose):
            return

        # navigate back to initial pose in map frame
        return_pose = PoseStamped()
        return_pose.header.frame_id = 'map'
        return_pose.header.stamp = self.get_clock().now().to_msg()
        return_pose.pose.position.x = self.initial_pose.translation.x
        return_pose.pose.position.y = self.initial_pose.translation.y
        return_pose.pose.orientation.x = self.initial_pose.rotation.x
        return_pose.pose.orientation.y = self.initial_pose.rotation.y
        return_pose.pose.orientation.z = self.initial_pose.rotation.z
        return_pose.pose.orientation.w = self.initial_pose.rotation.w

        self.get_logger().info(
            f'Returning to initial pose: '
            f'x={return_pose.pose.position.x:.3f} '
            f'y={return_pose.pose.position.y:.3f}'
        )
        if not self.send_goal(return_pose):
            return

        self.get_logger().info('Returned to initial pose successfully. Shutting down.')


def main(args=None):
    rclpy.init(args=args)
    node = NavAndReturnNode()
    time.sleep(3.0)
    node.nav_and_return()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()