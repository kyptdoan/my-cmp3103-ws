#!/usr/bin/env python

# go_to_pre_pushing_pose node: computes the pre-pushing pose from the 3D back-projected cube and patch's position,
# and navigate to that pose.
# Idle by default - activated by /go_to_pre_pushing_pose/start from state machine
# Publishes /go_to_pre_pushing_pose/success when the goal is reached.

# Reference: https://api.nav2.org/actions/humble/navigatetopose.html

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import Empty
import json
import math

# path to the positio
CUBE_FILE     = '/tmp/brown_pos.json'
PATCH_FILE    = '/tmp/red_pos.json'
STANDOFF      = 0.05 # behind-cube stand-off distance

class GoToPrePushingPoseNode(Node):
    def __init__(self):
        super().__init__('go_to_pre_pushing_pose_node')

        # Action client for Nav2's NavigateToPose action
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to start signal from state machine
        self.create_subscription(
            Empty, 
            'go_to_pre_pushing_pose/start', 
            self.start_callback, 
            1
        )

        # Publisher to signal success to state machine
        self.pub_success = self.create_publisher(
            Empty, 
            'go_to_pre_pushing_pose/success', 
            1
        )

        # Publisher to signal failure to state machine
        self.pub_failed = self.create_publisher(
            Empty, 
            'go_to_pre_pushing_pose/failure', 
            1
        )

        # node states
        self.active = False
    
    def start_callback(self, msg):
        """
        Activate the node and begin navigation to the pre-pushing pose.
        """
        self.get_logger().info('Received start signal, activating GoToPrePushingPose node...')
        self.active = True

        # check if Nav2 is ready
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.pub_failed.publish(Empty())
            self.active = False
            return
        
        self.send_goal()
    
    def compute_pre_push_pose(self):
        """
        Load cube and patch positions, compute the pre-pushing pose:
        5 cm behind the cube along the cube-to-patch vector.
        """
        with open(CUBE_FILE, 'r') as f:
            cube = json.load(f)
        with open(PATCH_FILE, 'r') as f:
            patch = json.load(f)
        
        # Vector from cube to patch (pushing direction)
        dx = patch['x'] - cube['x']
        dy = patch['y'] - cube['y']
        dist = math.hypot(dx, dy)

        # Normalise
        nx = dx / dist
        ny = dy / dist

        # build the pose up in the (x, y, yaw) format
        pre_push = {
            'x':   round(cube['x'] - nx * STANDOFF, 8),
            'y':   round(cube['y'] - ny * STANDOFF, 8),
            'yaw': round(math.atan2(ny, nx), 8),
        }

        # log info
        self.get_logger().info(f'cube:      {cube}')
        self.get_logger().info(f'patch:     {patch}')
        self.get_logger().info(f'nx={nx:.3f} ny={ny:.3f}')
        self.get_logger().info(f'pre_push:  {pre_push}')

        return pre_push
    
    def send_goal(self):
        """
        Send a NavigateToPose goal to Nav2 for the computed pre-pushing pose.
        """
        # stay idle until start signal is received
        if not self.active:
            self.get_logger().warn('Node not active, ignoring start signal.')
            return 
        
        try:
            pose = self.compute_pre_push_pose()
        except Exception as e:
            self.get_logger().error(f'Failed to compute pre-pushing pose: {e}')
            self.pub_failed.publish(Empty())
            self.active = False
            return
        
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

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending goal: {pose}')

        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Called when Nav2 responds to our goal request.
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
    node = GoToPrePushingPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()

