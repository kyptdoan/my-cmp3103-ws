#!/usr/bin/env python
# find_a_target node: searches for a brown cube touching the red patch.
# once found, publishes a single PointStamped and a success signal.
# then goes idle until /find_a_target/start is received again.
# Publishes a failure signal after a timeout with no target found.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

from rclpy.qos import QoSProfile, DurabilityPolicy


SEARCH_TTMEOUT = 5.0 # seconds before declaring failure

class FindATargetNode(Node):
    def __init__(self):
        super().__init__('find_a_target_node')

        self.br = CvBridge()

        # Subcribe to the robot's camera stream to get the current frame
        self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.camera_callback,
            1
        )

        # State Machine sends Empty on this topic to trigger this node
        self.create_subscription(
            Empty,
            '/find_a_target/start',
            self.start_callback,
            1
        )

        # publish PointStamped for target pushing if a target is found
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_target = self.create_publisher(
            PointStamped,
            '/target/target_point',
            latch_qos
        )

        # publish as Empty to state machine when a target is found
        self.pub_success = self.create_publisher(
            Empty,
            '/find_a_target/success',
            1
        )

        # publish as Empty to state machine when no target is found
        self.pub_failure = self.create_publisher(
            Empty,
            '/find_a_target/failure',
            1
        )

        # idle by default - only activate after /find_a_target/start received
        self.searching = False
        self.search_start_time = None

    def start_callback(self, msg):
        self.get_logger().info('Start signal received — searching for new target.')
        self.searching = True
        self.search_start_time = time.time()

    def camera_callback(self, data):
        
        if not self.searching:
            return
        
        # check timeouut - declare failure if exceeded
        if time.time() - self.search_start_time > SEARCH_TTMEOUT:
            self.get_logger().info('Search timeout - no target found.')
            self.searching = False
            self.pub_failure.publish(Empty())
            return
        
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        h, w = current_frame.shape[:2]

        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(current_frame_hsv,
                               (0, 150, 50),
                               (10, 255, 255))
        
        brown_mask = cv2.inRange(current_frame_hsv,
                                 (5, 100, 50),
                                 (25, 255, 200))
        
        blue_mask = cv2.inRange(current_frame_hsv,
                                (100, 150, 50),
                                (130, 255, 200))
        
        brown_contours, hierarchy = cv2.findContours(brown_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        touching_contours = []
        for cube in brown_contours:
            if cv2.contourArea(cube) < 100:
                continue

            cube_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(cube_mask, [cube], 0, 255, -1)

            se = np.ones((3, 3), np.uint8)
            cube_mask_dilated = cv2.dilate(cube_mask, se, iterations=1)

            overlap = cv2.bitwise_and(cube_mask_dilated, red_mask)
            if cv2.countNonZero(overlap) > 0:
                touching_contours.append(cube)

        if not touching_contours:
            return # keep searching until timeout
        
        # latch onto first found - no sorting
        target_cube = touching_contours[0]
        
        # find the target cube's centroid
        M = cv2.moments(target_cube)
        target_cx = M['m10'] / M['m00']
        target_cy = M['m01'] / M['m00']
        area = cv2.contourArea(target_cube)

        # publish the target point with a transient local QoS to ensure the message to be received by push_a_cube
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.point.x = float(target_cx) / w
        msg.point.y = float(target_cy) / h
        msg.point.z = float(area) / (w * h)
        self.pub_target.publish(msg)

        # publish the success signal
        self.pub_success.publish(Empty())

        # go idle until next start signal
        self.searching = False
        self.get_logger().info('Target found - publishing success and going idle.')

        # visualise
        cv2.circle(current_frame, (int(target_cx), int(target_cy)), 7, (0, 255, 0), -1)
        cv2.drawContours(current_frame, [target_cube], -1, (255, 0, 255), 2)
        cv2.imshow("Find A Target", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    cv2.startWindowThread()
    node = FindATargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
