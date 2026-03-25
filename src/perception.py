#!/usr/bin/env python
# Perception node: searches for a brown cube touching the red patch.
# Once found, latches onto it and publishes the stable centroid every frame.
# The contact point is only used internally to confirm the cube is still touching.
# Stops searching entirely until control node publishes on /perception/reset.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.br = CvBridge()

        self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.camera_callback,
            1
        )

        # control node sends Empty on this topic when task is done
        self.create_subscription(
            Empty,
            '/perception/reset',
            self.reset_callback,
            1
        )

        self.pub_target = self.create_publisher(PointStamped, '/target/contact_point', 1)

        # latched target state
        self.target_acquired = False
        self.target_cx = None  # centroid x of latched target (pixels)
        self.target_cy = None  # centroid y of latched target (pixels)

    def reset_callback(self, msg):
        self.get_logger().info('Reset received — searching for new target.')
        self.target_acquired = False
        self.target_cx = None
        self.target_cy = None

    def camera_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        h, w = current_frame.shape[:2]

        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(current_frame_hsv,
                               (0, 150, 50),
                               (10, 255, 255))

        brown_mask = cv2.inRange(current_frame_hsv,
                                 (5, 100, 50),
                                 (25, 255, 200))

        if self.target_acquired:
            # --- tracking mode: find the contour closest to last known centroid
            brown_contours, _ = cv2.findContours(brown_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            best_cube = None
            best_dist = float('inf')
            for cube in brown_contours:
                if cv2.contourArea(cube) < 100:
                    continue
                M = cv2.moments(cube)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                dist = np.hypot(cx - self.target_cx, cy - self.target_cy)
                if dist < best_dist:
                    best_dist = dist
                    best_cube = cube

            if best_cube is None:
                return

            # update tracked centroid
            M = cv2.moments(best_cube)
            self.target_cx = M['m10'] / M['m00']
            self.target_cy = M['m01'] / M['m00']
            target_cube = best_cube

        else:
            # --- search mode: look for first touching contour and latch
            if cv2.countNonZero(red_mask) == 0:
                return

            brown_contours, _ = cv2.findContours(brown_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            touching_contours = []
            for cube in brown_contours:
                if cv2.contourArea(cube) < 100:
                    continue

                cube_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.drawContours(cube_mask, [cube], 0, 255, -1)

                kernel = np.ones((5, 5), np.uint8)
                cube_mask_dilated = cv2.dilate(cube_mask, kernel, iterations=1)

                overlap = cv2.bitwise_and(cube_mask_dilated, red_mask)
                if cv2.countNonZero(overlap) > 0:
                    touching_contours.append(cube)

            if not touching_contours:
                return

            # latch onto first found — no sorting
            target_cube = touching_contours[0]
            M = cv2.moments(target_cube)
            if M['m00'] == 0:
                return

            self.target_cx = M['m10'] / M['m00']
            self.target_cy = M['m01'] / M['m00']
            self.target_acquired = True
            self.get_logger().info('Target latched.')

        # publish the stable centroid
        area = cv2.contourArea(target_cube)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.point.x = float(self.target_cx) / w  # normalised centroid x [0,1]
        msg.point.y = float(self.target_cy) / h  # normalised centroid y [0,1]
        msg.point.z = float(area) / (w * h)       # normalised area
        self.pub_target.publish(msg)

        # visualise — show centroid only
        cv2.circle(current_frame, (int(self.target_cx), int(self.target_cy)), 7, (0, 255, 0), -1)
        cv2.drawContours(current_frame, [target_cube], -1, (255, 0, 255), 2)
        cv2.imshow("Perception", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    cv2.startWindowThread()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()