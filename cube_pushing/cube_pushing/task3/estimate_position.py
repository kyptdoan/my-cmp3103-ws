#!/usr/bin/env python3

"""
A node to estimate the 3D position of a coloured target in map frame using the RGB-D camera.
The node:
- makes the robot rotate to search for the target (brown cube and red patch).
- estimates the 3D position of the target using the depth image and camera intrinsics.
- transforms the position from camera frame to map frame using TF.
- saves the estimated position to a JSON file.
"""

# import libraries
import rclpy
from rclpy.node import Node
 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

from nav2_msgs.action import NavigateToPose
import tf2_geometry_msgs
from std_msgs.msg import Empty
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

import time
import json

COLOURS_RANGES = {
    'red1': ((0, 180, 100), (10, 255, 255)),
    'red2': ((170, 180, 100), (180, 255, 255)),
    'brown': ((5, 100, 50), (25, 255, 200)),
}

class EstimatePositionNode(Node):
    def __init__(self):
        super().__init__('estimate_position_node')

        self.br = CvBridge()

        # Specify colour parameters
        self.declare_parameter('brown', True)
        use_brown = self.get_parameter('brown').get_parameter_value().bool_value

        if use_brown:
            self.key = 'brown'
        else:
            self.key = 'red'  # combine red1 and red2 for full red range

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        self.sub_color = self.create_subscription(
            Image,
            'limo/depth_camera_link/image_raw',
            self.color_callback,
            1
        )

        self.sub_color = self.create_subscription(
            Image,
            'limo/depth_camera_link/depth/image_raw',
            self.depth_callback,
            1
        )

        # Publishes as Empty to state machine when the target position has been estimated and saved
        self.pub_done = self.create_publisher(
            Empty,
            '/estimate_position/success',
            1,
        )

        # Publishes as Empty to state machine when the target position estimation has failed
        self.pub_failed = self.create_publisher(
            Empty,
            '/estimate_position/failure',
            1,
        )

        # Subscribes to start signal for brown target from state machine
        self.sub_start_brown = self.create_subscription(
            Empty,
            '/estimate_position/start_brown',
            self.start_brown_callback,
            1,
        )

        # Subscribes to start signal for red target from state machine
        self.sub_start_brown = self.create_subscription(
            Empty,
            '/estimate_position/start_red',
            self.start_red_callback,
            1,
        )
 
        # latest frames
        self.depth_image = None

        # camera intrinsic (from ros2 topic echo /limo/depth_camera_link/camera_info)
        self.fx = 448.6252424876914
        self.fy = 448.6252424876914
        self.cx = 320.5
        self.cy = 240.5

        # PID controller
        self.last_error_x = 0.0
        self.integral_error_x = 0.0
        self.Kp = 0.7
        self.Ki = 0.01
        self.Kd = 0.1

        self.centre_threshold = 20

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # node states
        self.active = False
        self.search_start_time = None

    def start_brown_callback(self, msg):
        self.get_logger().info('Received start signal (brown).')
        self.key = 'brown'
        self.active = True
        self.search_start_time = None

    def start_red_callback(self, msg):
        self.get_logger().info('Received start signal (red).')
        self.key = 'red'
        self.active = True
        self.search_start_time = None

    def depth_callback(self, data):

        # only start when activated by start signal
        if not self.active:
            return

        self.depth_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def color_callback(self, data):
        # only start when activated by start signal
        if not self.active:
            return
        
        if self.search_start_time is None:
            self.search_start_time = time.time()
            self.get_logger().info('Starting search for target...')
        elif time.time() - self.search_start_time > 50.0:
            # stop searching and report as no target found after 50 seconds
            self.get_logger().warn('Target not found after 50 seconds - publishing failure')
            self.pub_failed.publish(Empty())
            self.search_start_time = None  # reset for next time
            self.active = False
            return

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Build mask for the target colour
        if self.key == 'red':
            mask1 = cv2.inRange(current_frame_hsv, COLOURS_RANGES['red1'][0], COLOURS_RANGES['red1'][1])
            mask2 = cv2.inRange(current_frame_hsv, COLOURS_RANGES['red2'][0], COLOURS_RANGES['red2'][1])
            target_mask = cv2.bitwise_or(mask1, mask2)
        else:
            target_mask = cv2.inRange(current_frame_hsv, COLOURS_RANGES['brown'][0], COLOURS_RANGES['brown'][1])

        # Find target contour (keep the biggest one if targeting red, the smallest one if targeting brown)
        target_contours, target_hierarchy = cv2.findContours(target_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        target_contours = sorted(target_contours, key=cv2.contourArea, reverse=(self.key == 'red'))[:1]

        # Twist message to publish
        tw = Twist()

        if len(target_contours) > 0:
            # debug
            self.get_logger().info(f'Contour area: {cv2.contourArea(target_contours[0]):.1f}')

            M = cv2.moments(target_contours[0])
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                image_centre_x = data.width / 2
                error_x = cx - image_centre_x

                if abs(error_x) > self.centre_threshold:
                    # PID angular correction
                    self.integral_error_x += error_x
                    d_error_x = error_x - self.last_error_x
                    self.last_error_x = error_x

                    tw.angular.z = -(
                        self.Kp * error_x / image_centre_x +
                        self.Ki * self.integral_error_x / image_centre_x +
                        self.Kd * d_error_x / image_centre_x
                    )

                    self.get_logger().info(f'Centering: error={error_x:.1f}px  angular.z={tw.angular.z:.3f}')
                else:
                    # centroid is centred, reset PID and estimate 3D position
                    self.integral_error_x = 0.0
                    self.last_error_x = 0.0
                    tw.angular.z = 0.0

                    self.estimate_3d_position(cx, cy)
                
                # draw centroid and contour
                cv2.drawContours(current_frame, target_contours, -1, (255, 0, 255), 3)
                cv2.circle(current_frame, (cx, cy), 5, (0, 255, 0), -1)
        else:
            # no target - rotate to search
            self.integral_error_x = 0.0
            self.last_error_x = 0.0
            tw.angular.z = 0.3
            self.get_logger().info('No target found, searching...')

        # Publish velocities
        self.pub_cmd_vel.publish(tw)

        cv2.imshow('estimate_position', current_frame)
        cv2.waitKey(1)

    def estimate_3d_position(self, cx, cy):

        if self.depth_image is None:
            self.get_logger().warn('No depth image yet')
            return
        
        h, w = self.depth_image.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            self.get_logger().warn('Centroid out of depth image bounds')
            return
        
        # sample a small patch around centroid for a more stable depth reading
        padding = 5
        y0 = max(0, cy - padding)
        y1 = min(h, cy + padding)
        x0 = max(0, cx - padding)
        x1 = min(w, cx + padding)
        patch = self.depth_image[y0:y1, x0:x1].astype(np.float32)

        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            self.get_logger().warn('No valid depth in centroid patch')
            return
        
        Z = float(np.median(valid))

        # back-project 2D into 3D using pinhole model
        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy

        # transform to map frame
        point_depth = PointStamped()
        point_depth.header.frame_id = 'depth_link'
        point_depth.header.stamp = rclpy.time.Time().to_msg()
        point_depth.point.x = X
        point_depth.point.y = Y
        point_depth.point.z = Z

        try:
            point_map = self.tf_buffer.transform(point_depth, 'map')
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            self.pub_failed.publish(Empty())
            self.active = False
            return

        self.get_logger().info(
            f'Cube in map frame: x={point_map.point.x:.8f} y={point_map.point.y:.8f} z={point_map.point.z:.8f}'
        )

        point_map = self.tf_buffer.transform(point_depth, 'map')

        goal = {
            'x': round(point_map.point.x, 8),
            'y': round(point_map.point.y, 8),
            'z': 0.0,
            # 'w': 1.0
        }

        # Write the estimated position to disk for use by other nodes
        try:
            with open(f'/tmp/{self.key}_pos.json', 'w') as f:
                json.dump(goal, f)
            self.get_logger().info(f'Pos saved: {goal}')
            self.pub_done.publish(Empty())
            self.active = False
        except Exception as e:
            self.get_logger().error(f'Failed to save position: {e}')
            self.pub_failed.publish(Empty())
            self.active = False

def main(args=None):
    print("Starting estimate_position.py")
    cv2.startWindowThread()
    rclpy.init(args=args)
    node = EstimatePositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()