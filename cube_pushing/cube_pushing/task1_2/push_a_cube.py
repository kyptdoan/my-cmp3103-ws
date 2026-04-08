#!/usr/bin/env python

"""
A node to perceive ca coloured cube touching the red patch and push it away.
This is executed after the robot has reached the desired pose saved by 'estimate_pose_node'.
The node:
_ activates on a start signal.
_ makes the robot rotate to find touching cubes by detecting intersection between the red patch mask and cube mask.
_ if there are touching cubes:
    _ blocks the centroid of the first found cube contour in the image frame as a target.
    _ keep it centred in the frame using a PID controller.
    _ goes towards it until its appearance meets an area threshold in image frame.
    _ pushes it ahead until there is no red pixels left in the image frame (keep pushing for 2 more seconds to guarantee the cube is totally out of the patch).
    _ publishes the success signal to notify the state machine.
    _ goes idle.
_ if there is no touching cubes found after 50s rotating:
    _ publishes the failure signal to notify the state machine.
    _ goes idle.

Implementation referenced and adapted from: https://github.com/LCAS/teaching/blob/2425-devel/src/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_chaser.py
"""

# import libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# HSV colour ranges (brown for level 1, blue for level 2)
COLOUR_RANGES = {
    'brown': ((5,  100, 50), (25,  255, 200)),
    'blue':  ((100, 150, 50), (130, 255, 200)),
}


class PushACubeNode(Node):
    def __init__(self):
        super().__init__('push_a_cube_node')
 
        self.br = CvBridge()
 
        # specify colour parameter
        self.declare_parameter('brown', True)
        use_brown = self.get_parameter('brown').get_parameter_value().bool_value

        if use_brown:
            key = 'brown' # level 1
        else:
            key = 'blue' # level 2

        self.cube_lower, self.cube_upper = COLOUR_RANGES[key]
        self.get_logger().info(f'Targeting {key} cubes.')
    
        # Publish velocity command to control the robot motion
        self.pub_cmd_vel = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            1
        )
 
        # Publish as Empty to state machine when a cube is successfully pushed away
        self.pub_done = self.create_publisher(
            Empty, 
            '/push_a_cube/done', 
            1
        )
 
        # Publish as Empty to state machine when no touching cube found after 10s of rotating
        self.pub_failure = self.create_publisher(
            Empty, 
            '/push_a_cube/failure', 
            1
        )

        # Subscribe to start signal from state machine
        self.create_subscription(
            Empty, 
            '/push_a_cube/start', 
            self.start_callback, 
            1
        )

        # Subscribe to the robot's color camera stream
        self.create_subscription(
            Image, 
            '/limo/depth_camera_link/image_raw', 
            self.camera_callback, 
            1
        )
 
        # PID state (heuristically chosen through experiments)
        self.last_error_x = 0.0
        self.integral_error_x = 0.0
        self.Kp = 0.7
        self.Ki = 0.01
        self.Kd = 0.1
 
        # node state
        self.active = False # guard flag (only perform when activated by a state machine)
        self.search_start_time = None # time epoch when the robot starts rotating to search
        self.pushing = False # pushing mode
        self.target_point = None # the cube centroid that the robot is targeting
        self.last_cx = None # last known cx of the target centroid
        self.red_gone_time = None # duration since there is no red pixel left in the image frame
    
    def start_callback(self, msg):
        self.get_logger().info('Start signal received — beginning push sequence.')
        self.active = True # activate the node right after receiving the start signal
        self.pushing = False
        self.last_error_x = 0.0
        self.integral_error_x = 0.0
        self.last_cx = None
        self.target_point = None
        self.search_start_time = None

    def camera_callback(self, data):

        # only start when activated
        if not self.active:
            return

        # convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # get size of the image frame
        h, w = current_frame.shape[:2]

        # convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    
        # create masks for red patch and the target coloured cubes
        red_mask  = cv2.inRange(current_frame_hsv, (0, 100, 50), (10, 255, 255))
        cube_mask = cv2.inRange(current_frame_hsv, self.cube_lower, self.cube_upper)

        # twist message to publish
        tw = Twist()

        # only perform searching when it is not targeting or pushing any cubes
        if self.pushing:
            if cv2.countNonZero(red_mask) == 0:
                # stop condition for pushing: red patch gone
                if self.red_gone_time is None:
                    self.red_gone_time = time.time()
                    self.get_logger().info('Red patch gone — continuing for 2 more seconds.')
                elif time.time() - self.red_gone_time > 2.0:
                    self.get_logger().info('Push complete.')
                    self.active = False # deactivate the node after completing a single push
                    self.pushing = False
                    self.last_cx = None
                    self.red_gone_time = None
                    self.pub_cmd_vel.publish(Twist())
                    self.pub_done.publish(Empty()) # publish done signal for state machine
                    self.active = False # goes idle

                    # Visualisation
                    current_frame_small = cv2.resize(current_frame, (0,0), fx=0.4, fy=0.4) # reduce image size
                    cv2.imshow("Push A Cube", current_frame_small)
                    cv2.waitKey(1)

                    return
                else:
                    # keep pushing for 2 more second
                    tw.linear.x = 0.2
                    tw.angular.z = 0.0
                    self.pub_cmd_vel.publish(tw)
                    
                    # Visualisation
                    current_frame_small = cv2.resize(current_frame, (0,0), fx=0.4, fy=0.4) # reduce image size
                    cv2.imshow("Push A Cube", current_frame_small)
                    cv2.waitKey(1)

                return
            else:
                # keep pushing when cube is temporarily invisible once pushing is started
                tw.linear.x = 0.2
                tw.angular.z = 0.0
                self.pub_cmd_vel.publish(tw)
                
                # Visualisation
                current_frame_small = cv2.resize(current_frame, (0,0), fx=0.4, fy=0.4) # reduce image size
                cv2.imshow("Push A Cube", current_frame_small)
                cv2.waitKey(1)

                return
        else:
            # when it is not pushing
            # identify touching cubes
            cube_contours, cube_hierarchy = cv2.findContours(cube_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            touching_contours = []
            for cube in cube_contours:

                # discard noise contours (too small to be a cube)
                if cv2.contourArea(cube) < 100:
                    continue
                
                # create a blank mask same size as the image frame
                m = np.zeros((h, w), dtype=np.uint8)

                # fill the cube's contour on the mask
                cv2.drawContours(m, [cube], 0, 255, -1)

                # structure element for dilation
                structured_element = np.ones((5, 5), np.uint8)

                # dilation
                m_dilated = cv2.dilate(m, structured_element, iterations=1)

                # check if dilated cube mask overlaps the red patch
                overlap = cv2.bitwise_and(m_dilated, red_mask)

                # detect touching cube
                if cv2.countNonZero(overlap) > 0:
                    touching_contours.append(cube)

            # no touching contour - rotate to search
            if not touching_contours:
                if self.search_start_time is None:
                    # initialise search starting timestamp
                    self.search_start_time = time.time()
                elif time.time() - self.search_start_time > 50.0:
                    # stop searching and report as no touching cube left after 50s of searching
                    self.get_logger().info('No target found after 50s - publishing failure')
                    self.pub_failure.publish(Empty())
                    self.search_start_time = None
                    self.active = False
                    return
                
                self.integral_error_x = 0.0
                self.last_error_x = 0.0
                tw.linear.x = 0.0
                tw.angular.z = 0.3
                self.pub_cmd_vel.publish(tw)
                self.get_logger().info('No touching cube found, searching...')
                
                # Visualisation
                current_frame_small = cv2.resize(current_frame, (0,0), fx=0.4, fy=0.4) # reduce image size
                cv2.imshow("Push A Cube", current_frame_small)
                cv2.waitKey(1)

                return
            else:
                # touching cube found - reset search timer
                self.search_start_time = None

                # lock onto the first found touching contour
                # detect it by tracking the centroid that is closest to it in each frame
                if self.target_point is None:
                    target_cube = touching_contours[0]
                    M0 = cv2.moments(target_cube)
                    if M0['m00'] != 0:
                        self.target_point = int(M0['m10'] / M0['m00'])
                    self.last_cx = self.target_point
                else:
                    target_cube = min(touching_contours, key=lambda c: abs(cv2.moments(c)['m10'] / (cv2.moments(c)['m00']) - self.target_point))

                # find the centre of the target cube's contours
                M = cv2.moments(target_cube)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                self.last_cx = cx

                image_centre_x = w / 2
                error_x = cx - image_centre_x
                centre_threshold = 20 # pixels

                if  abs(error_x) > centre_threshold:
                    # not centred — PID rotation
                    error_x_norm = error_x / (w / 2)
                    self.integral_error_x += error_x_norm
                    d_error_x = error_x_norm - self.last_error_x
                    self.last_error_x = error_x_norm

                    tw.linear.x = 0.0
                    tw.angular.z = -(self.Kp * error_x_norm +
                                    self.Ki * self.integral_error_x +
                                    self.Kd * d_error_x)
        
                    self.get_logger().info(f'Centering: error_x={error_x:.1f}px, angular.z={tw.angular.z:.3f}')
                else:
                    # centred - reset PID
                    self.integral_error_x = 0.0
                    self.last_error_x = 0.0

                    if cy > h * 0.75:
                        # start pushing if cy of the cube centroid is going high in the image frame (indicating the close distance)
                        self.pushing = True
                        self.get_logger().info(f'Pushing!')
                        tw.linear.x = 0.2
                        tw.angular.z = 0.0
                    else:
                        # approaching the cube if it is not close enough to push
                        tw.linear.x = 0.2
                        tw.angular.z = 0.0
                
                # visualise
                current_frame_contours = cv2.drawContours(current_frame, [target_cube], -1, (255, 0, 255), 2)
                cv2.circle(current_frame_contours, (round(cx), round(cy)), 5, (0, 255, 0), -1)
                current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
                cv2.imshow("Push A Cube", current_frame_contours_small)
                cv2.waitKey(1)

        self.pub_cmd_vel.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    cv2.startWindowThread()
    node = PushACubeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()


        


