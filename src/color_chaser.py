#!/usr/bin/env python

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')

        # publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # subscribe to the camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # state tracking
        self.last_cx = None # last known centroid x position
        self.object_visible = False
        self.pushing = False
        self.push_area_threshold = 15000

        # PID controller
        self.last_error_x = 0.0
        self.integral_error_x = 0.0

        self.Kp = 0.7
        self.Ki = 0.01
        self.Kd = 0.1

        # cube searching
        self.cube_on_patch = False

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        red_mask = cv2.inRange(current_frame_hsv,
                                         (0, 150, 50), 
                                         (10, 255, 255)) # red
        
        brown_mask = cv2.inRange(current_frame_hsv,
                                        (10, 100, 100),
                                        (20, 200, 255)) # brown

        # Find red contour (sorted by area and keep only the biggest one)
        red_contours, red_hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)[:1]

        # Find brown contour
        brown_contours, brown_hierarchy = cv2.findContours(brown_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, red_contours, 0, (255, 255, 0), 20)
        # current_frame_contours = cv2.drawContours(current_frame, brown_contours, 0, (255, 0, 255), 20)    # filled

        self.tw=Twist() # twist message to publish
        
        if len(red_contours) > 0:
            # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            
            # keep only the largest red area
            patch_contour = max(red_contours, key=cv2.contourArea)
            M_patch = cv2.moments(patch_contour)
            if M_patch['m00'] > 0:

                # get patch centroid
                cx_patch = int(M_patch['m10'] / M_patch['m00'])
                cy_patch = int(M_patch['m01'] / M_patch['m00'])

                cv2.circle(current_frame, (round(cx_patch), round(cy_patch)), 5, (0, 255, 0), -1)

                centre_threshold = 20

                image_centre_x = data.width / 2
                error_x = cx_patch - image_centre_x

                if abs(error_x) > centre_threshold:
                    self.integral_error_x += error_x            # accumulate error over time
                    d_error_x = error_x - self.last_error_x     # rate of change
                    self.last_error_x = error_x

                    self.tw.angular.z = -(self.Kp * float(error_x) / (data.width / 2) +
                            self.Ki * float(self.integral_error_x) / (data.width / 2) +
                            self.Kd * float(d_error_x) / (data.width / 2))
                    
                    print(f"Centering patch, error: {error_x:.1f}px")
                    print(f"angular.z set to: {self.tw.angular.z}")
                else:
                    # reset PID state when patch is centred
                    self.integral_error_x = 0.0
                    self.last_error_x = 0.0

                    # debug
                    if len(brown_contours) > 0:
                        print(f"Brown cube found")
                    else:
                        print(f"No brown cube found")

                    # for each cube found
                    for cube in brown_contours:

                        # filter noise
                        if cv2.contourArea(cube) < 500:
                            continue

                        # find cube centroid
                        M_cube = cv2.moments(cube)
                        if M_cube['m00'] > 0:
                            cx_cube = int(M_cube['m10'] / M_cube['m00'])
                            cy_cube = int(M_cube['m01'] / M_cube['m00'])

                        # create filled masks for both
                        # patch_mask = np.zeros(current_frame.shape[:2], dtype=np.uint8)
                        # cube_mask = np.zeros(current_frame.shape[:2], dtype=np.uint8)

                        # cv2.drawContours(patch_mask, [patch_contour], 0, 255, -1)  # filled

                        # # check if they overlap
                        # overlap = cv2.bitwise_and(patch_mask, cube_mask)
                        # if cv2.countNonZero(overlap) > 0:
                        #     print("Cube is ON the patch")
                        #     self.cube_on_patch = True
                        #     break

                            # # update last known position
                            # self.last_cx = cx
                            # self.object_visible = True

                            # cv2.circle(current_frame, (round(cx), round(cy)), 5, (0, 255, 0), -1)

                            # image_centre_x = data.width / 2

                            # error_x = cx - image_centre_x
                            # self.integral_error_x += error_x            # accumulate error over time
                            # d_error_x = error_x - self.last_error_x     # rate of change
                            # self.last_error_x = error_x

                            # self.tw.angular.z = -(self.Kp * float(error_x) / (data.width / 2) +
                            #         self.Ki * float(self.integral_error_x) / (data.width / 2) +
                            #         self.Kd * float(d_error_x) / (data.width / 2))

                                # # check if close enough to start pushing
                                # if area > self.push_area_threshold or cy > data.height * 0.75:
                                #     self.pushing = True
                                #     print(f"Pushing mode activated! area={area}, cy={cy}")
                                
                                # if self.pushing:
                                #     # keep pushing forward at a steady speed
                                #     self.tw.linear.x = 0.2
                                #     self.tw.angular.z = 0.0 # go straight while pushing
                                # else:
                                #     targetle_area = 10000
                                #     error_area = target_area - area
                                #     self.tw.linear.x = float(error_area) / target_area * 0.3
                                #     self.tw.linear.x = max(-0.1, min(0.3, self.tw.linear.x))
                    if self.cube_on_patch:
                        # proceed to push logic
                        pass
                    else:
                        print("Patch is clear!")
        else:
            self.object_visible = False
            print("Object not visible")

            # if self.pushing:
            #     # object disappeared as too close - keep pushing forward
            #     print("Pushing mode - continuing forward")
            #     self.tw.linear.x = 0.2
            #     self.tw.angular.z = 0.0
            
            # elif self.last_cx is not None:
            #     # object disappeared but we weren't pushing yet - 
            #     # turn back toward last known position
            #     image_centre_x = data.width / 2
            #     error_x = self.last_cx - image_centre_x
            #     self.tw.angular.z = -float(error_x) / (data.width / 2) * 0.5
            #     print(f"Searching based on last known position: {self.last_cx}")

            # else:
            #     # never seen the object, just rotate to search
            #     self.tw.angular.z = 0.3
            #     self.tw.linear.x = 0.0
            
            self.tw.angular.z = 0.3
            self.tw.linear.x = 0.0

        self.pub_cmd_vel.publish(self.tw)

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_chaser.py.')
    cv2.startWindowThread()

    rclpy.init(args=args)

    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()