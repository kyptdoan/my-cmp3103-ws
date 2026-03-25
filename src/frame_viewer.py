import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2

from std_msgs.msg import String
import numpy as np

class FrameViewer(Node):
    def __init__(self):
        super().__init__('frame_viewer')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", cv_image_small)

        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')
    cv2.startWindowThread()
    
    rclpy.init(args=args)

    frame_viewer = FrameViewer()

    rclpy.spin(frame_viewer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frame_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()