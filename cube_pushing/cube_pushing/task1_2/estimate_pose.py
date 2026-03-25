#!/usr/bin/env python3
"""
A node to estimate and save the robot's current pose in the map frame built towards
the patch region only, using SLAM with slam-toolbox.
This is used only once at the beginning of level 1 and 2 to save the initial
pose of the robot after spawning (as this is heuristically determined as the
best position to observe the patch region).
The saved pose is used to navigate the robot back after each push to have a
good perception for the next push.
_ Activates on a start signal.
_ Performs a TF look up.
_ Save 2D pose to a runtime file.
Implementation reference: https://automaticaddison.com/how-to-create-a-tf-listener-using-ros-2-and-python/
"""

# import libraries
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Empty
import json
import math

class EstimatePoseNode(Node):
    def __init__(self):
        super().__init__('estimate_pose_node')

        # TF buffer and listener to query transform between frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribe to start signal from state machine
        self.create_subscription(
            Empty,
            '/estimate_pose/start',
            self.start_callback,
            1
        )

        # publish to notify state machine when goal is reached
        self.pub_success = self.create_publisher(
            Empty,
            '/estimate_pose/success',
            1
        )

        # publish to notify state machine when goal is failed
        self.pub_failure = self.create_publisher(
            Empty,
            '/estimate_pose/failure',
            1
        )

        # guard flag (only perform when activated by a state machine)
        self.active = False
    
    def start_callback(self, msg):
        """
        Activate the node and immediately attempt to save the pose
        """
        self.get_logger().info('Start estimating pose.')
        self.active = True

        self.save()

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

    def save(self):
        """
        Look up the robot's current pose in the map frame and write it to disk.
        Deactivates after a single successful or failed attempt.
        """

        if not self.active:
            return

        try:
            # Look up transform from base_link into the map frame (robot pose in map)
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Extract 2D pose: (x, y, yaw)
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y

            roll, pitch, yaw = self.euler_from_quaternion(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
                  
            self.current_yaw = yaw 

            pose = {
                'x': self.current_x,
                'y': self.current_y,
                'yaw': self.current_yaw,
            }
            
            # Write pose into a known file that will be used by other nodes
            with open('/tmp/saved_pose.json', 'w') as f:
                json.dump(pose, f)
            print(f"Pose saved: {pose}")

            # publish success signal to state machine
            self.pub_success.publish(Empty())
            self.active = False

        except Exception as e:
            # TF look up failed, notify the state machine
            print(f"TF error: {e}")
            self.pub_failure.publish(Empty())
            self.active = False

def main(args=None):
    rclpy.init(args=args)
    node = EstimatePoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()