import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

import math

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        # nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # subscribe to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        self.current_pose = None
        
        self.initial_yaw = None
        self.goal_sent = False
        self.second_goal_sent = False

        self.create_timer(1.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def get_current_pose_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform
        except Exception as e:
            print(f"TF error: {e}")
            return None
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        if self.current_pose is None:
            print("Waiting for odometry...")
            return
        
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            print("Nav2 not ready...")
            return

        if not self.goal_sent:
            self.send_goal(x_offset=1.5, y_offset=0.0, yaw_offset=math.radians(45))
            self.goal_sent = True
    
    
    def send_goal(self, x_offset, y_offset, yaw_offset=0.0):

        transform = self.get_current_pose_in_map()
        if transform is None:
            print("Transform is None, cannot send goal")
            return
        
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        # save initial yaw on first goal
        if self.initial_yaw is None:
            self.initial_yaw = yaw

        # apply yaw offset
        new_yaw = yaw + yaw_offset

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = transform.translation.x + x_offset * math.cos(new_yaw)
        goal_pose.pose.position.y = transform.translation.y + x_offset * math.sin(new_yaw)
        
        # convert new_yaw back to quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(new_yaw / 2)
        goal_pose.pose.orientation.w = math.cos(new_yaw / 2)

        goal_msg.pose = goal_pose

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        print(f"Goal sent: {goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}")
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected!")
        else:
            print("Goal accepted!")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        print("Goal reached! Sending rotation goal...")
        if not self.second_goal_sent:
            self.second_goal_sent = True
            self.send_rotation_goal()

    def send_rotation_goal(self):
        # rotate 180 degrees from initial pose
        target_yaw = self.initial_yaw + math.pi

        # convert yaw back to quaternion
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # stay at current position, just rotate
        transform = self.get_current_pose_in_map()
        goal_pose.pose.position.x = transform.translation.x
        goal_pose.pose.position.y = transform.translation.y

        # yaw to quaternion
        goal_pose.pose.orientation.z = math.sin(target_yaw / 2)
        goal_pose.pose.orientation.w = math.cos(target_yaw / 2)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        print(f"Rotation goal sent, target yaw: {math.degrees(target_yaw):.1f}deg")
    
def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()