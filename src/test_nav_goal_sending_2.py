import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(1.0, self.timer_callback)
        self.goal_sent = False

    def timer_callback(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            print("Nav2 not ready...")
            return
        if not self.goal_sent:
            self.send_saved_goal()
            self.goal_sent = True

    def send_saved_goal(self):
        with open('pre_pushing_pose.json', 'r') as f:
            pose = json.load(f)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose['x']
        goal_pose.pose.position.y = pose['y']
        goal_pose.pose.orientation.z = pose['z']
        goal_pose.pose.orientation.w = pose['w']

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        print(f"Sending to saved pose: {pose}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected!")
        else:
            print("Goal accepted!")
            goal_handle.get_result_async().add_done_callback(
                lambda f: print("Goal reached!"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoToPose())

if __name__ == '__main__':
    main()