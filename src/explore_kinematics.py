import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.wheel_vel_left = self.create_subscription(Float32, '/wheel_vel_left', self.wheel_callback, 1)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.wheel_radius = 0.025
        self.robot_radius = 0.1

    def wheel_callback(self, msg):
        w_l = msg.data # left wheel velocity from topic
        w_r = 0.0 # right wheel assumed to be 0

        # use forward kinematics to get linear and angular velocity
        (v, a) = self.forward_kinematics(w_l, w_r)

        # build and publish the Twist message
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = a
        self.cmd_publisher.publish(twist)
        self.get_logger().info(f"Publised Twist: linear={v:.3f}, angular={a:.3f}")


    # computing the forward kinematics for a differential drive
    def forward_kinematics(self, w_l, w_r):
        c_l = self.wheel_radius * w_l
        c_r = self.wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_r - c_l) / (2 * self.robot_radius)
        return (v, a)


    # computing the inverse kinematics for a differential drive
    def inverse_kinematics(self, v, a):
        c_l = v - (self.robot_radius * a)
        c_r = v + (self.robot_radius * a)
        w_l = c_l / self.wheel_radius
        w_r = c_r / self.wheel_radius
        return (w_l, w_r)


    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
    
    def inverse_kinematics_from_twist(self, t):
        return self.inverse_kinematics(t.linear.x, t.angular.z)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()