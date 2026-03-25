#!/usr/bin/env python
# state_machine node: owns the state transitions for task 1 and 2.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class StateMachineNode(Node):

    ESTIMATE = 'ESTIMATE'
    NAVIGATE = 'NAVIGATE'
    PUSH     = 'PUSH'
    DONE     = 'DONE'

    MAX_NAV_RETRIES = 3

    def __init__(self):
        super().__init__('state_machine_node')

        # publishers - send start signals to other nodes
        self.pub_estimate_pose_start = self.create_publisher(Empty, '/estimate_pose/start', 1)
        self.pub_go_to_pose_start    = self.create_publisher(Empty, '/go_to_pose/start', 1)
        self.pub_push_a_cube_start   = self.create_publisher(Empty, '/push_a_cube/start', 1)

        # subscribers - receive done/success/failure signals from other nodes
        self.create_subscription(Empty, '/estimate_pose/success', self.estimate_pose_success_callback, 1)
        self.create_subscription(Empty, '/estimate_pose/failure', self.estimate_pose_failure_callback, 1)
        self.create_subscription(Empty, '/go_to_pose/done',       self.go_to_pose_done_callback,       1)
        self.create_subscription(Empty, '/go_to_pose/failed',     self.go_to_pose_failed_callback,     1)
        self.create_subscription(Empty, '/push_a_cube/done',      self.push_a_cube_done_callback,      1)
        self.create_subscription(Empty, '/push_a_cube/failure',   self.push_a_cube_failure_callback,   1)

        # signal flags - set by callbacks, consumed by tick
        self.flag_estimate_pose_success = False
        self.flag_estimate_pose_failure = False
        self.flag_go_to_pose_done       = False
        self.flag_go_to_pose_failed     = False
        self.flag_push_a_cube_done      = False
        self.flag_push_a_cube_failure   = False

        self.nav_retries = 0

        # start in ESTIMATE state
        self.state = self.ESTIMATE
        self.get_logger().info(f'State machine started — state: {self.state}')

        # tick at 10 Hz
        self.create_timer(0.1, self.tick)

    # --- signal callbacks ---

    def estimate_pose_success_callback(self, msg):
        self.get_logger().info('Estimate success received.')
        self.flag_estimate_pose_success = True

    def estimate_pose_failure_callback(self, msg):
        self.flag_estimate_pose_failure = True

    def go_to_pose_done_callback(self, msg):
        self.get_logger().info('Go to pose done received.')
        self.flag_go_to_pose_done = True

    def go_to_pose_failed_callback(self, msg):
        self.flag_go_to_pose_failed = True

    def push_a_cube_done_callback(self, msg):
        self.get_logger().info('Push done received.')
        self.flag_push_a_cube_done = True

    def push_a_cube_failure_callback(self, msg):
        self.flag_push_a_cube_failure = True

    # --- state machine ---

    def tick(self):
        if self.state == self.ESTIMATE:
            self.pub_estimate_pose_start.publish(Empty())
            self.get_logger().info('Sent estimate_pose/start — waiting for pose estimate.')
            self.transition_to(self.ESTIMATE + '_WAIT')

        elif self.state == self.ESTIMATE + '_WAIT':
            if self.flag_estimate_pose_success:
                self.flag_estimate_pose_success = False
                self.transition_to(self.NAVIGATE)
            elif self.flag_estimate_pose_failure:
                self.flag_estimate_pose_failure = False
                self.get_logger().warn('Pose estimation failed — giving up.')
                self.transition_to(self.DONE)

        elif self.state == self.NAVIGATE:
            self.pub_go_to_pose_start.publish(Empty())
            self.get_logger().info('Sent go_to_pose/start — waiting for arrival.')
            self.transition_to(self.NAVIGATE + '_WAIT')

        elif self.state == self.NAVIGATE + '_WAIT':
            if self.flag_go_to_pose_done:
                self.flag_go_to_pose_done = False
                self.nav_retries = 0
                self.transition_to(self.PUSH)
            elif self.flag_go_to_pose_failed:
                self.flag_go_to_pose_failed = False
                self.nav_retries += 1
                if self.nav_retries >= self.MAX_NAV_RETRIES:
                    self.get_logger().warn(f'Navigation failed {self.MAX_NAV_RETRIES} times — giving up.')
                    self.transition_to(self.DONE)
                else:
                    self.get_logger().warn(f'Navigation failed — retrying ({self.nav_retries}/{self.MAX_NAV_RETRIES}).')
                    self.transition_to(self.NAVIGATE)

        elif self.state == self.PUSH:
            self.pub_push_a_cube_start.publish(Empty())
            self.get_logger().info('Sent push_a_cube/start — waiting for push to complete.')
            self.transition_to(self.PUSH + '_WAIT')

        elif self.state == self.PUSH + '_WAIT':
            if self.flag_push_a_cube_done:
                self.flag_push_a_cube_done = False
                self.transition_to(self.NAVIGATE)
            elif self.flag_push_a_cube_failure:
                self.flag_push_a_cube_failure = False
                self.get_logger().info('No cube found — all done.')
                self.transition_to(self.DONE)

        elif self.state == self.DONE:
            self.get_logger().info('All done — shutting down.')
            rclpy.shutdown()

    def transition_to(self, new_state):
        self.get_logger().info(f'Transition: {self.state} → {new_state}')
        self.state = new_state

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()