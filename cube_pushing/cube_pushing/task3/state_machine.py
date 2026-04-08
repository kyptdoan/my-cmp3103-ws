#!/usr/bin/env python
# state_machine node: owns the state transitions for task 1 and 2.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class StateMachineNode(Node):

    ESTIMATE_BROWN = 'ESTIMATE_BROWN'
    ESTIMATE_RED = 'ESTIMATE_RED'
    NAVIGATE = 'NAVIGATE'
    DONE = 'DONE'
    FAILED = 'FAILED'

    MAX_NAV_RETRIES = 3

    def __init__(self):
        super().__init__('state_machine_node')

        # publishers - send start signals to other nodes
        self.pub_estimate_brown = self.create_publisher(Empty, '/estimate_position/start_brown', 1)
        self.pub_estimate_red = self.create_publisher(Empty, '/estimate_position/start_red', 1)
        self.pub_go_to_pre_pushing_pose = self.create_publisher(Empty, '/go_to_pre_pushing_pose/start', 1)

        # subscribers - receive success/failure signals from other nodes
        self.create_subscription(Empty, '/estimate_position/success', self.estimate_position_success_callback, 1)
        self.create_subscription(Empty, '/estimate_position/failure', self.estimate_position_failure_callback, 1)
        self.create_subscription(Empty, '/go_to_pre_pushing_pose/success', self.go_to_pre_pushing_pose_success_callback, 1)
        self.create_subscription(Empty, '/go_to_pre_pushing_pose/failure', self.go_to_pre_pushing_pose_failure_callback, 1)

        # signal flags - set by callbacks, consumed by tick
        self.flag_estimate_position_success = False
        self.flag_estimate_position_failure = False
        self.flag_go_to_pre_pushing_pose_success = False
        self.flag_go_to_pre_pushing_pose_failure = False

        self.nav_retries = 0

        # start in ESTIMATE_BROWN state
        self.state = self.ESTIMATE_BROWN
        
        # # guard flags to prevent re-publishing start signals while waiting
        # self.waiting_for_estimate = False
        # self.waiting_for_navigate = False

        # Tick at 10 Hz
        self.create_timer(0.1, self.tick)

        self.get_logger().info('State machine started.')

    
    # --- signal callbacks ---

    def estimate_position_success_callback(self, msg):
        self.get_logger().info('Estimate success received.')
        self.flag_estimate_position_success = True

    def estimate_position_failure_callback(self, msg):
        self.flag_estimate_position_failure = True

    def go_to_pre_pushing_pose_success_callback(self, msg):
        self.get_logger().info('Go to pose done received.')
        self.flag_go_to_pre_pushing_pose_success = True

    def go_to_pre_pushing_pose_failure_callback(self, msg):
        self.flag_go_to_pre_pushing_pose_failure = True

    # --- state machine ---
    def tick(self):
        if self.state == self.ESTIMATE_BROWN:
            self.pub_estimate_brown.publish(Empty())
            self.get_logger().info('Sent estimate_position/start_brown — waiting for brown position estimate.')
            self.transition_to(self.ESTIMATE_BROWN + '_WAIT')
        
        elif self.state == self.ESTIMATE_BROWN + '_WAIT':
            if self.flag_estimate_position_success:
                self.flag_estimate_position_success = False
                self.transition_to(self.ESTIMATE_RED)
            elif self.flag_estimate_position_failure:
                self.flag_estimate_position_failure = False
                self.get_logger().warn('Brown estimation failed — giving up.')
                self.transition_to(self.FAILED)

        elif self.state == self.ESTIMATE_RED:
            self.pub_estimate_red.publish(Empty())
            self.get_logger().info('Sent estimate_position/start_red — waiting for red position estimate.')
            self.transition_to(self.ESTIMATE_RED + '_WAIT')
        
        elif self.state == self.ESTIMATE_RED + '_WAIT':
            if self.flag_estimate_position_success:
                self.flag_estimate_position_success = False
                self.nav_retries = 0
                self.transition_to(self.NAVIGATE)
            elif self.flag_estimate_position_failure:
                self.flag_estimate_position_failure = False
                self.get_logger().warn('Red estimation failed — giving up.')
                self.transition_to(self.FAILED)

        elif self.state == self.NAVIGATE:
            self.pub_go_to_pre_pushing_pose.publish(Empty())
            self.get_logger().info('Sent push_a_cube/start — waiting for arrival at pre-pushing pose.')
            self.transition_to(self.NAVIGATE + '_WAIT')
        
        elif self.state == self.NAVIGATE + '_WAIT':
            if self.flag_go_to_pre_pushing_pose_success:
                self.flag_go_to_pre_pushing_pose_success = False
                self.nav_retries = 0
                self.transition_to(self.DONE)
            elif self.flag_go_to_pre_pushing_pose_failure:
                self.flag_go_to_pre_pushing_pose_failure = False
                self.nav_retries += 1
                if self.nav_retries >= self.MAX_NAV_RETRIES:
                    self.get_logger().warn(f'Navigation failed {self.MAX_NAV_RETRIES} times — giving up.')
                    self.transition_to(self.FAILED)
                else:
                    self.get_logger().warn(f'Navigation failed — retrying ({self.nav_retries}/{self.MAX_NAV_RETRIES}).')
                    self.transition_to(self.NAVIGATE)

        elif self.state == self.DONE:
            self.get_logger().info('All done — shutting down.', once=True)
            rclpy.shutdown()

        elif self.state == self.FAILED:
            self.get_logger().error('Task failed — shutting down.', once=True)
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