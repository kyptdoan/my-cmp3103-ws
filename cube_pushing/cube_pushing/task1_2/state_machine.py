#!/usr/bin/env python
# state_machine node: owns the state transitions for task 1 and 2.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class StateMachineNode(Node):

    # main states of the model
    ESTIMATE = 'ESTIMATE'
    NAVIGATE = 'NAVIGATE'
    PUSH     = 'PUSH'
    DONE     = 'DONE'

    # max retries (for pose estimation and navigation)
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('state_machine_node')

        # publishers - send start signals to other nodes
        self.pub_estimate_pose_start = self.create_publisher(Empty, '/estimate_pose/start', 1)
        self.pub_go_to_pose_start    = self.create_publisher(Empty, '/go_to_pose/start', 1)
        self.pub_push_a_cube_start   = self.create_publisher(Empty, '/push_a_cube/start', 1)

        # subscribers - receive done/success/failure signals from other nodes
        self.create_subscription(Empty, '/estimate_pose/success', self.estimate_pose_success_callback, 1)
        self.create_subscription(Empty, '/estimate_pose/failure', self.estimate_pose_failure_callback, 1)
        self.create_subscription(Empty, '/go_to_pose/done', self.go_to_pose_done_callback, 1)
        self.create_subscription(Empty, '/go_to_pose/failed', self.go_to_pose_failed_callback, 1)
        self.create_subscription(Empty, '/push_a_cube/done', self.push_a_cube_done_callback, 1)
        self.create_subscription(Empty, '/push_a_cube/failure', self.push_a_cube_failure_callback, 1)

        # signal flags - set by callbacks, consumed by tick
        self.flag_estimate_pose_success = False
        self.flag_estimate_pose_failure = False
        self.flag_go_to_pose_done = False
        self.flag_go_to_pose_failed = False
        self.flag_push_a_cube_done = False
        self.flag_push_a_cube_failure = False

        # pose estimation retry counter
        self.est_retries = 0

        # navigation retry counter
        self.nav_retries = 0

        # start in ESTIMATE state
        self.state = self.ESTIMATE
        self.get_logger().info(f'State machine started — state: {self.state}')

        # tick at 10 Hz
        self.create_timer(0.1, self.tick)

    # --- signal callbacks ---
    # set signal received flags to True after receiving

    def estimate_pose_success_callback(self, msg):
        self.get_logger().info('Estimate success received.')
        self.flag_estimate_pose_success = True

    def estimate_pose_failure_callback(self, msg):
        self.get_logger().info('Estimate failure received.')
        self.flag_estimate_pose_failure = True

    def go_to_pose_done_callback(self, msg):
        self.get_logger().info('Go to pose success received.')
        self.flag_go_to_pose_done = True

    def go_to_pose_failed_callback(self, msg):
        self.get_logger().info('Go to pose failure received.')
        self.flag_go_to_pose_failed = True

    def push_a_cube_done_callback(self, msg):
        self.get_logger().info('Push done received.')
        self.flag_push_a_cube_done = True

    def push_a_cube_failure_callback(self, msg):
        self.get_logger().info('Push failure received.')
        self.flag_push_a_cube_failure = True

    # --- state machine ---
    # a 10 Hz tick

    def tick(self):
        if self.state == self.ESTIMATE:
            # send /estimate_pose/start topic to activate the EstimatePose node.
            self.pub_estimate_pose_start.publish(Empty())
            self.get_logger().info('Sent estimate_pose/start — waiting for pose estimate.')
            self.transition_to(self.ESTIMATE + '_WAIT') # transition to an intermediate state to wait for feedback signal

        elif self.state == self.ESTIMATE + '_WAIT':
            # intermediate state to wait for feedback signal from EstimatePose node
            if self.flag_estimate_pose_success:
                # reset flags and retry counter after a success is received
                self.flag_estimate_pose_success = False
                self.est_retries = 0
                # transitions to PUSH state
                self.transition_to(self.PUSH)
            elif self.flag_estimate_pose_failure:
                # update flags and retry counter if a failure is received
                self.flag_estimate_pose_failure = False
                self.est_retries += 1
                if self.est_retries >= self.MAX_RETRIES:
                    # run out of retries, end program with errors
                    self.get_logger().warn(f'Pose estimation failed {self.MAX_RETRIES} times — giving up.')
                    self.transition_to(self.DONE)
                else:
                    # still have retries left, retrying...
                    self.get_logger().warn(f'Pose estimation failed  - retrying ({self.est_retries}/{self.MAX_RETRIES}).')
                    self.transition_to(self.ESTIMATE)

        elif self.state == self.NAVIGATE:
            # send /go_to_pose/start topic to activate the GoToPose node.
            self.pub_go_to_pose_start.publish(Empty())
            self.get_logger().info('Sent go_to_pose/start — waiting for arrival.')
            self.transition_to(self.NAVIGATE + '_WAIT') # transition to an intermediate state

        elif self.state == self.NAVIGATE + '_WAIT':
            # intermediate state to wait for feedback signal from GoToPose node
            if self.flag_go_to_pose_done:
                # reset flags and retry counter when success
                self.flag_go_to_pose_done = False
                self.nav_retries = 0
                # transitions to PUSH state
                self.transition_to(self.PUSH)
            elif self.flag_go_to_pose_failed:
                # update flags and retry counter when failure
                self.flag_go_to_pose_failed = False
                self.nav_retries += 1
                if self.nav_retries >= self.MAX_RETRIES:
                    # run out of retries, end program with error
                    self.get_logger().warn(f'Navigation failed {self.MAX_RETRIES} times — giving up.')
                    self.transition_to(self.DONE)
                else:
                    # still have retries left, retrying
                    self.get_logger().warn(f'Navigation failed — retrying ({self.nav_retries}/{self.MAX_RETRIES}).')
                    self.transition_to(self.NAVIGATE)

        elif self.state == self.PUSH:
            # send /push_a_cube/start topic to activate the PushACube node.
            self.pub_push_a_cube_start.publish(Empty())
            self.get_logger().info('Sent push_a_cube/start — waiting for push to complete.')
            self.transition_to(self.PUSH + '_WAIT') # transition to an intermediate state to wait for feedback signal

        elif self.state == self.PUSH + '_WAIT':
            # intermediate state to wait for feedback signal from PushACube node
            if self.flag_push_a_cube_done:
                # reset flags and retry counter when success
                self.flag_push_a_cube_done = False
                # transitions to NAVIGATE state
                self.transition_to(self.NAVIGATE)
            elif self.flag_push_a_cube_failure:
                # update flags and retry counter when failure
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