#!/usr/bin/env python
# Control node: receives normalised centroid from perception node,
# centres the robot on the target using a PID controller, then pushes.
# Once pushing, target loss is interpreted as "close enough" — keep pushing forward.
# Stops only when the watchdog fires AND we were already pushing (red patch gone).

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Empty
import time


TARGET_TIMEOUT = 0.5  # seconds without a message before target is considered lost


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pub_reset = self.create_publisher(Empty, '/perception/reset', 1)

        self.create_subscription(
            PointStamped,
            '/target/contact_point',
            self.target_callback,
            1
        )

        # PID state
        self.last_error_x = 0.0
        self.integral_error_x = 0.0
        self.Kp = 0.7
        self.Ki = 0.01
        self.Kd = 0.1

        # pushing state
        self.pushing = False
        self.push_area_threshold = 0.15
        self.last_target_time = None

        self.create_timer(0.1, self.control_loop)

    def target_callback(self, msg):
        self.last_target_time = time.time()

        centroid_x_norm = msg.point.x
        centroid_y_norm = msg.point.y
        area_norm = msg.point.z

        tw = Twist()

        if self.pushing:
            # already in push mode — just keep going straight, ignore centroid
            tw.linear.x = 0.2
            tw.angular.z = 0.0

        else:
            error_x = centroid_x_norm - 0.5
            centre_threshold_norm = 0.03

            if abs(error_x) > centre_threshold_norm:
                # not centred — rotate with PID
                self.integral_error_x += error_x
                d_error_x = error_x - self.last_error_x
                self.last_error_x = error_x

                tw.angular.z = -(self.Kp * error_x +
                                 self.Ki * self.integral_error_x +
                                 self.Kd * d_error_x)

                self.get_logger().info(
                    f'Centering: error_x={error_x:.3f}, angular.z={tw.angular.z:.3f}'
                )

            else:
                # centred — reset PID
                self.integral_error_x = 0.0
                self.last_error_x = 0.0

                if area_norm > self.push_area_threshold or centroid_y_norm > 0.75:
                    self.pushing = True
                    self.get_logger().info(
                        f'Pushing! area_norm={area_norm:.3f}, cy_norm={centroid_y_norm:.3f}'
                    )
                    tw.linear.x = 0.2
                    tw.angular.z = 0.0
                else:
                    # approach
                    target_area_norm = self.push_area_threshold * 0.7
                    error_area = target_area_norm - area_norm
                    tw.linear.x = float(error_area) / target_area_norm * 0.3
                    tw.linear.x = max(-0.1, min(0.3, tw.linear.x))

        self.pub_cmd_vel.publish(tw)

    def control_loop(self):
        """Watchdog: behaviour depends on whether we were pushing or not."""
        if self.last_target_time is None:
            return

        elapsed = time.time() - self.last_target_time
        if elapsed < TARGET_TIMEOUT:
            return

        if self.pushing:
            # target lost while pushing — keep going forward, red patch will disappear
            tw = Twist()
            tw.linear.x = 0.2
            tw.angular.z = 0.0
            self.pub_cmd_vel.publish(tw)
            self.get_logger().info('Target lost during push — continuing forward.')

            # extend the timeout so this only logs once per second
            self.last_target_time = time.time() - (TARGET_TIMEOUT - 1.0)

        else:
            # target lost before push started — stop and reset
            self.get_logger().info('Target lost before push — stopping and resetting perception.')
            self.pushing = False
            self.integral_error_x = 0.0
            self.last_error_x = 0.0
            self.last_target_time = None
            self.pub_cmd_vel.publish(Twist())
            self.pub_reset.publish(Empty())

    def stop_pushing(self):
        """Called when red patch is confirmed gone — stop robot and reset."""
        self.get_logger().info('Red patch gone — task done, resetting.')
        self.pushing = False
        self.integral_error_x = 0.0
        self.last_error_x = 0.0
        self.last_target_time = None
        self.pub_cmd_vel.publish(Twist())
        self.pub_reset.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()