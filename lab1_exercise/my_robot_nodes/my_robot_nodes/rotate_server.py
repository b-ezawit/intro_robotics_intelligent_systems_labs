#!/usr/bin/env python3
"""Action server that rotates the robot on the spot for a specified angle,
measured with wheel odometry, and provides feedback during rotation."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """target - current, wrapped to [-pi, pi] so the angle change is correct."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self.yaw = None

        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)

        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info("rotate_server action server ready on /rotate")

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        # Reject a goal of zero or if odometry hasn't been received yet
        if goal.angle == 0.0 or self.yaw is None:
            self.get_logger().warn(f"Rejecting goal: angle={goal.angle}, yaw_ready={self.yaw is not None}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        target_angle = float(goal_handle.request.angle)
        turn_speed = abs(float(self.get_parameter("turn_speed").value))
        self.get_logger().info(f"Rotating by {target_angle:.3f} rad at {turn_speed:.2f} rad/s")

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        turned = 0.0
        last_yaw = self.yaw

        twist = Twist()
        # Positive angle turns left (positive angular.z), negative turns right (negative angular.z)
        twist.angular.z = turn_speed if target_angle > 0.0 else -turn_speed

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                result.turned = float(turned)
                self.get_logger().info(f"Rotation canceled after {turned:.3f} rad")
                return result

            current_yaw = self.yaw
            delta = angle_difference(current_yaw, last_yaw)
            turned += delta
            last_yaw = current_yaw

            # Check if turned through the goal angle
            if target_angle > 0.0 and turned >= target_angle:
                break
            elif target_angle < 0.0 and turned <= target_angle:
                break

            self.cmd_vel.publish(twist)
            feedback.turned = float(turned)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        self.cmd_vel.publish(Twist())
        goal_handle.succeed()
        result.turned = float(turned)
        self.get_logger().info(f"Rotation complete, turned {result.turned:.3f} rad")
        return result


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
