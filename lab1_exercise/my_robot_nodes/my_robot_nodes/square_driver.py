#!/usr/bin/env python3
"""Action client node that drives diffdrive_bot in a square by commanding
drive_distance and rotate action servers sequentially."""

import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def send_drive_distance(self, distance):
        self.get_logger().info("Waiting for /drive_distance action server...")
        if not self.drive_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/drive_distance action server not available")
            return False

        goal = DriveDistance.Goal()
        goal.distance = float(distance)
        send_future = self.drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("DriveDistance goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def send_rotate(self, angle):
        self.get_logger().info("Waiting for /rotate action server...")
        if not self.rotate_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/rotate action server not available")
            return False

        goal = Rotate.Goal()
        goal.angle = float(angle)
        send_future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Rotate goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def drive_square(self):
        side_length = float(self.get_parameter("side_length").value)
        quarter_turn = math.pi / 2.0  # 90 degrees left turn (~1.5708 rad)

        self.get_logger().info(f"Starting square trajectory with side length {side_length:.2f} m")

        for side in range(1, 5):
            self.get_logger().info(f"--- Side {side}/4: Driving forward {side_length:.2f} m ---")
            success = self.send_drive_distance(side_length)
            if not success:
                self.get_logger().error(f"DriveDistance failed on side {side}. Stopping square trajectory.")
                return False

            self.get_logger().info(f"--- Side {side}/4: Rotating quarter turn left (1.57 rad) ---")
            success = self.send_rotate(quarter_turn)
            if not success:
                self.get_logger().error(f"Rotate failed on side {side}. Stopping square trajectory.")
                return False

        self.get_logger().info("Successfully completed square trajectory!")
        return True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.drive_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
