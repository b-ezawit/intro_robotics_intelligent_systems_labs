#!/usr/bin/env python3
"""Node that turns away from obstacles when enabled, publishes robot status,
and provides services to enable/disable operation and set turn speed."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")

        # Step 3 & 5 parameters
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        # Step 5: Starts switched off
        self.enabled = False

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

        # Step 5 Services
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

        self.get_logger().info("turn_away node initialized (currently disabled)")

    def on_enable(self, request, response):
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "turn_away enabled"
        else:
            if self.enabled:
                self.enabled = False
                # Publish one zero velocity when switched off
                self.cmd_vel_pub.publish(Twist())
            else:
                self.enabled = False
            response.success = True
            response.message = "turn_away disabled and stopped"
        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter("max_turn_speed").value
        if not 0.0 < request.turn_speed <= max_turn_speed:
            response.success = False
            response.message = f"turn_speed must be above 0 and at most {max_turn_speed} rad/s"
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed:.2f} rad/s"
        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg):
        # Read parameters dynamically each time a message arrives (Step 3)
        forward_speed = self.get_parameter("forward_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        if not self.enabled:
            state = "stopped"
        elif not msg.too_close:
            state = "forward"
            twist = Twist()
            twist.linear.x = float(forward_speed)
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        else:
            state = "turning"
            twist = Twist()
            twist.linear.x = 0.0
            # Angle is positive when obstacle is to the left, so turn right (negative angular.z), and left otherwise
            twist.angular.z = -float(turn_speed) if msg.angle > 0.0 else float(turn_speed)
            self.cmd_vel_pub.publish(twist)

        # Step 4: Publish RobotStatus on /robot_status
        status = RobotStatus()
        status.state = state
        status.obstacle_distance = float(msg.distance)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
