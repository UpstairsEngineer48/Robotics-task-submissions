#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

from bot_control.ik import inverse_kinematics


class GoToGoal(Node):

    def __init__(self):

        super().__init__("go_to_goal")

        self.create_subscription(
            Pose2D,
            "/bot_pose",
            self.pose_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.goal_points = [
            (938.8, 61.8),
            (939.2, 941.5),
            (59.8, 62.8),
            (60.2, 942.0),
        ]

        self.current_goal = 0

        self.kp = 0.002

        self.goal_threshold = 40

        self.max_velocity = 0.4

        self.k_theta = 1.5

        self.max_omega = 1.0

        self.theta_threshold = math.radians(5.0)

        self.CAMERA_OFFSET = math.radians(0.0)

        self.get_logger().info(
            "Go To Goal Node Started."
        )

    def pose_callback(self, msg):

        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

        if self.current_goal >= len(self.goal_points):

            wheel_msg = Float64MultiArray()
            wheel_msg.data = [0.0, 0.0, 0.0]

            self.cmd_pub.publish(wheel_msg)
            return

        goal_x, goal_y = self.goal_points[self.current_goal]

        ex = goal_x - self.robot_x
        ey = goal_y - self.robot_y

        distance = math.sqrt(ex * ex + ey * ey)

        self.get_logger().info(
            f"[goal {self.current_goal}] robot=({self.robot_x:.1f},{self.robot_y:.1f}) "
            f"dist={distance:.1f}"
        )

        if distance < self.goal_threshold:

            self.get_logger().info(
                f"Reached Goal {self.current_goal}"
            )

            self.current_goal += 1
            return

        vx_world = self.kp * ex
        vy_world = self.kp * ey

        desired_theta = math.atan2(ey, ex) + self.CAMERA_OFFSET

        theta_error = desired_theta - self.robot_theta

        theta_error = math.atan2(
            math.sin(theta_error),
            math.cos(theta_error)
        )

        omega = self.k_theta * theta_error

        if omega > self.max_omega:
            omega = self.max_omega
        elif omega < -self.max_omega:
            omega = -self.max_omega

        c = math.cos(self.robot_theta)
        s = math.sin(self.robot_theta)

        vx_robot = c * vx_world + s * vy_world
        vy_robot = -s * vx_world + c * vy_world

        speed = math.sqrt(vx_robot**2 + vy_robot**2)

        if speed > self.max_velocity:

            scale = self.max_velocity / speed

            vx_robot *= scale
            vy_robot *= scale

        w1, w2, w3 = inverse_kinematics(
            vx_robot,
            vy_robot,
            omega
        )

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [w1, w2, w3]

        self.cmd_pub.publish(wheel_msg)


def main(args=None):

    rclpy.init(args=args)

    node = GoToGoal()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
