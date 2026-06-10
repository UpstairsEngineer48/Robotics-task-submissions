#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen



class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('WaypointNavigator')

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen'
        )

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            pass

        self.pen_off()

        self.x = 0.0
        self.y = 0.0

        self.i = 0

        self.waypoints = [
            (5.5, 9.5),
            (7.0, 5.5),
            (10.0, 5.5),
            (7.8, 3.0),
            (9.0, 0.5),
            (5.5, 2.5),
            (2.0, 0.5),
            (3.2, 3.0),
            (1.0, 5.5),
            (4.0, 5.5),
            (5.5, 9.5)
        ]

        self.timer = self.create_timer(
            0.1,
            self.move
        )

    def pen_off(self):

        req = SetPen.Request()

        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 3
        req.off = 1

        self.pen_client.call_async(req)

    def pen_on(self):

        req = SetPen.Request()

        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 3
        req.off = 0

        self.pen_client.call_async(req)

    def pose_callback(self, msg):

        self.x = msg.x
        self.y = msg.y

    def move(self):

        if self.i >= len(self.waypoints):

            msg = Twist()

            msg.linear.x = 0.0
            msg.linear.y = 0.0

            self.pub.publish(msg)

            self.get_logger().info(
                "Reached all waypoints"
            )

            self.timer.cancel()
            return

        newx, newy = self.waypoints[self.i]

        diffx = newx - self.x
        diffy = newy - self.y

        distance = (diffx**2 + diffy**2)**.5

        if distance < 0.001:

            self.get_logger().info(
                f"Reached waypoint {self.i + 1} {self.x, self.y}"
            )

            self.i += 1

            if self.i == 1:
                self.pen_on()

            return

        msg = Twist()

        

        msg.linear.x = diffx 
        msg.linear.y =  diffy 
        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = WaypointNavigator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()