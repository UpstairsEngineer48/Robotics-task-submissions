#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class InfinityTracer(Node):

    def __init__(self):
        super().__init__('infinity_tracer')

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.move)

        self.count = 0

    def move(self):

        msg = Twist()
        msg.angular.z = 3.0
        msg.linear.x = 0.007*msg.angular.z*(self.count//0.1)**1

        self.pub.publish(msg)

        self.count += 1


def main(args=None):

    rclpy.init(args=args)

    node = InfinityTracer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
