#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion

from bot_control.ik import inverse_kinematics


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TraceSpiral(Node):

    def __init__(self):

        super().__init__("trace_spiral")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )


        self.timer = self.create_timer(
            0.01,
            self.control_loop
        )



        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx_world = 0.0
        self.vy_world = 0.0
        self.omega_world = 0.0

        self.have_odom = False

 
        # Spiral parameters
        # r = a * (theta)
        # theta = t
        self.a = 0.2
        self.t = 0.0
        self.dt = 0.01
   
        self.max_time = 30.0
        self.done = False

        self.kp_pos = 1.0
        self.kd_pos = 0.40

    

        self.max_wheel_speed = 8.0

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta=0.0

    def control_loop(self):

        if self.done:
            return


  
        if self.t >= self.max_time:

            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0]
            self.publisher.publish(stop_msg)

            self.get_logger().info("Spiral complete.")

            self.done = True
            self.timer.cancel()
            return

   
        omega = 0.25         
        theta = omega * self.t

        r = self.a * theta

        xd = r * math.cos(theta)
        yd = r * math.sin(theta)


        r_dot = self.a * omega

        xd_dot = (r_dot)

        yd_dot = (r * omega * math.cos(theta))

        ex = xd - self.x
        ey = yd - self.y


        vx_world = (xd_dot+self.kp_pos * ex)

        vy_world = (yd_dot +self.kp_pos *ey)
        

        # -----------------------------
        # Wheel commands
        # -----------------------------
        w1, w2, w3 = inverse_kinematics(
            vx_world,
            vy_world,
            0.0
        )

        # -----------------------------
        # Wheel saturation
        # -----------------------------
        w1 = max(min(w1, self.max_wheel_speed), -self.max_wheel_speed)
        w2 = max(min(w2, self.max_wheel_speed), -self.max_wheel_speed)
        w3 = max(min(w3, self.max_wheel_speed), -self.max_wheel_speed)

        msg = Float64MultiArray()
        msg.data = [w1, w2, w3]
        self.publisher.publish(msg)

        self.get_logger().info(
            f"t={self.t:.2f} "
            f"theta={theta:.2f} "
            f"r={r:.2f} "
            f"x={self.x:.2f} "
            f"y={self.y:.2f}"
        )

        self.t += self.dt

def main(args=None): 
    rclpy.init(args=args) 
    node = TraceSpiral() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    node.destroy_node() 
    rclpy.shutdown() 
if __name__ == "__main__": 
    main()
