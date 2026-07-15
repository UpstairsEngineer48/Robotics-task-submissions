#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from bot_control.ik import inverse_kinematics

class TraceEllipse(Node):

    def __init__(self):
        super().__init__("trace_ellipse")

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
        #self.have_odom = False?????????????????????
       
        # Ellipse parameters
        
        self.a = 1.0
        self.b = 2.0
        self.w = 0.12

        self.t = 0.0
	self.dt = 0.01 
        self.kp_pos = 1.0
        self.kd_pos = 0.40

        # Go-to-start gains (PD)
  
        self.kp_start = 1.0
        self.kd_start = 0.15
     
        # Previous errors (PD only)
        self.prev_ex = 0.0
        self.prev_ey = 0.0

        # Ellipse start

        self.start_x = self.a
        self.start_y = 0.0
        self.goal_tolerance = 0.02
        self.phase = "goto_start"
        self.done = False

        self.max_wheel_speed = 8.0

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = 0.0
       
    def control_loop(self):

        if self.done:
            return

        if self.phase == "goto_start":

            ex = self.start_x - self.x
            ey = self.start_y - self.y
        
            d_ex = (ex - self.prev_ex) / self.dt
            d_ey = (ey - self.prev_ey) / self.dt

            self.prev_ex = ex
            self.prev_ey = ey


            vx_world = self.kp_start * ex + self.kd_start * d_ex
            vy_world = self.kp_start * ey + self.kd_start * d_ey

            
            w1, w2, w3 = inverse_kinematics(
                vx_world,
                vy_world,
                0
            )

            w1 = max(min(w1, self.max_wheel_speed), -self.max_wheel_speed)
            w2 = max(min(w2, self.max_wheel_speed), -self.max_wheel_speed)
            w3 = max(min(w3, self.max_wheel_speed), -self.max_wheel_speed)

            msg = Float64MultiArray()
            msg.data = [w1, w2, w3]
            self.publisher.publish(msg)

            self.get_logger().info(
                f"[goto_start] "
                f"x={self.x:.3f} "
                f"y={self.y:.3f} "
                f"theta={self.theta:.3f} "
                f"ex={ex:.3f} "
                f"ey={ey:.3f} "
                f"w1={w1:.3f} "
                f"w2={w2:.3f} "
                f"w3={w3:.3f}"
            )

            if math.hypot(ex, ey) <= self.goal_tolerance:

                self.get_logger().info("Reached ellipse start point, beginning ellipse trace.")
                self.phase = "ellipse"
                self.prev_ex = 0.0
                self.prev_ey = 0.0
                self.t = 0.0

        elif self.phase == "ellipse":

            if self.w * self.t >= 2.0 * math.pi:

                stop_msg = Float64MultiArray()
                stop_msg.data = [0.0, 0.0, 0.0]
                self.publisher.publish(stop_msg)

                self.get_logger().info("Ellipse complete, stopping.")

                self.done = True
                self.timer.cancel()
                return


            xd = self.a * math.cos(self.w * self.t)
            yd = self.b * math.sin(self.w * self.t)

            xd_dot = -self.a * self.w * math.sin(self.w * self.t)
            yd_dot =  self.b * self.w * math.cos(self.w * self.t)

            ex = xd - self.x
            ey = yd - self.y

            x_dot =self.vx_world
            y_dot =self.vy_world

            vx_world=(xd_dot+self.kp_pos*ex)

            vy_world=(yd_dot+self.kp_pos*ey)

            w1, w2, w3 = inverse_kinematics(
                vx_world,
                vy_world,
                0
            )

            w1 = max(min(w1, self.max_wheel_speed), -self.max_wheel_speed)
            w2 = max(min(w2, self.max_wheel_speed), -self.max_wheel_speed)
            w3 = max(min(w3, self.max_wheel_speed), -self.max_wheel_speed)

            msg = Float64MultiArray()
            msg.data = [w1, w2, w3]
            self.publisher.publish(msg)

            self.get_logger().info(
                f"t={self.t:.2f} "
                f"xd={xd:.3f} yd={yd:.3f} "
                f"x={self.x:.3f} y={self.y:.3f} "
                f"vx={vx_world:.3f} vy={vy_world:.3f} "
                f"w1={w1:.3f} w2={w2:.3f} w3={w3:.3f}"
            )

            self.t += self.dt

def main(args=None):

    rclpy.init(args=args)

    node = TraceEllipse()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0]
        node.publisher.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
