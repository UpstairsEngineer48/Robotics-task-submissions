#!/usr/bin/env python3

import math
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class Feedback(Node):

    def __init__(self):

        super().__init__("feedback")


        self.count=0
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            "/camera",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.pose_pub = self.create_publisher(
            Pose2D,
            "/bot_pose",
            10
        )

        self.dictionary = aruco.getPredefinedDictionary(
            aruco.DICT_4X4_100
        )

        self.parameters = aruco.DetectorParameters()

        self.detector = aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )


        self.robot_id = 1

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.corner_ids = [10, 12, 4, 8]
        self.corner_markers = {}

        self.get_logger().info("Feedback Node Started")



    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        corners, ids, rejected = self.detector.detectMarkers(frame)

        aruco.drawDetectedMarkers(frame, corners, ids)

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners[0]

            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]

            theta = math.atan2(dy, dx)

            if marker_id in self.corner_ids:
                self.corner_markers[marker_id] = (cx, cy)

            if marker_id == self.robot_id:
                theta = math.atan2(dy, dx)-math.radians(64.359)
                self.robot_x = cx
                self.robot_y = cy
                self.robot_theta = theta

                cv2.circle(
                    frame,
                    (int(cx), int(cy)),
                    5,
                    (0, 0, 255),
                    -1
                )

                length = 65

                end_x = int(cx + length * math.cos(theta))
                end_y = int(cy + length * math.sin(theta))

                cv2.arrowedLine(
                    frame,
                    (int(cx), int(cy)),
                    (end_x, end_y),
                    (255, 0, 0),
                    2
                )

            cv2.putText(
                frame,
                f"ID:{marker_id}",
                (int(cx), int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        pose = Pose2D()
        pose.x = self.robot_x
        pose.y = self.robot_y
        pose.theta = self.robot_theta

        self.pose_pub.publish(pose)
        self.get_logger().info(f"x:{pose.x:.1f} y:{pose.y:.1f}  theta:{math.degrees(pose.theta):.2f}"
                )
        if len(self.corner_markers) == 4 and self.count==0:

            self.get_logger().info("Corner Coordinates:")
            self.count+=1
            for marker_id in sorted(self.corner_markers.keys()):

                x, y = self.corner_markers[marker_id]

                self.get_logger().info(f"ID {marker_id}: ({x:.1f}, {y:.1f})")


        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)



def main(args=None):

    rclpy.init(args=args)

    node = Feedback()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()