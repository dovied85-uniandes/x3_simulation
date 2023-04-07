#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from math import pi, sin, cos, atan2


class X3Localizer(Node):
    def __init__(self):
        super().__init__("X3_localizer")
        self.pose_subscriber = self.create_subscription(PoseArray, "world/quadcopter_teleop/pose/info", self.callback_pose, 10)
        self.pose_publisher = self.create_publisher(Pose, "X3/pose", 10)
        self.pose = None
        self.create_timer(0.1, self.publish_pose)
        self.get_logger().info("X3 Localizer Node has been started.")

    def callback_pose(self, msg):
        # get drone pose
        self.pose = msg.poses[0]     
    
    def publish_pose(self):
        if self.pose is None:
            return
        self.pose_publisher.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)
    node = X3Localizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
