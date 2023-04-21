#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Vector3
from ros_gz_interfaces.srv import ManageMarker
from drone_interfaces.srv import GetTrajectoryPose

from math import sin, cos, atan2, pi
from datetime import datetime
import re

output_file_pos = open('/home/david/ros2_ws/log/speed_test_pos.txt', 'w')
output_file_pos.write('t,x,y,z,yaw\n')
output_file_vel = open('/home/david/ros2_ws/log/speed_test_vel.txt', 'w')
output_file_vel.write('t,vx,vy,vz,wz\n')
Tm = 0.1

class X3SpeedTestNode(Node):
    def __init__(self):
        super().__init__("X3_speed_test")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "pose", self.callback_pose, 10)
        self.create_timer(Tm, self.control_loop)
        self.current_position_ = None
        self.current_yaw_ = None
        self.get_logger().info("Speed Test Node has been started.")
        self.cnt = 0

    def callback_pose(self, msg):
        self.current_position_ = msg.position
        self.current_yaw_ = clip_angle(2*atan2(msg.orientation.z, msg.orientation.w))
        # Log info:
        output_file_pos.write(f"{self.get_clock().now().nanoseconds},{self.current_position_.x:.4f},{self.current_position_.y:.4f},{self.current_position_.z:.4f},{self.current_yaw_:.4f}\n")
      
    def control_loop(self):
        if self.current_position_ is None:
            return
        # Vx and Vy velocities to publish in world frame
        Vy = 0.0
        Vx = 0.0
        Vz = 0.0
        Wz = 1.0 if self.cnt <= 100 else 1.5
        # Transform velocities to drone frame
        vx_r =  cos(self.current_yaw_)*Vx + sin(self.current_yaw_)*Vy
        vy_r = -sin(self.current_yaw_)*Vx + cos(self.current_yaw_)*Vy
        vz_r = Vz
        w_r  = Wz
        # Apply velocity command
        msg = Twist()
        msg.linear.x = vx_r #clip(vx_r, -2.0, 2.0)
        msg.linear.y = vy_r #clip(vy_r, -2.0, 2.0)
        msg.linear.z = vz_r #clip(vz_r, -2.0, 2.0)
        msg.angular.z = w_r #clip(w_r, -2.0, 2.0)
        self.cmd_vel_publisher_.publish(msg)
        # Log info:
        output_file_vel.write(f"{self.get_clock().now().nanoseconds},{msg.linear.x:.4f},{msg.linear.y:.4f},{msg.linear.z:.4f},{msg.angular.z:.4f}\n")
        self.cnt += 1


def clip_angle(angle):
    if angle < -pi:
        angle += 2*pi
    elif angle >= pi:
        angle -= 2*pi
    return angle


def clip(u, u_min, u_max):
    return max(min(u, u_max), u_min)
    

def main(args=None):
    rclpy.init(args=args)
    node = X3SpeedTestNode()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        output_file_pos.flush()
        output_file_pos.close()
        output_file_vel.flush()
        output_file_vel.close()


if __name__ == "__main__":
    main()
