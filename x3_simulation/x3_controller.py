#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Vector3
from ros_gz_interfaces.srv import ManageMarker
from drone_interfaces.srv import GetTrajectoryPose

from math import sin, cos, atan2, pi
from datetime import datetime
import re

output_file = open('/home/david/ros2_ws/log/path' + re.sub(":|\s","-",str(datetime.now())[:-7]) + '.txt', 'w')
output_file.write('x,y,z,yaw,x_r,y_r,z_r,yaw_r,vx,vy,vz,w,vx_P,vy_P,vz_P,w_P,vx_I,vy_I,vz_I,w_I,vx_D,vy_D,vz_D,w_D\n')
Tm = 0.1

# Ley de Control: Kp + Ki*Tm/(1-z^-1) + Kd/(Tm + tau)*(1-z^-1)/(1-tau/(Tm + tau)*z^-1)
# K_COEFS = [[Kd_Vx, Ki_Vx, Kd_Vx, tau_Vx], [Kd_Vy, Ki_Vy, Kd_Vy, tau_Vy], [Kd_Vz, Ki_Vz, Kd_Vz, tau_Vz], [Kd_Wz, Ki_Wz, Kd_Wz, tau_Wz]]
K_COEFS = [[1.5, 0.5, 0.5, 0.025], [1.5, 0.5, 0.5, 0.025], [1.5, 0.5, 0.5, 0.025], [1.5, 0.5, 0.5, 0.025]]
#K_COEFS = [[2, 0, 0, 0], [2, 0, 0, 0], [2, 0, 0, 0], [2, 0, 0, 0]]

class X3ControllerNode(Node):
    def __init__(self):
        super().__init__("X3_controller")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "X3/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "X3/pose", self.callback_pose, 10)
        self.trajectory_subscriber_ = self.create_subscription(Pose, "X3/trajectory_pose", self.callback_trajectory, 10)
        self.create_timer(Tm, self.control_loop)
        self.current_position_ = None
        self.current_yaw_ = None
        self.desired_position_ = None
        self.desired_yaw_ = None
        self.trail_idx_ = 1000000
        self.trail_history_ = []
        self.e_last = [0,0,0,0]
        self.u_last = [[0,0,0,0], [0,0,0,0]] #[u_k-1_I, u_k-1_D]
        self.get_logger().info("X3 Controller Node has been started.")
        self.cnt = 0

    def callback_pose(self, msg):
        self.current_position_ = msg.position
        self.current_yaw_ = clip_angle(2*atan2(msg.orientation.z, msg.orientation.w))
    
    def callback_trajectory(self, msg):
        self.desired_position_ = msg.position
        self.desired_yaw_ = clip_angle(2*atan2(msg.orientation.z, msg.orientation.w))
    
    def control_loop(self):
        if self.current_position_ is None or self.desired_position_ is None:
            return
        # Calculate errors
        e = [self.desired_position_.x - self.current_position_.x, self.desired_position_.y - self.current_position_.y, self.desired_position_.z - self.current_position_.z, clip_angle(self.desired_yaw_ - self.current_yaw_)]
        # Calculate desired velocities in world frame with control law
        u_world = []
        for i in range(4):
            Kp, Ki, Kd, tau = K_COEFS[i]
            u_P = Kp*e[i]
            u_I = self.u_last[0][i] + Ki*Tm*e[i]
            u_D = (tau*self.u_last[1][i] + Kd*(e[i]-self.e_last[i]))/(Tm+tau)
            u_world.append((u_P + u_I + u_D, u_P, u_I, u_D))
            # Store history for the next control action
            self.e_last[i] = e[i]
            self.u_last[0][i] = u_I
            self.u_last[1][i] = u_D
        # Transform velocities to drone frame
        vx_r =  cos(self.current_yaw_)*u_world[0][0] + sin(self.current_yaw_)*u_world[1][0]
        vy_r = -sin(self.current_yaw_)*u_world[0][0] + cos(self.current_yaw_)*u_world[1][0]
        vz_r = u_world[2][0]
        w_r  = u_world[3][0]
        # Apply control command
        msg = Twist()
        msg.linear.x = vx_r #clip(vx_r, -2.0, 2.0)
        msg.linear.y = vy_r #clip(vy_r, -2.0, 2.0)
        msg.linear.z = vz_r #clip(vz_r, -2.0, 2.0)
        msg.angular.z = w_r #clip(w_r, -2.0, 2.0)
        self.cmd_vel_publisher_.publish(msg)
        self.cnt += 1
        # Log info:
        output_file.write(f"{self.current_position_.x:.4f},{self.current_position_.y:.4f},{self.current_position_.z:.4f},{self.current_yaw_:.4f},")
        output_file.write(f"{self.desired_position_.x:.4f},{self.desired_position_.y:.4f},{self.desired_position_.z:.4f},{self.desired_yaw_:.4f},")
        output_file.write(f"{u_world[0][0]:.4f},{u_world[1][0]:.4f},{u_world[2][0]:.4f},{u_world[3][0]:.4f},")
        output_file.write(f"{u_world[0][1]:.4f},{u_world[1][1]:.4f},{u_world[2][1]:.4f},{u_world[3][1]:.4f},")
        output_file.write(f"{u_world[0][2]:.4f},{u_world[1][2]:.4f},{u_world[2][2]:.4f},{u_world[3][2]:.4f},")
        output_file.write(f"{u_world[0][3]:.4f},{u_world[1][3]:.4f},{u_world[2][3]:.4f},{u_world[3][3]:.4f}\n")
        # Update trail
        self.trail_history_.append(self.current_position_)
        if len(self.trail_history_) == 11:
            self.draw_trail()
            self.trail_history_ = [self.current_position_]

    def draw_trail(self):
        client = self.create_client(ManageMarker, "marker")
        req = ManageMarker.Request()
        req.marker.action = 0
        req.marker.id = self.trail_idx_
        self.trail_idx_ += 1
        if self.trail_idx_ == 2000000:
            self.trail_idx_ = 1000000
        req.marker.type = 3
        req.marker.material.diffuse.r, req.marker.material.diffuse.g, req.marker.material.diffuse.b, req.marker.material.diffuse.a = 0.0, 0.0, 0.0, 1.0
        req.marker.lifetime.sec = 900
        req.marker.scale.x, req.marker.scale.y, req.marker.scale.z = 1.0, 1.0, 1.0
        for trail_pt in self.trail_history_:
            pt = Vector3()
            pt.x, pt.y, pt.z = trail_pt.x, trail_pt.y, trail_pt.z
            req.marker.point.append(pt)
        # we don't need to wait for a response, so we just call the service:
        client.call_async(req)


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
    node = X3ControllerNode()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        output_file.flush()
        output_file.close()


if __name__ == "__main__":
    main()
