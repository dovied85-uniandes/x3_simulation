#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from ros_gz_interfaces.msg import Marker
from ros_gz_interfaces.srv import ManageMarker
from drone_interfaces.srv import GetViewpoint

from math import sin, cos, atan2, sqrt, copysign


class X3CameraTaskNode(Node):
    def __init__(self):
        super().__init__("X3_camera_task")
        self.pose_subscriber_ = self.create_subscription(Pose, "X3/pose", self.callback_pose, 10)
        self.target_marker_ = None
        self.get_logger().info("X3 Camera Task Node has been started.")
        self.get_next_viewpoint()
    
    def get_next_viewpoint(self):
        client = self.create_client(GetViewpoint, "X3/get_viewpoint")
        req = GetViewpoint.Request()
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Get Viewpoint...")
        future = client.call_async(req)
        future.add_done_callback(self.cb_get_next_viewpoint)
    
    def cb_get_next_viewpoint(self, future):
        try:
            response = future.result()
            self.target_marker_ = response.viewpoint
            self.get_logger().info(f"Viewpoint #{self.target_marker_.id} received. Gimbal angle: {response.viewpoint.gimbal_angle:.2f}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_pose(self, msg):
        position = msg.position
        # update marker if the drone is close
        if self.target_marker_ is not None:
            mk_position = self.target_marker_.pose.position
            d = sqrt((position.x - mk_position.x)**2 + (position.y - mk_position.y)**2 + (position.z - mk_position.z)**2)
            if d < 0.10:
                self.edit_current_marker()
                self.get_next_viewpoint()

    def edit_current_marker(self):
        mk_position = self.target_marker_.pose.position
        client = self.create_client(ManageMarker, "marker")
        req = ManageMarker.Request()
        req.marker.action = 0
        req.marker.id = self.target_marker_.id
        req.marker.type = 6
        req.marker.material.diffuse.r, req.marker.material.diffuse.g, req.marker.material.diffuse.b, req.marker.material.diffuse.a = 0.0, 1.0, 0.0, 0.8
        req.marker.scale.x, req.marker.scale.y, req.marker.scale.z = 0.1, 0.1, 0.1
        req.marker.pose.position = mk_position
        # we don't need to wait for a response, so we just call the service:
        client.call_async(req)


def clip_angle(angle):
    if angle < -pi:
        angle += 2*pi
    elif angle >= pi:
        angle -= 2*pi
    return angle
    

def main(args=None):
    rclpy.init(args=args)
    node = X3CameraTaskNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
