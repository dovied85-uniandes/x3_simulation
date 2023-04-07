#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from ros_gz_interfaces.msg import Marker
from ros_gz_interfaces.srv import SpawnEntity, ManageMarkers
from drone_interfaces.srv import GetTrajectoryPose, GetViewpoint

from math import sin, cos, pi
from .planning import BoxPlanner, CylinderPlanner


class X3PlannerNode(Node):
    def __init__(self):
        super().__init__("X3_planner")
        box = BoxPlanner(5.0, 4.0, 2.0, 6.0, 4.0, 1.0, pi/3)
        #box = CylinderPlanner(3.0, 2.0, -1.0, -4.0, 1.0)
        box.transform_by_closest((0, 0, 0))
        self.declare_parameter("vel", 0.1)
        vel = self.get_parameter("vel").value
        self.path_ = box.complete_world_trajectory((0,0,0,0), 2.0, vel) #(x,y,z,yaw,is_VP)
        self.current_path_idx_ = 0
        # Load the box in the simulator
        self.add_building_to_world(box)
        # Load the markers in the simulator
        self.markers_ = list(filter(lambda pt: pt[4], self.path_))
        self.current_marker_idx_ = 0
        self.add_markers_to_world()
        # Trajectory publisher
        self.trajectory_publisher_ = self.create_publisher(Pose, "X3/trajectory_pose", 10)
        self.create_timer(0.1, self.publish_trajectory_pose)
        # Viewpoint service
        self.service_viewpoint_ = self.create_service(GetViewpoint, "X3/get_viewpoint", self.callback_get_viewpoint)
        self.get_logger().info("X3 Planner Node has been started.")
       
    def publish_trajectory_pose(self):
        curr_pt = self.path_[self.current_path_idx_]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = curr_pt[0:3]
        pose.orientation.z, pose.orientation.w = sin(curr_pt[3]/2), cos(curr_pt[3]/2)
        self.trajectory_publisher_.publish(pose)
        # increase reference index
        if self.current_path_idx_ < len(self.path_)-1:
            self.current_path_idx_ += 1
  
    def callback_get_viewpoint(self, request, response):
        viewpoint = self.markers_[self.current_marker_idx_]
        response.viewpoint.id = self.current_marker_idx_ + 1
        response.viewpoint.pose.position.x, response.viewpoint.pose.position.y, response.viewpoint.pose.position.z = viewpoint[0:3]
        response.viewpoint.pose.orientation.z, response.viewpoint.pose.orientation.w = sin(viewpoint[3]/2), cos(viewpoint[3]/2)
        response.viewpoint.gimbal_angle = 0.0 # FALTA AGREGAR EL ANGULO A LOS VIEWPOINTS
        self.current_marker_idx_ += 1
        if self.current_marker_idx_ == len(self.markers_):
            self.current_marker_idx_ = 0
        return response

    def add_building_to_world(self, building):
        client = self.create_client(SpawnEntity, "world/quadcopter_teleop/create")
        req = SpawnEntity.Request()
        req.entity_factory.sdf = building.sdf_string()
        req.entity_factory.name = 'building'
        req.entity_factory.pose.position.x, req.entity_factory.pose.position.y, req.entity_factory.pose.position.z = building.x, building.y, building.z
        req.entity_factory.pose.orientation.z, req.entity_factory.pose.orientation.w = sin(building.yaw/2), cos(building.yaw/2)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Spawn Entity...")
        future = client.call_async(req)
        future.add_done_callback(self.cb_add_building_to_world)
    
    def cb_add_building_to_world(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def add_markers_to_world(self):
        client = self.create_client(ManageMarkers, "marker_array")
        req = ManageMarkers.Request()
        for i, pt in enumerate(self.markers_):
            mk = Marker()
            mk.action = 0
            mk.id = i + 1
            mk.type = 6
            mk.material.diffuse.r, mk.material.diffuse.g, mk.material.diffuse.b, mk.material.diffuse.a = 1.0, 0.0, 0.0, 0.8
            mk.scale.x, mk.scale.y, mk.scale.z = 0.1, 0.1, 0.1
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = pt[0], pt[1], pt[2]
            req.markers.markers.append(mk)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Manage Markers...")
        future = client.call_async(req)
        future.add_done_callback(self.cb_add_markers_to_world)

    def cb_add_markers_to_world(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    

def main(args=None):
    rclpy.init(args=args)
    node = X3PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
