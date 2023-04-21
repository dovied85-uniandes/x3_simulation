#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, Vector3
from ros_gz_interfaces.msg import Marker
from ros_gz_interfaces.srv import SpawnEntity, ManageMarker, ManageMarkers
from drone_interfaces.srv import GetTrajectoryPose, GetViewpoint

from math import sin, cos, pi, ceil
from .planning import BoxPlanner, CylinderPlanner, PlanePlanner


class X3PlannerNode(Node):
    def __init__(self):
        super().__init__("X3_planner")
        self.declare_parameter("vel", 0.1)
        vel = self.get_parameter("vel").value
        self.declare_parameter("dist", 1.0)
        distance = self.get_parameter("dist").value
        # Se define el tipo de planeador
        #box = BoxPlanner(5.0, 4.0, 2.0, 6.0, 4.0, 1.0, pi/3)
        #box = CylinderPlanner(2.0, 2.0, -1.0, -4.0, 5.0)
        box = PlanePlanner(8, 8, 0, 0, 5, yaw=0, roll=0)
        # Se tranforma el edificio para que el punto inicial de la trayectoria este cerca al origen
        box.transform_by_closest((0, 0, 0))
        # Se planean los puntos de la ruta en marco local(dada la distancia)
        box.complete_keypoints(distance)
        # Se agregan los puntos del despegue del dron
        box.prepend_point((0,0,1,0), (0,0,0,0))
        box.prepend_point((0,0,0,0), (0,0,0,0))
        # Se planea la trayectoria completa en marco local (dada la velocidad y factor de suavizado k)
        box.complete_world_trajectory(vel, k=0.5)
        self.path_= box.get_trajectory() #(x,y,z,yaw)
        self.current_path_idx_ = 0
        # Load the box in the simulator
        self.add_building_to_world(box)
        # Load the markers in the simulator
        self.markers_ = box.get_viewpoints() #((x, y, z, yaw), gimbal_degrees)
        self.current_marker_idx_ = 0
        self.add_markers_to_world()
        self.draw_path()
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
        viewpoint = self.markers_[self.current_marker_idx_][0]
        gimbal_deg = self.markers_[self.current_marker_idx_][1]
        response.viewpoint.id = self.current_marker_idx_ + 1
        response.viewpoint.pose.position.x, response.viewpoint.pose.position.y, response.viewpoint.pose.position.z = viewpoint[0:3]
        response.viewpoint.pose.orientation.z, response.viewpoint.pose.orientation.w = sin(viewpoint[3]/2), cos(viewpoint[3]/2)
        response.viewpoint.gimbal_angle = gimbal_deg
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
        req.entity_factory.pose.orientation.x = sin(building.roll/2)*cos(building.yaw/2)
        req.entity_factory.pose.orientation.y = sin(building.roll/2)*sin(building.yaw/2)
        req.entity_factory.pose.orientation.z = cos(building.roll/2)*sin(building.yaw/2)
        req.entity_factory.pose.orientation.w = cos(building.roll/2)*cos(building.yaw/2)
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
        for i, (pt, _) in enumerate(self.markers_):
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
    
    def draw_path(self):
        client = self.create_client(ManageMarker, "marker")
        req = ManageMarker.Request()
        req.marker.action = 0
        req.marker.id = 999999
        req.marker.type = 5
        req.marker.material.diffuse.r, req.marker.material.diffuse.g, req.marker.material.diffuse.b, req.marker.material.diffuse.a = 0.0, 0.0, 1.0, 0.5
        req.marker.scale.x, req.marker.scale.y, req.marker.scale.z = 1.0, 1.0, 1.0
        for x, y, z, _ in self.path_[::ceil(len(self.path_)/1000.0)]:
            pt = Vector3()
            pt.x, pt.y, pt.z = x, y, z
            req.marker.point.append(pt)
        # we don't need to wait for a response, so we just call the service:
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Manage Marker...")
        client.call_async(req)
    

def main(args=None):
    rclpy.init(args=args)
    node = X3PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
