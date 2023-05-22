#!/usr/bin/env python3
from math import pi, ceil, sin, cos, tan, degrees, radians, sqrt, atan, atan2
from functools import partial

FOV = 79
AR = 16.0/9.0
Tm = 0.1

class Planner():

    # SDF
    def sdf_string(self):
        pass

    # Transforma dimensiones y orientación del edificio para que el punto de vista inicial sea el más cercano a pt
    def transform_by_closest(self, pt):
        pass

    # Calcula los puntos de vista y los puntos de la trayectoria (en marco local) y las conexiones entre los puntos de trayectoria
    # view_pt: ((x, y, z, yaw), gimbal_degrees, (face_id, row_id, col_id)
    # path_pt: ((x, y, z, yaw), (Vx, Vy, Vz, Omega)) --> velocidades normalizadas!
    # conn:    function
    def complete_keypoints(self, distance):
        pass

    # Retorna los viewpoints en coordenas globales:
    def get_viewpoints(self):
        return list(map(lambda x: (self.to_world(x[0]), x[1], x[2]), self.view_pts))
    
    # Retorna la trayectoria en coordenas globales:
    def get_trajectory(self):
        return list(map(self.to_world, self.trajectory))

    # Transforma un punto o vector de coordenadas locales a globales
    def to_world(self, pt, vector=False):
        x =  cos(self.yaw)*pt[0] - sin(self.yaw)*cos(self.roll)*pt[1] + sin(self.yaw)*sin(self.roll)*pt[2] + (0.0 if vector else self.x)
        y =  sin(self.yaw)*pt[0] + cos(self.yaw)*cos(self.roll)*pt[1] - cos(self.yaw)*sin(self.roll)*pt[2] + (0.0 if vector else self.y)
        z =                                      sin(self.roll)*pt[1]               + cos(self.roll)*pt[2] + (0.0 if vector else self.z)
        if len(pt) == 3:
            return (x, y, z)
        yaw = clip_angle(pt[3] + self.yaw)
        return (x, y, z, yaw)
    
    # Transforma un punto o vector de coordenadas globales a locales
    def to_local(self, pt, vector=False):
        tx =                 cos(self.yaw)*self.x +                sin(self.yaw)*self.y
        ty = -cos(self.roll)*sin(self.yaw)*self.x + cos(self.roll)*cos(self.yaw)*self.y + sin(self.roll)*self.z
        tz =  sin(self.roll)*sin(self.yaw)*self.x - sin(self.roll)*cos(self.yaw)*self.y + cos(self.roll)*self.z
        x =                 cos(self.yaw)*pt[0] +                sin(self.yaw)*pt[1]                        + (0.0 if vector else -tx)
        y = -cos(self.roll)*sin(self.yaw)*pt[0] + cos(self.roll)*cos(self.yaw)*pt[1] + sin(self.roll)*pt[2] + (0.0 if vector else -ty)
        z =  sin(self.roll)*sin(self.yaw)*pt[0] - sin(self.roll)*cos(self.yaw)*pt[1] + cos(self.roll)*pt[2] + (0.0 if vector else -tz)
        yaw = clip_angle(pt[3] + (0.0 if vector else -self.yaw))
        return (x, y, z, yaw)

    # Agrega un punto (en coordenadas globales) al inicio de la trayectoria, para unirlo por línea recta
    def prepend_point(self, p, v):
        self.path_pts.insert(0, (self.to_local(p), self.to_local(v, True)))
        self.conns.insert(0, straight_path_3d)

    # Agrega un punto (en coordenadas globales) al final de la trayectoria, para unirlo por línea recta
    def append_point(self, p, v):
        self.path_pts.append((self.to_local(p), self.to_local(v, True)))
        self.conns.append(straight_path_3d)

    # Calcula la trayectoria completa (en marco local)
    # Cada elemento de la trayectoria final es: (x, y, z, yaw)
    def complete_world_trajectory(self, drone_vel, k=0.5):
        N = len(self.path_pts)
        self.trajectory = []
        dt = 0.0
        for i in range(N - 1):
            p1, v1, p2, v2 = self.path_pts[i][0], self.path_pts[i][1], self.path_pts[i + 1][0], self.path_pts[i + 1][1]
            connecting_fcn = self.conns[i]
            traj_segm, dt = connecting_fcn(p1, v1, p2, v2, drone_vel, dt, k)
            self.trajectory += traj_segm


class PlanePlanner(Planner):

    def __init__(self, length, width, x, y, z, yaw=0, roll=0):
        self.length = 1.0*length
        self.width = 1.0*width
        self.x = 1.0*x
        self.y = 1.0*y
        self.z = 1.0*z
        self.yaw = 1.0*yaw
        self.roll = 1.0*roll
    
    def transform_by_closest(self, pt):
        corners = [(-self.length/2, -self.width/2, 0), (self.length/2, -self.width/2, 0), (self.length/2, self.width/2, 0), (-self.length/2, self.width/2, 0)]
        corners = list(map(self.to_world, corners))
        ds = list(map(lambda p: (p[0]-pt[0])**2 + (p[1]-pt[1])**2 + (p[2]-pt[2])**2, corners))
        min_d = min(ds)
        self.closest_pt = 0
        if ds[1] == min_d:
            self.closest_pt = 1
            if self.roll == 0:
                self.length, self.width = self.width, self.length
                self.yaw = clip_angle(self.yaw + pi/2)
        elif ds[2] == min_d:
            self.closest_pt = 2
            if self.roll == 0:
                self.yaw = clip_angle(self.yaw + pi)
        elif ds[3] == min_d:
            self.closest_pt = 3
            if self.roll == 0:
                self.length, self.width = self.width, self.length
                self.yaw = clip_angle(self.yaw - pi/2)
    
    def complete_keypoints(self, distance, pt0=(0,0,0,0)):
        dl = 2*distance*tan(radians(FOV/2))
        dh = dl / AR
        M = int(ceil(self.length / dl))
        N = int(ceil(self.width / dh))
        yaw = pi/2
        self.view_pts = []
        self.path_pts = []
        self.conns = []
        # Puntos iniciales de seguridad (si se comienza por la parte posterior del plano):
        pt0 = self.to_local(pt0)
        x1, x2, y1, y2 = (1-M)*0.5*dl, (M-1)*0.5*dl, (1-N)*0.5*dh, (N-1)*0.5*dh
        safe_pts = [(x1 - distance, y1, 0), (x1, y1 - distance, 0), (x2, y1 - distance, 0), (x2 + distance, y1, 0), (x2 + distance, y2, 0), (x2, y2 + distance, 0), (x1, y2 + distance, 0), (x1 - distance, y2, 0)]
        safe_pts_distances = list(map(lambda pt: (pt[0] - pt0[0])**2 + (pt[1] - pt0[1])**2 + (pt[2] - pt0[2])**2, safe_pts))
        min_idx = safe_pts_distances.index(min(safe_pts_distances))
        if pt0[2] < 0:
            x, y, z = safe_pts[min_idx]
            self.path_pts.append(((x, y, z, yaw), (0.0, 0.0, 1.0, 0.0)))
            self.conns.append(straight_path_3d)
            vx = 1.0 if min_idx in [0,7] else -1.0 if min_idx in [3,4] else 0.0
            vy = 1.0 if min_idx in [1,2] else -1.0 if min_idx in [5,6] else 0.0
            self.path_pts.append(((x, y, distance, yaw), (vx, vy, 0.0, 0.0)))
            self.conns.append(straight_path_3d)
        if min_idx < 2:
            i, j, vi, vj = 0, 0, 1, 1
        elif min_idx < 4:
            i, j, vi, vj = M-1, 0, -1, 1
        elif min_idx < 6:
            i, j, vi, vj = M-1, N-1, -1, -1
        else:
            i, j, vi, vj = 0, N-1, 1, -1
        z = distance
        while i >= 0 and i < M and j >= 0 and j < N:
            x = (-M/2.0 + 0.5 + i)*dl
            y = (-N/2.0 + 0.5 + j)*dh
            self.view_pts.append(((x, y, z, yaw), -90.0 + degrees(self.roll), (0, j, i)))
            # movimiento horizontal
            if M >= N:
                if i == 0 or i == M - 1:
                    self.path_pts.append(((x, y, z, yaw), (vi*1.0, 0.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
                if (i == M - 1 and vi == 1) or (i == 0 and vi == -1):
                    j += vj
                    vi *= -1
                else:
                    i += vi
            # movimiento vertical
            else:
                if j == 0 or j == N - 1:
                    self.path_pts.append(((x, y, z, yaw), (0.0, vj*1.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
                if (j == N - 1 and vj == 1) or (j == 0 and vj == -1):
                    i += vi
                    vj *= -1
                else:
                    j += vj
        # Regresar al punto inicial
        if pt0[2] < 0:
            p, (vx, vy, vz, w) = self.path_pts[1]
            self.path_pts.append((p, (-vx, -vy, -vz, 0.0)))
            self.conns.append(straight_path_3d)
        p, (vx, vy, vz, w) = self.path_pts[0]
        self.path_pts.append((p, (-vx, -vy, -vz, 0.0)))
        self.conns.append(straight_path_3d)
    
    def sdf_string(self):
        return f"<?xml version=\"1.0\" ?>\
        <sdf version=\"1.6\">\
            <model name=\"test_box\">\
                <static>true</static>\
                <link name=\"link\">\
                    <!--\
                    <collision name=\"collision\">\
                        <geometry>\
                            <plane>\
                                <normal>0 0 1</normal>\
                                <size>{self.length} {self.width}</size>\
                            </plane>\
                        </geometry>\
                    </collision>\
                    -->\
                    <visual name=\"visual\">\
                        <geometry>\
                            <plane>\
                                <normal>0 0 1</normal>\
                                <size>{self.length} {self.width}</size>\
                            </plane>\
                        </geometry>\
                        <material>\
                            <ambient>0.2 0.2 0.2 1.0</ambient>\
                            <diffuse>0.5 1.0 1.0 1.0</diffuse>\
                            <specular>0.0 0.0 0.0 1.0</specular>\
                        </material>\
                    </visual>\
                </link>\
            </model>\
        </sdf>"


class BoxPlanner(Planner):

    def __init__(self, length, width, height, x, y, z, yaw=0):
        self.length = length
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.roll = 0

    def transform_by_closest(self, pt):
        corners = [(-self.length/2, -self.width/2, -self.height/2), (self.length/2, -self.width/2, -self.height/2), (self.length/2, self.width/2, -self.height/2), (-self.length/2, self.width/2, -self.height/2)]
        corners = list(map(self.to_world, corners))
        ds = list(map(lambda p: (p[0]-pt[0])**2 + (p[1]-pt[1])**2 + (p[2]-pt[2])**2, corners))
        min_d = min(ds)
        if ds[1] == min_d:
            self.length, self.width = self.width, self.length
            self.yaw = clip_angle(self.yaw + pi/2)
        elif ds[2] == min_d:
            self.yaw = clip_angle(self.yaw + pi)
        elif ds[3] == min_d:
            self.length, self.width = self.width, self.length
            self.yaw = clip_angle(self.yaw - pi/2)
        
    def complete_keypoints(self, distance, pt0=(0,0,0,0)):
        dl = 2*distance*tan(radians(FOV/2))
        dh = dl / AR
        M = int(ceil(self.length / dl))
        N = int(ceil(self.width / dl))
        L = int(ceil(self.height / dh))
        self.view_pts = []
        self.path_pts = []
        self.conns = []
        # Punto inicial de seguridad (si se comienza muy cerca al edificio):
        pt0 = self.to_local(pt0)
        if pt0[0] < -self.length/2 and pt0[1] > -self.width/2:
            self.path_pts.append(((-self.length/2 - distance, -self.width/2 - distance, (-L/2.0 + 0.5)*dh, pi/2), (1.0, 0.0, 0.0, 0.0)))
            self.conns.append(straight_path_3d)
        # Las 4 caras:
        for k in range(0, L):
            z = (-L/2.0 + 0.5 + k)*dh
            # cara 1:
            yaw = pi/2
            y = -self.width/2.0 - distance
            x = -self.length/2.0
            if k > 0:
                self.path_pts.append(((x, y, z, yaw), (1.0, 0.0, 0.0, 0.0)))
                self.conns.append(straight_path_3d)
            for i in range(0, M):
                x = (-M/2.0 + 0.5 + i)*dl
                self.view_pts.append(((x, y, z, yaw), 0.0, (0, k, i)))
                if i == 0 and k == 0:
                    self.path_pts.append(((x, y, z, yaw), (1.0, 0.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
            x = self.length/2.0
            self.path_pts.append(((x, y, z, yaw), (1.0, 0.0, 0.0, 0.0)))
            self.conns.append(partial(arc_3d, distance, (self.length/2.0, -self.width/2.0)))
            # cara 2:
            yaw = pi
            x = self.length/2.0 + distance
            y = -self.width/2.0
            self.path_pts.append(((x, y, z, yaw), (0.0, 1.0, 0.0, 0.0)))
            self.conns.append(straight_path_3d)
            for j in range(0, N):
                y = (-N/2.0 + 0.5 + j)*dl
                self.view_pts.append(((x, y, z, yaw), 0.0, (1, k, j)))
            y = self.width/2.0
            self.path_pts.append(((x, y, z, yaw), (0.0, 1.0, 0.0, 0.0)))
            self.conns.append(partial(arc_3d, distance, (self.length/2.0, self.width/2.0)))
            # cara 3:
            yaw = -pi/2
            y = self.width/2.0 + distance
            x = self.length/2.0
            self.path_pts.append(((x, y, z, yaw), (-1.0, 0.0, 0.0, 0.0)))
            self.conns.append(straight_path_3d)
            for i in range(0, M):
                x = (M/2.0 - 0.5 - i)*dl
                self.view_pts.append(((x, y, z, yaw), 0.0, (2, k, i)))
            x = -self.length/2.0
            self.path_pts.append(((x, y, z, yaw), (-1.0, 0.0, 0.0, 0.0)))
            self.conns.append(partial(arc_3d, distance, (-self.length/2.0, self.width/2.0)))
            # cara 4:
            yaw = 0.0
            x = -self.length/2.0 - distance
            y = self.width/2.0
            self.path_pts.append(((x, y, z, yaw), (0.0, -1.0, 0.0, 0.0)))
            self.conns.append(straight_path_3d)
            for j in range(0, N):
                y = (N/2.0 - 0.5 - j)*dl
                self.view_pts.append(((x, y, z, yaw), 0.0, (3, k, j)))
                if k == L - 1 and j == N - 1:
                    self.path_pts.append(((x, y, z, yaw), (0.0, 0.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
            if k < L - 1:
                y = -self.width/2.0
                self.path_pts.append(((x, y, z, yaw), (0.0, -1.0, 0.0, 0.0)))
                self.conns.append(partial(arc_3d, distance, (-self.length/2.0, -self.width/2.0)))
        # La cara superior:
        N = int(ceil(self.width / dh))
        yaw = pi/2
        z = self.height/2.0 + distance
        self.path_pts.append(((x, y, z, yaw), (0.0, 0.0, 0.0, 0.0)))
        self.conns.append(straight_path_3d)
        vel_sign = 1
        i = j = 0
        while i >= 0 and i < M and j >= 0 and j < N:
            x = (-M/2.0 + 0.5 + i)*dl
            y = (-N/2.0 + 0.5 + j)*dh
            self.view_pts.append(((x, y, z, yaw), -90.0, (4, j, i)))
            # movimiento horizontal
            if M >= N:
                if i == 0 or i == M - 1:
                    self.path_pts.append(((x, y, z, yaw), (vel_sign*1.0, 0.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
                if (i == M - 1 and vel_sign == 1) or (i == 0 and vel_sign == -1):
                    j += 1
                    vel_sign *= -1
                else:
                    i += vel_sign
            # movimiento vertical
            else:
                if j == 0 or j == N - 1:
                    self.path_pts.append(((x, y, z, yaw), (0.0, vel_sign*1.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)
                if (j == N - 1 and vel_sign == 1) or (j == 0 and vel_sign == -1):
                    i += 1
                    vel_sign *= -1
                else:
                    j += vel_sign
    
    def sdf_string(self):
        return f"<?xml version=\"1.0\" ?>\
        <sdf version=\"1.6\">\
            <model name=\"test_box\">\
                <static>true</static>\
                <link name=\"link\">\
                    <collision name=\"collision\">\
                        <geometry>\
                            <box>\
                                <size>{self.length} {self.width} {self.height}</size>\
                            </box>\
                        </geometry>\
                    </collision>\
                    <visual name=\"visual\">\
                        <geometry>\
                            <box>\
                                <size>{self.length} {self.width} {self.height}</size>\
                            </box>\
                        </geometry>\
                        <material>\
                            <ambient>0.2 0.2 0.2 1.0</ambient>\
                            <diffuse>0.5 1.0 1.0 1.0</diffuse>\
                            <specular>0.0 0.0 0.0 1.0</specular>\
                        </material>\
                    </visual>\
                </link>\
            </model>\
        </sdf>"


class CylinderPlanner(Planner):

    def __init__(self, radius, height, x, y, z):
        self.radius = radius
        self.height = height
        self.x = x
        self.y = y
        self.z = z
        self.yaw = 0
        self.roll = 0

    def transform_by_closest(self, pt):
        self.yaw = atan2(pt[1] - self.y, pt[0] - self.x)    

    def complete_keypoints(self, distance):
        self.view_pts = []
        self.path_pts = []
        self.conns = []
        # Cara del cilindro
        dl = round(2 * distance * tan(radians(FOV/2)), 2)
        dh = round(dl / AR, 2)
        N = int(ceil(2*pi / min(2*atan(dl / (2*self.radius)), pi/2)))
        L = int(ceil(self.height / dh))
        pts = []
        for k in range(0, L):
            z = (-L/2.0 + 0.5 + k)*dh
            for i in range(N):
                theta = 2*pi / N * i
                x = (self.radius + distance)*cos(theta)
                y = (self.radius + distance)*sin(theta)
                yaw = clip_angle(theta + pi)
                self.view_pts.append(((x, y, z, yaw), 0.0, (0, k, i)))
                if i == 0 or i == N - 1:
                    vx = 0.0 if i == N - 1 and k == L - 1 else -sin(theta)
                    vy = 0.0 if i == N - 1 and k == L - 1 else cos(theta)
                    self.path_pts.append(((x, y, z, yaw), (vx, vy, 0.0, 0.0)))
                    if i == N - 1 and k == L - 1:
                        self.conns.append(straight_path_3d)
                    else:
                        self.conns.append(partial(arc_3d, self.radius + distance, (0.0, 0.0)))
        # Techo del cilindro
        N = int(ceil(2*self.radius / dh))
        z = self.height/2.0 + distance
        yaw = pi
        self.path_pts.append(((x, y, z, yaw), (0.0, 0.0, 0.0, 0.0)))
        for i in range(N):
            x = (N/2.0 - 0.5 - i)*dh
            l = 2*sqrt(self.radius**2 - x**2)
            M = int(ceil(l / dl))
            for j_aux in range(M):
                j = j_aux if i % 2 == 0 else M - 1 - j_aux
                vel_sign = 1 if i % 2 == 0 else -1
                y = (-M/2.0 + 0.5 + j)*dl
                self.view_pts.append(((x, y, z, yaw), -90.0, (1, i, j)))
                if j == 0 or j == M - 1:
                    self.path_pts.append(((x, y, z, yaw), (0.0, vel_sign*1.0, 0.0, 0.0)))
                    self.conns.append(straight_path_3d)    
        return pts

    def sdf_string(self):
        return f"<?xml version=\"1.0\" ?>\
        <sdf version=\"1.6\">\
            <model name=\"test_box\">\
                <static>true</static>\
                <link name=\"link\">\
                    <collision name=\"collision\">\
                        <geometry>\
                            <cylinder>\
                                <radius>{self.radius}</radius>\
                                <length>{self.height}</length>\
                            </cylinder>\
                        </geometry>\
                    </collision>\
                    <visual name=\"visual\">\
                        <geometry>\
                            <cylinder>\
                                <radius>{self.radius}</radius>\
                                <length>{self.height}</length>\
                            </cylinder>\
                        </geometry>\
                        <material>\
                            <ambient>0.2 0.2 0.2 1.0</ambient>\
                            <diffuse>0.5 1.0 1.0 1.0</diffuse>\
                            <specular>0.0 0.0 0.0 1.0</specular>\
                        </material>\
                    </visual>\
                </link>\
            </model>\
        </sdf>"


# HELPER FUNCTIONS

# retorna: t1, a1, t2, a2
def acceleration_times(x1, v1, x2, v2, T, k=0.25, is_yaw=False):
    if k == 0:
        return 0, 0, 0, 0
    dx = x2 - x1 if not is_yaw else clip_angle(x2 - x1)
    t1 = t2 = k*T/2
    vc = (2*dx - k*T/2*(v1 + v2))/(2*T - k*T)
    a1 = (vc - v1)/t1
    a2 = (v2 - vc)/t2
    return t1, a1, t2, a2

# retorna: t1, a1, t2, a2
def acceleration_times_2(x1, v1, x2, v2, T, k=0.25, is_yaw=False):
    if k == 0:
        return 0, 0, 0, 0
    dx = x2 - x1 if not is_yaw else clip_angle(x2 - x1)
    dv = v2 - v1
    # Velocidad se mantiene: las acceleraciones son de signos opuestos
    if dv == 0:
        t = k*T/2
        accel = 4*(dx - T*v1)/(k*(2 - k)*T**2)
        return t, accel, t, -accel
    # Velocidad cambia:
    # Aceleraciones en ambos tramos son de signos opuestos
    a = 2*dv
    b = 4*dx - 2*T*v1*(1 - k) - 2*T*v2*(1 + k)
    c = (k*T)**2*dv + 2*k*T*(T*v1 - dx)
    if b**2 - 4*a*c >= 0:
        t1 = (-b + sqrt(b**2 - 4*a*c))/(2*a)
        t2 = k*T - t1
        if t1 >= 0 and t2 >= 0:
            vc = (2*dx - v1*t1 - v2*t2)/(2*T - t1 - t2)
            return t1, (vc - v1)/t1, t2, (v2 - vc)/t2
        t1 = (-b - sqrt(b**2 - 4*a*c))/(2*a)
        t2 = k*T - t1
        if t1 >= 0 and t2 >= 0:
            vc = (2*dx - v1*t1 - v2*t2)/(2*T - t1 - t2)
            return t1, (vc - v1)/t1, t2, (v2 - vc)/t2
    # Aceleraciones en ambos tramos son del mismo signo
    t1 = (2*k*(dx - T*v1) - k**2*T*dv)/(2*(1 - k)*dv)
    t2 = k*T - t1
    if t1 >= 0 and t2 >= 0:
        accel = dv/(k*T)
        return t1, accel, t2, accel
    # En este punto no se puede satisfacer la condición del tiempo k
    t1 = 2*(v2*T - dx)/dv
    if t1 > 0 and t1 <= T:
        return t1, dv/t1, 0.0, 0.0
    t2 = 2*(dx - v1*T)/dv
    if t2 > 0 and t2 <= T:
        return 0.0, 0.0, t2, dv/t2
    # En este punto no se pudo solucionar el problema
    return None, None, None, None

def straight_path_1d(x1, v1, x2, v2, T, t0, k=0.25, is_yaw=False):
    t1, a1, t2, a2 = acceleration_times(x1, v1, x2, v2, T, k, is_yaw)
    vc = v1 + a1*t1 if k != 0 else (x2 - x1)/T
    xa = x1 + v1*t1 + a1*t1**2/2
    xb = xa + vc*(T - t1 - t2)
    path = []
    N = int((T - t0)/Tm) + 1
    t = t0
    for _ in range(N):
        if t <= t1:
            x = x1 + v1*t + a1*t**2/2
        elif t <= T-t2:
            x = xa + vc*(t - t1)
        else:
            x = xb + vc*(t - (T - t2)) + a2*(t - (T - t2))**2/2
        path.append(x if not is_yaw else clip_angle(x))
        t += Tm
    return path

def straight_path_3d(p1, v1, p2, v2, vel, t0, k=0.25):
    d = sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)
    T = d/vel
    path_x   = straight_path_1d(p1[0], v1[0]*vel, p2[0], v2[0]*vel, T, t0, k)
    path_y   = straight_path_1d(p1[1], v1[1]*vel, p2[1], v2[1]*vel, T, t0, k)
    path_z   = straight_path_1d(p1[2], v1[2]*vel, p2[2], v2[2]*vel, T, t0, k)
    path_yaw = straight_path_1d(p1[3], v1[3]*vel, p2[3], v2[3]*vel, T, t0, k, True)
    return list(zip(path_x, path_y, path_z, path_yaw)), (int((T - t0)/Tm) + 1)*Tm + t0 - T

def arc_3d(radius, center, p1, v1, p2, v2, vel, t0, k=0.25):
    theta1 = atan2(p1[1] - center[1], p1[0] - center[0])
    theta2 = atan2(p2[1] - center[1], p2[0] - center[0])
    if theta2 < theta1:
        theta2 += 2*pi
    d = sqrt((p1[2] - p2[2])**2 + (radius*(theta1 - theta2))**2)
    T = d/vel
    omega1 = vel*sqrt(v1[0]**2 + v1[1]**2)/radius
    omega2 = vel*sqrt(v2[0]**2 + v2[1]**2)/radius
    thetas = straight_path_1d(theta1, omega1, theta2, omega2, T, t0, k)
    path_x = list(map(lambda t: center[0] + radius*cos(t), thetas))
    path_y = list(map(lambda t: center[1] + radius*sin(t), thetas))
    path_z = straight_path_1d(p1[2], v1[2]*vel, p2[2], v2[2]*vel, T, t0, k)
    path_yaw = list(map(lambda x, y: atan2(center[1] - y, center[0] - x), path_x, path_y))
    return list(zip(path_x, path_y, path_z, path_yaw)), (int((T - t0)/Tm) + 1)*Tm + t0 - T

#(-pi, pi]
def clip_angle(angle):
    while angle <= -pi:
        angle += 2*pi
    while angle > pi:
        angle -= 2*pi
    return angle