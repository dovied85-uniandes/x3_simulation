#!/usr/bin/env python3
from math import pi, ceil, sin, cos, tan, radians, sqrt, atan, atan2

FOV = 79
AR = 16.0/9.0

class Planner():

    # SDF
    def sdf_string(self):
        pass

    # Transforma dimensiones y orientación del edificio para que el punto de vista inicial sea el más cercano a pt
    def transform_by_closest(self, pt):
        pass

    # Calcula los puntos de vista para escanear el alrededor del edificio a una distancia dada
    def spiral(self, distance):
        pass
    
    # Calcula los puntos de vista para escanear techo del edificio a una distancia dada
    def top_floor(self, distance):
        pass
    
    # Une los dos conjuntos de puntos de vista (alrededor + techo)
    def complete_viewpoints(self, distance):
        return self.spiral(distance) + self.top_floor(distance)

    # Calcula la trayectoria completa (en marco local) uniendo los puntos de vista generando puntos intermedios para viajar a la velocidad dada (puntos espaciados a 100ms)
    def complete_local_trajectory(self, distance, vel):
        pass

    # Transforma un punto de coordenadas locales a globales
    def to_world(self, pt):
        x = cos(self.yaw)*pt[0] - sin(self.yaw)*pt[1] + self.x
        y = sin(self.yaw)*pt[0] + cos(self.yaw)*pt[1] + self.y
        z = pt[2] + self.z
        if len(pt) == 3:
            return (x, y, z)
        return (x, y, z, clip_angle(pt[3] + self.yaw), pt[4])

    # Calcula la trayectoria completa (en marco global) desde una configuracion inicial q0
    # Cada elemento de la trayectoria es: (x, y, z, yaw, is_viewpoint)
    def complete_world_trajectory(self, q0, distance, vel):
        path = self.complete_local_trajectory(distance, vel)
        path = list(map(self.to_world, path))
        takeoff = straight_path_3d((q0[0], q0[1], q0[2], q0[3], False), (0.0, 0.0, 1.0, 0.0, False), 0.5)
        path = takeoff + straight_path_3d((0.0, 0.0, 1.0, 0.0, False), path[0], vel)[:-1] + path
        return path



class BoxPlanner(Planner):

    def __init__(self, length, width, height, x, y, z, yaw=0):
        self.length = length
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

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
        
    def spiral(self, distance):
        dl = round(2 * distance * tan(radians(FOV/2)), 2)
        dh = round(dl / AR, 2)
        M = int(ceil(self.length / dl))
        N = int(ceil(self.width / dl))
        L = int(ceil(self.height / dh))
        pts = []
        for k in range(0, L):
            z = round((-L/2.0 + 0.5 + k)*dh, 2)
            # cara 1:
            y = round(-self.width/2.0 - distance, 2)
            for i in range(-1, M+1):
                x = round(-self.length/2.0 if i == -1 else self.length/2.0 if i == M else (-M/2.0 + 0.5 + i)*dl, 2)
                pts.append((x, y, z, pi/2, False if i == -1 or i == M else True))
            # cara 2:
            x = round(self.length/2.0 + distance, 2)
            for j in range(-1, N+1):
                y = round(-self.width/2.0 if j == -1 else self.width/2.0 if j == N else (-N/2.0 + 0.5 + j)*dl, 2)
                pts.append((x, y, z, -pi, False if j == -1 or j == N else True))
            # cara 3:
            y = round(self.width/2.0 + distance, 2)
            for i in range(-1, M+1):
                x = round(self.length/2.0 if i == -1 else -self.length/2.0 if i == M else (M/2.0 - 0.5 - i)*dl, 2)
                pts.append((x, y, z, -pi/2, False if i == -1 or i == M else True))
            # cara 4:
            x = round(-self.length/2.0 - distance, 2)
            for j in range(-1, N+1):
                y = round(self.width/2.0 if j == -1 else -self.width/2.0 if j == N else (N/2.0 - 0.5 - j)*dl, 2)
                pts.append((x, y, z, 0, False if j == -1 or j == N else True))
        return pts

    def top_floor(self, distance):
        dl = round(2 * distance * tan(radians(FOV/2)), 2)
        dh = round(dl / AR, 2)
        M = int(ceil(self.length / dl))
        N = int(ceil(self.width / dh))
        vel = 1
        z = round(self.height/2.0 + distance, 2)
        i = j = 0
        pts = [(round(-self.length/2.0 - distance, 2), round(-self.width/2.0, 2), z, pi/2, False)]
        while i >= 0 and i < M and j >= 0 and j < N:
            x = round((-M/2.0 + 0.5 + i)*dl, 2)
            y = round((-N/2.0 + 0.5 + j)*dh, 2)
            pts.append((x, y, z, pi/2, True))
            # movimiento horizontal
            if M >= N:
                if (i == M - 1 and vel == 1) or (i == 0 and vel == -1):
                    j += 1
                    vel *= -1
                else:
                    i += vel
            # movimiento vertical
            else:
                if (j == N - 1 and vel == 1) or (j == 0 and vel == -1):
                    i += 1
                    vel *= -1
                else:
                    j += vel
        return pts
    
    def complete_local_trajectory(self, distance, vel):
        pts = self.complete_viewpoints(distance)
        path = [pts[0]]
        prev_pt = None
        for pt in pts:
            if prev_pt is not None:
                if pt[3] == prev_pt[3] or (pt[0] == prev_pt[0] and pt[1] == prev_pt[1]):
                    path += straight_path_3d(prev_pt, pt, vel)
                else:
                    r = sqrt(((prev_pt[0] - pt[0])**2 + (prev_pt[1] - pt[1])**2)/2)
                    path += circle_arc_3d(r, prev_pt, pi/2, pt[2] - prev_pt[2], vel, False)
            prev_pt = pt
        return path
    
    def sdf_string(self):
        return f"<?xml version=\"1.0\" ?>\
        <sdf version=\"1.6\">\
            <model name=\"test_box\">\
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

    def transform_by_closest(self, pt):
        self.yaw = atan2(pt[1] - self.y, pt[0] - self.x)    

    def spiral(self, distance):
        dl = round(2 * distance * tan(radians(FOV/2)), 2)
        dh = round(dl / AR, 2)
        N = int(ceil(2*pi / min(2*atan(dl / (2*self.radius)), pi/2)))
        L = int(ceil(self.height / dh))
        pts = []
        for k in range(0, L):
            z = round((-L/2.0 + 0.5 + k)*dh, 2)
            for i in range(N):
                theta = clip_angle(2*pi / N * i)
                x = round((self.radius + distance)*cos(theta), 2)
                y = round((self.radius + distance)*sin(theta), 2)
                pts.append((x, y, z, clip_angle(theta + pi), True))         
        return pts

    def top_floor(self, distance):
        dl = round(2 * distance * tan(radians(FOV/2)), 2)
        dh = round(dl / AR, 2)
        N = int(ceil(2*self.radius / dh))
        z = round(self.height/2.0 + distance, 2)
        pts = [(self.radius + distance, 0, z, clip_angle(pi), False)]
        for i in range(N):
            x = round((N/2.0 - 0.5 - i)*dh, 2)
            l = 2*sqrt(self.radius**2 - x**2)
            M = int(ceil(l / dl))
            for j_aux in range(M):
                j = j_aux if i % 2 == 0 else M - 1 - j_aux
                y = round((-M/2.0 + 0.5 + j)*dl, 2)
                pts.append((x, y, z, clip_angle(pi), True))
        return pts
    
    def complete_local_trajectory(self, distance, vel):
        pts = self.complete_viewpoints(distance)
        path = [pts[0]]
        prev_pt = None
        for pt in pts:
            if prev_pt is not None:
                if pt[3] == prev_pt[3]:
                    path += straight_path_3d(prev_pt, pt, vel)
                else:
                    path += circle_arc_3d(self.radius + distance, prev_pt, clip_angle(pt[3] - prev_pt[3]), pt[2] - prev_pt[2], vel, pt[4])
            prev_pt = pt
        return path

    def sdf_string(self):
        return f"<?xml version=\"1.0\" ?>\
        <sdf version=\"1.6\">\
            <model name=\"test_box\">\
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


# Helper functions:

def straight_path_1d(x1, x2, N, angle=False):
    assert(N > 0)
    dx = ((x2 - x1) if not angle else clip_angle(x2 - x1)) / N
    path = []
    for i in range(N):
        path.append(x1 + (i+1)*dx if not angle else clip_angle(x1 + (i+1)*dx))
    return path


def straight_path_3d(p1, p2, vel, auto_heading=False):
    d = sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
    N = max(round(d / (vel * 0.1)), 1) # sample time fixed @ 100ms
    path_x = straight_path_1d(p1[0], p2[0], N)
    path_y = straight_path_1d(p1[1], p2[1], N)
    path_z = straight_path_1d(p1[2], p2[2], N)
    path_yaw = [atan2(p2[1]-p1[1], p2[0]-p1[0])]*N if auto_heading else straight_path_1d(p1[3], p2[3], N, angle=True)
    path = list(zip(path_x, path_y, path_z, path_yaw, [False]*(N-1)+[p2[4]]))
    return path


def circle_arc_3d(r, p1, d_theta, dz, vel, p2_viewpoint=False):
    d = sqrt((d_theta*r)**2 + dz**2)
    N = max(round(d / (vel * 0.1)), 1) # sample time fixed @ 100ms
    x0, y0, z0, yaw0, _ = p1
    # theta: angle from circle's origin (xc, yc) to drone (x, y):
    xc = x0 + r*cos(yaw0)
    yc = y0 + r*sin(yaw0)
    theta = clip_angle(yaw0+pi)
    path = []
    for i in range(N):
        theta = clip_angle(theta + d_theta/N)
        x = xc + r*cos(theta)
        y = yc + r*sin(theta)
        z = z0 + (i+1)*dz/N
        yaw = clip_angle(theta + pi)
        path.append((x, y, z, yaw, p2_viewpoint if i == N-1 else False))
    return path


def clip_angle(angle):
    if angle <= -pi:
        angle += 2*pi
    elif angle > pi:
        angle -= 2*pi
    return angle