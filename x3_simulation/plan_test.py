from planning import PlanePlanner
from math import pi

box = PlanePlanner(7, 3, 0, 0, 5, yaw=pi/3, roll=pi/12)
# Se tranforma el edificio para que el punto inicial de la trayectoria este cerca al origen
box.transform_by_closest((0, 0, 0))
# Se planean los puntos de la ruta en marco local(dada la distancia)
box.complete_keypoints(1.0)
# Se agregan los puntos del despegue del dron
box.prepend_point((0,0,1,0), (0,0,0,0))
box.prepend_point((0,0,0,0), (0,0,0,0))
# Se planea la trayectoria completa en marco local (dada la velocidad y factor de suavizado k)
box.complete_world_trajectory(0.25, k=0.5)
path = box.get_trajectory()

for pt in path:
    print(pt)