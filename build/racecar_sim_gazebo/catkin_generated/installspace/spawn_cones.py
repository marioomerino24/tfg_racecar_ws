#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import math
import rospy
import rospkg

from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest, DeleteModelRequest
import tf.transformations as tft

def quat_from_yaw(yaw):
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

def unit(vx, vy):
    n = math.hypot(vx, vy)
    if n < 1e-9: return (1.0, 0.0)
    return (vx / n, vy / n)

def generate_centerline(pattern, N, DS, X0, Y0, YAW0, ARC_R, SL_A, SL_L, CH_DY, CH_M):
    pts = []   # [(x, y, yaw_t)]
    if pattern == 'corridor':
        # Tangente constante yaw = YAW0
        tx, ty = math.cos(YAW0), math.sin(YAW0)
        for i in range(N):
            x = X0 + i * DS * tx
            y = Y0 + i * DS * ty
            pts.append((x, y, YAW0))
    elif pattern == 'arc':
        # Centro del círculo a la izquierda de yaw0 si R>0
        # Centro = (Xc, Yc) = (X0, Y0) - R * n0   con n0 = rot90(t0)
        t0x, t0y = math.cos(0.0), math.sin(0.0)  # referencia de theta=0 a lo largo del +x
        # Usamos parámetro theta medido desde el punto inicial
        # Tangente en arc es yaw = theta + pi/2 (CCW)
        # Colocamos el primer punto en (X0, Y0) con theta=theta0
        # Elegimos theta0 tal que pos(theta0) = (X0, Y0) con centro (Xc,Yc)
        # Para simplificar: tomamos centro (Xc,Yc) = (X0, Y0 - ARC_R)
        Xc = X0
        Yc = Y0 - ARC_R
        # theta0 = pi/2 para que (X0, Y0) = (Xc + R*cos(theta0), Yc + R*sin(theta0))
        theta0 = math.pi/2.0
        dtheta = DS / max(1e-6, ARC_R)
        for i in range(N):
            theta = theta0 + i * dtheta
            x = Xc + ARC_R * math.cos(theta)
            y = Yc + ARC_R * math.sin(theta)
            yaw_t = theta + math.pi/2.0
            pts.append((x, y, yaw_t))
    elif pattern == 'slalom':
        # y(x) = Y0 + A*sin(2pi x / L), yaw = atan(dy/dx)
        k = 2.0 * math.pi / max(1e-6, SL_L)
        for i in range(N):
            x = X0 + i * DS
            y = Y0 + SL_A * math.sin(k * (x - X0))
            dy_dx = SL_A * k * math.cos(k * (x - X0))
            yaw_t = math.atan2(dy_dx, 1.0)
            pts.append((x, y, yaw_t))
    elif pattern == 'chicane':
        # Tramo 1: y = Y0 + CH_DY, Tramo 2: y = Y0 - CH_DY, resto Y0
        for i in range(N):
            x = X0 + i * DS
            if i < CH_M:
                y = Y0 + CH_DY
            elif i < 2 * CH_M:
                y = Y0 - CH_DY
            else:
                y = Y0
            yaw_t = YAW0
            pts.append((x, y, yaw_t))
    else:
        rospy.logwarn("pattern '%s' no reconocido; usando corridor", pattern)
        return generate_centerline('corridor', N, DS, X0, Y0, YAW0, ARC_R, SL_A, SL_L, CH_DY, CH_M)
    return pts

def main():
    rospy.init_node('spawn_cones')

    pattern = rospy.get_param("~pattern", "corridor")
    W       = float(rospy.get_param("~W", 3))
    DS      = float(rospy.get_param("~DS", 2.0))
    N       = int(rospy.get_param("~N", 8))
    X0      = float(rospy.get_param("~X0", 3.0))
    Y0      = float(rospy.get_param("~Y0", 0.0))
    YAW0    = float(rospy.get_param("~YAW0", 0.0))
    ARC_R   = float(rospy.get_param("~ARC_R", 12.0))
    SL_A    = float(rospy.get_param("~SL_A", 0.7))
    SL_L    = float(rospy.get_param("~SL_L", 8.0))
    CH_DY   = float(rospy.get_param("~CH_DY", 1.0))
    CH_M    = int(rospy.get_param("~CH_M", 4))
    clear   = bool(rospy.get_param("~clear_existing", True))
    prefix  = rospy.get_param("~prefix", "cone")

    # Espera a Gazebo
    rospy.loginfo("[spawn_cones] esperando servicios de Gazebo...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delcli = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    # Carga SDF del cono
    rp = rospkg.RosPack()
    sdf_path = os.path.join(rp.get_path("racecar_description"),
                             "models", "cone", "model.sdf")
    if not os.path.isfile(sdf_path):
        rospy.logfatal("No se encuentra %s", sdf_path)
        return
    with open(sdf_path, "r") as f:
        model_xml = f.read()

    # Opcional: borra conos existentes con este prefijo
    if clear:
        for side in ['L', 'R']:
            for i in range(N):
                name = "%s_%s_%d" % (prefix, side, i)
                try:
                    delcli(DeleteModelRequest(model_name=name))
                except Exception:
                    pass  # ignorar si no existe

    # Genera centro y normales
    center = generate_centerline(pattern, N, DS, X0, Y0, YAW0, ARC_R, SL_A, SL_L, CH_DY, CH_M)

    spawned = 0
    for i, (xc, yc, yaw_t) in enumerate(center):
        # Tangente (cos, sin), normal a la izquierda = rot90(t)
        tx, ty = math.cos(yaw_t), math.sin(yaw_t)
        nx, ny = -ty, tx
        nx, ny = unit(nx, ny)
        # Izquierda (+W/2) y derecha (-W/2)
        xL, yL = xc + 0.5 * W * nx, yc + 0.5 * W * ny
        xR, yR = xc - 0.5 * W * nx, yc - 0.5 * W * ny

        # Orientación de los conos: no es crítica; alineamos con yaw_t
        q = quat_from_yaw(yaw_t)

        # Spawnea L
        poseL = Pose(Point(xL, yL, 0.0), q)
        reqL = SpawnModelRequest(model_name="%s_L_%d" % (prefix, i),
                                 model_xml=model_xml,
                                 robot_namespace="",
                                 initial_pose=poseL,
                                 reference_frame="world")
        # Spawnea R
        poseR = Pose(Point(xR, yR, 0.0), q)
        reqR = SpawnModelRequest(model_name="%s_R_%d" % (prefix, i),
                                 model_xml=model_xml,
                                 robot_namespace="",
                                 initial_pose=poseR,
                                 reference_frame="world")
        try:
            spawn(reqL)
            spawn(reqR)
            spawned += 2
        except Exception as e:
            rospy.logwarn("Fallo al spawnear par %d: %s", i, str(e))

    rospy.loginfo("[spawn_cones] patrón=%s, N=%d pares, W=%.2f, DS=%.2f -> %d conos",
                  pattern, N, W, DS, spawned)

if __name__ == "__main__":
    main()
