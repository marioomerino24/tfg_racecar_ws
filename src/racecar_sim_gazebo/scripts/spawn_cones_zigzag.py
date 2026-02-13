#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import math
import rospy
import rospkg

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from nav_msgs.msg import Path
import tf.transformations as tft

# Mensajes de conos
from racecar_cone_msgs.msg import Cone, ConeArray


def quat_from_yaw(yaw):
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])


def unit(vx, vy):
    n = math.hypot(vx, vy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (vx / n, vy / n)


def generate_centerline_zigzag(n_arcs, ds, x0, y0, yaw0, R):
    """
    Genera una traza media formada por n_arcs semicircunferencias concatenadas.
    - Curvatura constante kappa = ±1/R en cada arco.
    - Los arcos alternan signo de curvatura (izquierda/derecha) para formar el zig-zag.
    - El avance entre muestras a lo largo de la traza es aproximadamente ds.
    """
    pts = []  # lista de (x, y, yaw_t)

    # Estado inicial del primer arco
    x = x0
    y = y0
    yaw = yaw0

    for j in range(n_arcs):
        # Alterna el signo de curvatura: +1 (izquierda), -1 (derecha)
        sig = 1.0 if (j % 2 == 0) else -1.0

        # Longitud de un semicirculo de radio R
        L_arc = math.pi * R

        # Número de pasos para aproximar el arco, tal que ds_real ~ ds
        n_steps = max(1, int(round(L_arc / ds)))
        ds_real = L_arc / float(n_steps)

        # Guardamos el estado inicial del arco j
        x0_arc = x
        y0_arc = y
        yaw0_arc = yaw

        # Recorremos el arco:
        # s va de 0 a L_arc, muestreado con ds_real
        for k in range(n_steps + 1):
            s = k * ds_real
            if s > L_arc:
                s = L_arc

            # Estado sobre el arco en función de s (solución exacta para curvatura constante)
            yaw_s = yaw0_arc + sig * s / R

            # Fórmulas de integración cerrada:
            # x(s) = x0 + (R/sig) * [sin(yaw0 + sig*s/R) - sin(yaw0)]
            # y(s) = y0 + (R/sig) * [cos(yaw0) - cos(yaw0 + sig*s/R)]
            sin_yaw0 = math.sin(yaw0_arc)
            cos_yaw0 = math.cos(yaw0_arc)
            sin_yaw_s = math.sin(yaw_s)
            cos_yaw_s = math.cos(yaw_s)

            x_s = x0_arc + (R / sig) * (sin_yaw_s - sin_yaw0)
            y_s = y0_arc + (R / sig) * (cos_yaw0 - cos_yaw_s)

            # Evitamos duplicar el punto de unión entre arcos:
            # - En el primer arco guardamos desde s=0
            # - En los siguientes, ignoramos el primer punto (s=0), que coincide con final del arco anterior
            if j == 0 or k > 0:
                pts.append((x_s, y_s, yaw_s))

            if s >= L_arc:
                break

        # Actualizamos el estado final para el siguiente arco
        s_end = L_arc
        yaw_end = yaw0_arc + sig * s_end / R
        sin_yaw_end = math.sin(yaw_end)
        cos_yaw_end = math.cos(yaw_end)

        x = x0_arc + (R / sig) * (sin_yaw_end - sin_yaw0)
        y = y0_arc + (R / sig) * (cos_yaw0 - cos_yaw_end)
        yaw = yaw_end

    return pts


def main():
    rospy.init_node('spawn_cones_zigzag')

    # === Parámetros ajustables vía ROS ===
    # --- Geometría de la pista / conos ---
    W        = float(rospy.get_param("~W", 3.0))       # ancho entre conos L/R (m)
    DS       = float(rospy.get_param("~DS", 1.0))      # separación aprox. entre parejas de conos (m)
    N_ARCS   = int(rospy.get_param("~N_arcs", 6))      # número de semicircunferencias
    R_ARC    = float(rospy.get_param("~R_arc", 4.0))   # radio de cada semicircunferencia (m)

    # Número de parejas de conos a publicar/spawnear (total conos = 2*N)
    N_pairs  = int(rospy.get_param("~N", 20))

    # Pose inicial de la traza media
    X0       = float(rospy.get_param("~X0", 0.0))
    Y0       = float(rospy.get_param("~Y0", 0.0))
    YAW0     = float(rospy.get_param("~YAW0", 0.0))

    # Prefijo para nombrar los conos
    prefix   = rospy.get_param("~prefix", "cone_zigzag")

    # --- Publicación de la traza media para el nodo de set_model_state ---
    centerline_ds     = float(rospy.get_param("~centerline_ds", 0.1))   # resolución [m] a lo largo de la traza
    centerline_topic  = rospy.get_param("~centerline_topic",
                                        "/track/centerline_zigzag")     # topic para nav_msgs/Path
    frame_id          = rospy.get_param("~frame_id", "world")

    # Topic para publicar el ConeArray
    cones_topic       = rospy.get_param("~cones_topic",
                                        "/track/cones_zigzag_cones")

    # Publisher de la traza (latched para que nuevos subscritores reciban automáticamente)
    centerline_pub = rospy.Publisher(centerline_topic, Path,
                                     queue_size=1, latch=True)

    # Publisher de conos (latched)
    cones_pub = rospy.Publisher(cones_topic, ConeArray,
                                queue_size=1, latch=True)

    # Espera a servicio de Gazebo
    rospy.loginfo("[spawn_cones_zigzag] esperando servicio /gazebo/spawn_sdf_model ...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn  = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Carga SDF del cono
    rp = rospkg.RosPack()
    sdf_path = os.path.join(rp.get_path("racecar_description"),
                            "models", "cone", "model.sdf")
    if not os.path.isfile(sdf_path):
        rospy.logfatal("No se encuentra el modelo SDF del cono: %s", sdf_path)
        return

    with open(sdf_path, "r") as f:
        model_xml = f.read()

    # === 1) Genera la traza media para los CONOS (resolución DS) ===
    center_for_cones = generate_centerline_zigzag(N_ARCS, DS, X0, Y0, YAW0, R_ARC)

    # === 2) Genera la traza media para el MOVIMIENTO del coche (resolución centerline_ds) ===
    center_for_motion = generate_centerline_zigzag(N_ARCS, centerline_ds,
                                                   X0, Y0, YAW0, R_ARC)

    # === Construye ConeArray ===
    cones_msg = ConeArray()
    cones_msg.header.stamp = rospy.Time.now()
    cones_msg.header.frame_id = frame_id

    # === Spawnea conos a partir de center_for_cones (solo primeras N_pairs parejas) ===
    spawned = 0
    max_pairs = min(N_pairs, len(center_for_cones))
    rospy.loginfo("[spawn_cones_zigzag] Se usarán %d parejas de conos (petición N=%d, disponibles=%d)",
                  max_pairs, N_pairs, len(center_for_cones))

    for i, (xc, yc, yaw_t) in enumerate(center_for_cones):
        if i >= max_pairs:
            break

        # Vector tangente (cos, sin) y normal a la izquierda
        tx, ty = math.cos(yaw_t), math.sin(yaw_t)
        nx, ny = -ty, tx
        nx, ny = unit(nx, ny)

        # Posición de conos izquierdo y derecho
        xL, yL = xc + 0.5 * W * nx, yc + 0.5 * W * ny
        xR, yR = xc - 0.5 * W * nx, yc - 0.5 * W * ny

        q = quat_from_yaw(yaw_t)

        # Cono izquierdo
        poseL = Pose(Point(xL, yL, 0.0), q)
        reqL = SpawnModelRequest(model_name="%s_L_%d" % (prefix, i),
                                 model_xml=model_xml,
                                 robot_namespace="",
                                 initial_pose=poseL,
                                 reference_frame=frame_id)

        # Cono derecho
        poseR = Pose(Point(xR, yR, 0.0), q)
        reqR = SpawnModelRequest(model_name="%s_R_%d" % (prefix, i),
                                 model_xml=model_xml,
                                 robot_namespace="",
                                 initial_pose=poseR,
                                 reference_frame=frame_id)

        # Mensajes Cone correspondientes
        cone_L = Cone()
        cone_L.header = cones_msg.header
        cone_L.p.x = xL
        cone_L.p.y = yL
        cone_L.p.z = 0.0
        # Covarianza "ideal" pequeña (puedes cambiarlo o dejarlo a ceros si quieres)
        cone_L.cov_xy = [0.0, 0.0, 0.0, 0.0]
        cone_L.confidence = 1.0
        cone_L.source = "spawn"
        cone_L.id = int(2 * i)  # opcional

        cone_R = Cone()
        cone_R.header = cones_msg.header
        cone_R.p.x = xR
        cone_R.p.y = yR
        cone_R.p.z = 0.0
        cone_R.cov_xy = [0.0, 0.0, 0.0, 0.0]
        cone_R.confidence = 1.0
        cone_R.source = "spawn"
        cone_R.id = int(2 * i + 1)

        cones_msg.cones.append(cone_L)
        cones_msg.cones.append(cone_R)

        try:
            spawn(reqL)
            spawn(reqR)
            spawned += 2
        except Exception as e:
            rospy.logwarn("Fallo al spawnear par %d: %s", i, str(e))

    rospy.loginfo(
        "[spawn_cones_zigzag] N_arcs=%d, R_arc=%.2f, W=%.2f, DS=%.2f, N=%d -> %d conos spawneados",
        N_ARCS, R_ARC, W, DS, max_pairs, spawned
    )

    # Publica el ConeArray (latched)
    rospy.sleep(0.5)
    cones_pub.publish(cones_msg)
    rospy.loginfo("[spawn_cones_zigzag] Publicado ConeArray con %d conos en %s (frame_id=%s)",
                  len(cones_msg.cones), cones_topic, frame_id)

    # === Construye y publica la traza media como nav_msgs/Path ===
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = frame_id

    for (x_m, y_m, yaw_m) in center_for_motion:
        ps = PoseStamped()
        ps.header = path_msg.header
        ps.pose.position.x = x_m
        ps.pose.position.y = y_m
        ps.pose.position.z = 0.0
        ps.pose.orientation = quat_from_yaw(yaw_m)
        path_msg.poses.append(ps)

    rospy.loginfo("[spawn_cones_zigzag] Trayectoria media: %d muestras, ds=%.3f m",
                  len(path_msg.poses), centerline_ds)

    # Publicación (latched)
    rospy.sleep(0.5)
    centerline_pub.publish(path_msg)
    rospy.loginfo("[spawn_cones_zigzag] Publicado Path en %s (frame_id=%s)",
                  centerline_topic, frame_id)

    # Mantener el nodo vivo para que el latched topic siga disponible
    rospy.loginfo("[spawn_cones_zigzag] Nodo listo. Ctrl+C para salir.")
    rospy.spin()


if __name__ == "__main__":
    main()
