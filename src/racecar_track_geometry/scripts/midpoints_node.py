#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from racecar_cone_msgs.msg import ConePairArray, MidpointArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class MidpointsNode:
    def __init__(self):
        # Publicador de la nube de puntos medios
        self.pub_midpoints = rospy.Publisher("/track/midpoints",
                                             MidpointArray,
                                             queue_size=1)
        # Publicador del marcador para RViz
        self.pub_marker = rospy.Publisher("/track/midpoints_marker",
                                          Marker,
                                          queue_size=1)

        # Tamaño de la "X" (longitud total de cada brazo en metros)
        self.marker_size = rospy.get_param("~marker_size", 0.4)

        rospy.Subscriber("/cones/pairs", ConePairArray, self.cb, queue_size=1)

    def cb(self, msg):
        # ==== Mensaje MidpointArray ====
        out = MidpointArray()
        out.header = msg.header
        out.points = []
        out.width = []
        out.cov_xy_flat = []
        out.id = []
        out.confidence = []

        for k, pr in enumerate(msg.pairs):
            out.points.append(pr.midpoint)
            out.width.append(pr.width)

            # cov midpoint = 1/4 (SigmaL + SigmaR)
            cov = [0.25*(pr.left.cov_xy[0]+pr.right.cov_xy[0]),
                   0.25*(pr.left.cov_xy[1]+pr.right.cov_xy[1]),
                   0.25*(pr.left.cov_xy[2]+pr.right.cov_xy[2]),
                   0.25*(pr.left.cov_xy[3]+pr.right.cov_xy[3])]
            out.cov_xy_flat.extend(cov)

            out.id.append(k)
            out.confidence.append(1.0)

        self.pub_midpoints.publish(out)

        # ==== Marker para RViz (X verde en cada midpoint) ====
        marker = Marker()
        marker.header = msg.header
        marker.ns = "track_midpoints_x"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # El marker está ya en coordenadas del frame de los puntos
        marker.pose.orientation.w = 1.0  # identidad
        # Grosor de las líneas (en m)
        marker.scale.x = 0.03

        # Color verde opaco
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Sin expiración (lifetime=0) -> se actualiza cada callback
        marker.lifetime = rospy.Duration(0.0)

        marker.points = []

        # Semi-longitud de los brazos de la X
        half_len = 0.5 * self.marker_size

        for pt in out.points:
            # Primera diagonal: (-,-) -> (+,+)
            p1 = Point()
            p1.x = pt.x - half_len
            p1.y = pt.y - half_len
            p1.z = pt.z

            p2 = Point()
            p2.x = pt.x + half_len
            p2.y = pt.y + half_len
            p2.z = pt.z

            # Segunda diagonal: (-,+) -> (+,-)
            p3 = Point()
            p3.x = pt.x - half_len
            p3.y = pt.y + half_len
            p3.z = pt.z

            p4 = Point()
            p4.x = pt.x + half_len
            p4.y = pt.y - half_len
            p4.z = pt.z

            # LINE_LIST: cada par consecutivo de puntos es un segmento
            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p3)
            marker.points.append(p4)

        self.pub_marker.publish(marker)


if __name__=="__main__":
    rospy.init_node("midpoints")
    MidpointsNode()
    rospy.spin()
