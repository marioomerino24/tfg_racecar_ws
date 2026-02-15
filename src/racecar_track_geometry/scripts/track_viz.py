#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Visualizador unificado de geometría de pista
# ROS1 Melodic (Python 2)

import math
import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

from racecar_cone_msgs.msg import ConeArray, ConePairArray, MidpointArray, Centerline

def color(r, g, b, a=1.0):
    return ColorRGBA(r, g, b, a)

class TrackViz(object):
    def __init__(self):
        p = rospy.get_param

        # ====================== Parámetros generales ======================
        self.frame     = p("~frame_id", "odom")          # fixed frame para RViz
        self.ns        = p("~ns", "track")               # namespace lógico interno
        self.rate_hz   = float(p("~rate_hz", 20.0))      # Hz de refresco
        self.life_time = float(p("~lifetime", 0.15))     # tiempo de vida de markers (s)

        # ====================== Tópicos de IO ======================
        # Entradas de percepción/estimación vigentes
        self.topic_cones      = p("~topic/cones",      "/perception/lidar/cones")
        self.topic_pairs      = p("~topic/pairs",      "/deprecated/lr_pairs")
        self.topic_midpoints  = p("~topic/midpoints",  "/estimation/track/midpoints")
        self.topic_centerline = p("~topic/centerline", "/estimation/track/centerline")

        # Entradas nuevas desde nodos posteriores (centerline, borders, etc.)
        self.topic_ctr_path_in = p("~topic/centerline_path_in",
                                   "/planning/track/centerline_path")
        self.topic_left_path   = p("~topic/left_path",
                                   "/planning/track/left_boundary_path")
        self.topic_right_path  = p("~topic/right_path",
                                   "/planning/track/right_boundary_path")

        # Salidas (relativos al nombre del nodo)
        self.topic_markers = p("~topic/markers", "markers")

        # Mantengo compatibilidad con tu antiguo parámetro ~topic/centerline_path
        topic_ctr_path_legacy = p("~topic/centerline_path", "/planning/track/centerline_path")
        self.topic_ctr_path_out = p("~topic/centerline_path_out",
                                    topic_ctr_path_legacy)

        # ====================== Flags de activación ======================
        self.enable_cones           = bool(p("~enable/cones",           True))
        self.enable_pairs           = bool(p("~enable/pairs",           False))
        self.enable_midpoints       = bool(p("~enable/midpoints",       True))
        self.enable_centerline      = bool(p("~enable/centerline",      False))  # legacy
        self.enable_centerline_path = bool(p("~enable/centerline_path", True))   # salida Path

        # Nuevos flags (nodos posteriores)
        self.enable_centerline_path_in = bool(p("~enable/centerline_path_in", True))
        self.enable_border_paths       = bool(p("~enable/border_paths",       True))

        # ====================== Escalas geométricas ======================
        # Tamaños básicos
        self.scale_cone    = float(p("~scale/cone",              0.25))
        self.scale_mid     = float(p("~scale/midpoint",          0.15))
        self.line_w_pair   = float(p("~scale/pair_width",        0.05))
        self.line_w_center = float(p("~scale/centerline_width",  0.05))
        self.cross_size    = float(p("~scale/centerline_cross",  0.4))
        self.cross_width   = float(p("~scale/cross_width",       0.05))  # grosor de la X
        # Bordes L/R (Path nuevos)
        self.line_w_borders = float(p("~scale/borders_width",    0.05))

        # Midpoints: altura Z
        self.mid_z = float(p("~midpoints/z", 0.0))

        # Cruces: altura Z
        self.cross_z = float(p("~cross/z", 0.0))

        # Parámetros de anillos/texto de pares
        self.ring_radius    = float(p("~pairs/ring_radius",    0.22))
        self.ring_z         = float(p("~pairs/ring_z",         0.05))
        self.text_z         = float(p("~pairs/text_z",         0.25))
        self.text_scale     = float(p("~pairs/text_scale",     0.15))
        self.ring_width     = float(p("~pairs/ring_width",     0.03))
        self.num_circle_pts = int(  p("~pairs/num_circle_pts", 36))

        # Texto con índice original del ConeArray
        self.cone_idx_text_z     = float(p("~cones/idx_text_z",     0.35))
        self.cone_idx_text_scale = float(p("~cones/idx_text_scale", 0.15))
        # Offset lateral en XY para que el texto no quede justo encima del cono
        self.cone_idx_offset_x   = float(p("~cones/idx_offset_x",   0.40))
        self.cone_idx_offset_y   = float(p("~cones/idx_offset_y",   0.40))

        # ====================== Colores ======================
        # Izquierda: azul
        c = p("~color/left",        [0.0, 0.6, 1.0]); self.c_left   = color(c[0], c[1], c[2], 1.0)
        # Derecha: amarillo
        c = p("~color/right",       [1.0, 0.8, 0.0]); self.c_right  = color(c[0], c[1], c[2], 1.0)
        # Segmentos entre pares: gris
        c = p("~color/pairs",       [0.6, 0.6, 0.6]); self.c_pairs  = color(c[0], c[1], c[2], 0.9)
        # Midpoints: verde
        c = p("~color/mid",         [0.0, 1.0, 0.0]); self.c_mid    = color(c[0], c[1], c[2], 1.0)
        # Centerline final: magenta
        c = p("~color/centerline",  [1.0, 0.0, 1.0]); self.c_ctr    = color(c[0], c[1], c[2], 1.0)
        # Cruces: blanco
        c = p("~color/cross",       [1.0, 1.0, 1.0]); self.c_cross  = color(c[0], c[1], c[2], 1.0)

        # ====================== Publicadores / Subs ======================
        self.pub_markers  = rospy.Publisher(self.topic_markers,
                                            MarkerArray, queue_size=1)
        self.pub_ctr_path = rospy.Publisher(self.topic_ctr_path_out,
                                            Path, queue_size=1)

        # cachés de mensajes
        self.msg_cones       = None
        self.msg_pairs       = None
        self.msg_mid         = None
        self.msg_ctr         = None          # Centerline (legacy)
        self.msg_ctr_path_in = None          # Path centerline (nodo posterior)
        self.msg_left_path   = None          # Path borde izquierdo
        self.msg_right_path  = None          # Path borde derecho

        # Subs
        rospy.Subscriber(self.topic_cones,      ConeArray,     self._cb_cones, queue_size=1)
        if self.enable_pairs:
            rospy.Subscriber(self.topic_pairs, ConePairArray, self._cb_pairs, queue_size=1)
        rospy.Subscriber(self.topic_midpoints,  MidpointArray, self._cb_mid,   queue_size=1)
        rospy.Subscriber(self.topic_centerline, Centerline,    self._cb_ctr,   queue_size=1)

        # Subs nuevos (nodos posteriores)
        rospy.Subscriber(self.topic_ctr_path_in, Path, self._cb_ctr_path_in, queue_size=1)
        rospy.Subscriber(self.topic_left_path,   Path, self._cb_left_path,   queue_size=1)
        rospy.Subscriber(self.topic_right_path,  Path, self._cb_right_path,  queue_size=1)

        self._timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self._on_timer)

    # === Callbacks: solo cachean último mensaje ===
    def _cb_cones(self, msg):       self.msg_cones       = msg
    def _cb_pairs(self, msg):       self.msg_pairs       = msg
    def _cb_mid(self,   msg):       self.msg_mid         = msg
    def _cb_ctr(self,   msg):       self.msg_ctr         = msg
    def _cb_ctr_path_in(self, msg): self.msg_ctr_path_in = msg
    def _cb_left_path(self,   msg): self.msg_left_path   = msg
    def _cb_right_path(self,  msg): self.msg_right_path  = msg

    # === Utilidad interna: círculo discreto para anillos ===
    def _circle_points(self, x, y, z, r, n):
        pts = []
        for k in range(n + 1):  # +1 para cerrar el anillo
            th = 2.0 * math.pi * float(k) / float(n)
            px = x + r * math.cos(th)
            py = y + r * math.sin(th)
            pts.append(Point(px, py, z))
        return pts

    # === Timer: publica MarkerArray + Path ===
    def _on_timer(self, evt):
        ma = MarkerArray()
        now = rospy.Time.now()
        hdr = Header(stamp=now, frame_id=self.frame)
        ns  = self.ns
        uid = 0

        # ---------- Conos detectados (ConeArray) ----------
        if self.enable_cones and self.msg_cones is not None and self.msg_cones.cones:
            pts_cones = []
            for c in self.msg_cones.cones:
                if hasattr(c, "p"):
                    pts_cones.append(Point(c.p.x, c.p.y, 0.0))

            if pts_cones:
                m = Marker(header=hdr, ns=ns+"/cones", id=uid,
                           type=Marker.SPHERE_LIST, action=Marker.ADD)
                m.scale.x = m.scale.y = m.scale.z = self.scale_cone
                m.color = self.c_right
                m.lifetime = rospy.Duration(self.life_time)
                m.points = pts_cones
                ma.markers.append(m); uid += 1

        # ---------- Índice original de cada cono (ConeArray) ----------
        if self.msg_cones is not None and self.msg_cones.cones:
            for i, c in enumerate(self.msg_cones.cones):
                if not hasattr(c, "p"):
                    continue
                p = c.p
                mt = Marker()
                mt.header = hdr
                mt.ns = ns + "/cones_idx"
                mt.id = uid
                mt.type = Marker.TEXT_VIEW_FACING
                mt.action = Marker.ADD

                # Offset lateral fijo en el frame de RViz
                mt.pose.position.x = p.x + self.cone_idx_offset_x
                mt.pose.position.y = p.y + self.cone_idx_offset_y
                mt.pose.position.z = self.cone_idx_text_z
                mt.pose.orientation.w = 1.0

                mt.scale.z = self.cone_idx_text_scale
                mt.color.r = 0.0
                mt.color.g = 1.0
                mt.color.b = 0.0
                mt.color.a = 1.0

                mt.text = str(i)  # índice del ConeArray

                mt.lifetime = rospy.Duration(self.life_time)
                ma.markers.append(mt)
                uid += 1

        # ---------- Pares: segmentos + anillos + texto ----------
        if self.enable_pairs and self.msg_pairs is not None and self.msg_pairs.pairs:
            # Segmentos entre conos L/R
            m_seg = Marker(header=hdr, ns=ns+"/pairs_segments", id=uid,
                           type=Marker.LINE_LIST, action=Marker.ADD)
            m_seg.scale.x = self.line_w_pair
            m_seg.color   = self.c_pairs
            m_seg.lifetime = rospy.Duration(self.life_time)

            for pair in self.msg_pairs.pairs:
                Lp = pair.left.p
                Rp = pair.right.p
                m_seg.points.append(Point(Lp.x, Lp.y, 0.0))
                m_seg.points.append(Point(Rp.x, Rp.y, 0.0))

            ma.markers.append(m_seg); uid += 1

            # Anillos + texto sobre cada cono
            for i, pair in enumerate(self.msg_pairs.pairs):
                Lp = pair.left.p
                Rp = pair.right.p

                # En modo legacy con pares, left.id/right.id suele codificar el indice de pareja.
                idx_label = getattr(pair.left, "id", i + 1)
                num_str   = str(idx_label)

                # Texto blanco (sobre cada cono)
                for P in (Lp, Rp):
                    mt = Marker()
                    mt.header = hdr
                    mt.ns = ns + "/pair_text"
                    mt.id = uid
                    mt.type = Marker.TEXT_VIEW_FACING
                    mt.action = Marker.ADD
                    mt.pose.position.x = P.x
                    mt.pose.position.y = P.y
                    mt.pose.position.z = self.text_z
                    mt.pose.orientation.w = 1.0
                    mt.scale.z = self.text_scale
                    mt.color.r = 1.0
                    mt.color.g = 1.0
                    mt.color.b = 1.0
                    mt.color.a = 1.0
                    mt.text = num_str
                    mt.lifetime = rospy.Duration(self.life_time)
                    ma.markers.append(mt); uid += 1

                # Anillo L (amarillo)
                mrL = Marker()
                mrL.header = hdr
                mrL.ns = ns + "/pair_ring_left"
                mrL.id = uid
                mrL.type = Marker.LINE_STRIP
                mrL.action = Marker.ADD
                mrL.pose.orientation.w = 1.0
                mrL.scale.x = self.ring_width
                mrL.color.r, mrL.color.g, mrL.color.b, mrL.color.a = 1.0, 1.0, 0.0, 1.0
                mrL.points = self._circle_points(Lp.x, Lp.y, self.ring_z,
                                                 self.ring_radius, self.num_circle_pts)
                mrL.lifetime = rospy.Duration(self.life_time)
                ma.markers.append(mrL); uid += 1

                # Anillo R (azul)
                mrR = Marker()
                mrR.header = hdr
                mrR.ns = ns + "/pair_ring_right"
                mrR.id = uid
                mrR.type = Marker.LINE_STRIP
                mrR.action = Marker.ADD
                mrR.pose.orientation.w = 1.0
                mrR.scale.x = self.ring_width
                mrR.color.r, mrR.color.g, mrR.color.b, mrR.color.a = 0.0, 0.0, 1.0, 1.0
                mrR.points = self._circle_points(Rp.x, Rp.y, self.ring_z,
                                                 self.ring_radius, self.num_circle_pts)
                mrR.lifetime = rospy.Duration(self.life_time)
                ma.markers.append(mrR); uid += 1

        # ---------- Midpoints (legacy, nodo posterior) ----------
        if self.enable_midpoints and self.msg_mid is not None and self.msg_mid.points:
            m = Marker(header=hdr, ns=ns+"/midpoints", id=uid,
                       type=Marker.SPHERE_LIST, action=Marker.ADD)
            m.scale.x = m.scale.y = m.scale.z = self.scale_mid
            m.color = self.c_mid
            m.lifetime = rospy.Duration(self.life_time)
            m.points = [Point(p.x, p.y, self.mid_z) for p in self.msg_mid.points]
            ma.markers.append(m); uid += 1

        # ---------- Centerline: línea + cruces + Path (legacy Centerline msg) ----------
        if self.enable_centerline and self.msg_ctr is not None:
            pts = getattr(self.msg_ctr, "points", None)
            if not pts:
                pts = getattr(self.msg_ctr, "samples", [])

            if pts and len(pts) >= 2:
                # Línea de centerline (legacy)
                line = Marker(header=hdr, ns=ns+"/centerline_legacy", id=uid,
                              type=Marker.LINE_STRIP, action=Marker.ADD)
                line.scale.x = self.line_w_center
                line.color   = self.c_ctr
                line.lifetime = rospy.Duration(self.life_time)
                line.points = [Point(p.x, p.y, 0.0) for p in pts]
                ma.markers.append(line); uid += 1

                # Cruces en cada nodo
                cross = Marker(header=hdr, ns=ns+"/centerline_nodes_legacy", id=uid,
                               type=Marker.LINE_LIST, action=Marker.ADD)
                cross.scale.x = self.cross_width
                cross.color   = self.c_cross
                cross.lifetime = rospy.Duration(self.life_time)
                cross.points = []
                half = 0.5 * self.cross_size

                for p in pts:
                    zc = self.cross_z
                    p1 = Point(p.x - half, p.y - half, zc)
                    p2 = Point(p.x + half, p.y + half, zc)
                    p3 = Point(p.x - half, p.y + half, zc)
                    p4 = Point(p.x + half, p.y - half, zc)
                    cross.points.append(p1); cross.points.append(p2)
                    cross.points.append(p3); cross.points.append(p4)

                ma.markers.append(cross); uid += 1

        # ---------- Centerline Path (nodo posterior, Path) ----------
        if (self.enable_centerline_path_in and
                self.msg_ctr_path_in is not None and
                self.msg_ctr_path_in.poses):

            # Línea de centerline (Path)
            m_line = Marker(header=hdr, ns=ns+"/centerline_path", id=uid,
                            type=Marker.LINE_STRIP, action=Marker.ADD)
            m_line.scale.x = self.line_w_center
            m_line.color   = self.c_ctr
            m_line.lifetime = rospy.Duration(self.life_time)
            m_line.points = []

            # Cruces en cada nodo (sobre Path)
            m_cross = Marker(header=hdr, ns=ns+"/centerline_nodes", id=uid+1,
                             type=Marker.LINE_LIST, action=Marker.ADD)
            m_cross.scale.x = self.cross_width
            m_cross.color   = self.c_cross
            m_cross.lifetime = rospy.Duration(self.life_time)
            m_cross.points = []
            half = 0.5 * self.cross_size

            for ps in self.msg_ctr_path_in.poses:
                p = ps.pose.position
                m_line.points.append(Point(p.x, p.y, 0.0))

                zc = self.cross_z
                p1 = Point(p.x - half, p.y - half, zc)
                p2 = Point(p.x + half, p.y + half, zc)
                p3 = Point(p.x - half, p.y + half, zc)
                p4 = Point(p.x + half, p.y - half, zc)
                m_cross.points.append(p1); m_cross.points.append(p2)
                m_cross.points.append(p3); m_cross.points.append(p4)

            ma.markers.append(m_line); uid += 2   # consumimos 2 ids (line + cross)
            ma.markers.append(m_cross)

            # Re-publicar Path (si quieres que otros nodos lo consuman desde aquí)
            if self.enable_centerline_path:
                path_out = self.msg_ctr_path_in
                path_out.header.stamp = now
                path_out.header.frame_id = self.frame
                self.pub_ctr_path.publish(path_out)

        # ---------- Bordes L/R como Path (nodos posteriores) ----------
        if self.enable_border_paths:
            # Borde izquierdo
            if self.msg_left_path is not None and self.msg_left_path.poses:
                mL = Marker(header=hdr, ns=ns+"/border_left", id=uid+10,
                            type=Marker.LINE_STRIP, action=Marker.ADD)
                mL.scale.x = self.line_w_borders
                mL.color   = self.c_left
                mL.lifetime = rospy.Duration(self.life_time)
                for ps in self.msg_left_path.poses:
                    p = ps.pose.position
                    mL.points.append(Point(p.x, p.y, 0.0))
                ma.markers.append(mL)

            # Borde derecho
            if self.msg_right_path is not None and self.msg_right_path.poses:
                mR = Marker(header=hdr, ns=ns+"/border_right", id=uid+11,
                            type=Marker.LINE_STRIP, action=Marker.ADD)
                mR.scale.x = self.line_w_borders
                mR.color   = self.c_right
                mR.lifetime = rospy.Duration(self.life_time)
                for ps in self.msg_right_path.poses:
                    p = ps.pose.position
                    mR.points.append(Point(p.x, p.y, 0.0))
                ma.markers.append(mR)

        # Publicar markers
        if ma.markers:
            self.pub_markers.publish(ma)


def main():
    rospy.init_node("track_viz")
    TrackViz()
    rospy.spin()


if __name__ == "__main__":
    main()
