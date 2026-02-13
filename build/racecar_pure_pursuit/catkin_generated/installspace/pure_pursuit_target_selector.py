#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
import tf2_ros
import tf2_geometry_msgs

def _p2(x, y): return (x, y)
def _sub(a, b): return (a[0]-b[0], a[1]-b[1])
def _add(a, b): return (a[0]+b[0], a[1]+b[1])
def _dot(a, b): return a[0]*b[0] + a[1]*b[1]
def _norm2(a): return math.hypot(a[0], a[1])
def _scale(a, s): return (a[0]*s, a[1]*s)
def _clamp(x, lo, hi): return max(lo, min(hi, x))

class PurePursuitTargetSelector(object):
    def __init__(self):
        # Params
        self.rate_hz = rospy.get_param("~rate_hz", 40)
        self.odom_topic = rospy.get_param("~odom_topic", "/vesc/odom")
        self.path_topic = rospy.get_param("~path_topic", "/track/centerline")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")

        self.L = float(rospy.get_param("~wheelbase_L", 0.32))
        self.delta_max = math.radians(float(rospy.get_param("~delta_max_deg", 28.0)))

        self.mode = rospy.get_param("~lookahead_mode", "hybrid")
        self.L0 = float(rospy.get_param("~L0", 0.8))
        self.Lmin = float(rospy.get_param("~Lmin", 0.4))
        self.Lmax = float(rospy.get_param("~Lmax", 2.5))
        self.kv = float(rospy.get_param("~kv", 0.35))
        self.kk = float(rospy.get_param("~kk", 0.6))
        self.eps_kappa = float(rospy.get_param("~eps_kappa", 0.05))
        self.curv_window_m = float(rospy.get_param("~curv_window_m", 0.8))

        self.publish_ackermann = bool(rospy.get_param("~publish_ackermann", True))
        self.ack_topic = rospy.get_param("~ackermann_topic", "/pure_pursuit/drive")
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))

        # State
        self.path_pts = []   # [(x,y), ...] en odom
        self.s_cum = []      # arclength acumulada (m)
        self.path_stamp = rospy.Time(0)

        self.odom = None     # último Odometry
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # IO
        rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom, queue_size=10)
        rospy.Subscriber(self.path_topic, Path, self._cb_path, queue_size=1)

        self.pub_target_odom = rospy.Publisher("~target_point_odom", PointStamped, queue_size=1)
        self.pub_target_bl   = rospy.Publisher("~target_point_base_link", PointStamped, queue_size=1)
        self.pub_ld          = rospy.Publisher("~Ld", Float64, queue_size=1)
        self.pub_delta       = rospy.Publisher("~delta", Float64, queue_size=1)
        self.pub_kappa       = rospy.Publisher("~kappa", Float64, queue_size=1)
        self.pub_marker      = rospy.Publisher("~markers", Marker, queue_size=1)
        self.pub_ack         = rospy.Publisher(self.ack_topic, AckermannDriveStamped, queue_size=1) if self.publish_ackermann else None

        self.timer = rospy.Timer(rospy.Duration(1.0/float(self.rate_hz)), self._on_timer)

        rospy.loginfo("[pure_pursuit] ready: mode=%s, L0=%.3f Lmin=%.3f Lmax=%.3f kv=%.3f kk=%.3f" %
                      (self.mode, self.L0, self.Lmin, self.Lmax, self.kv, self.kk))

    def _cb_odom(self, msg):
        self.odom = msg

    def _cb_path(self, msg):
        # Esperamos nav_msgs/Path (preferible desde tu resampler uniforme).
        pts = []
        for ps in msg.poses:
            pts.append(_p2(ps.pose.position.x, ps.pose.position.y))
        if len(pts) >= 2:
            self.path_pts = pts
            self.s_cum = self._build_arclength(pts)
            self.path_stamp = msg.header.stamp
        else:
            rospy.logwarn_throttle(2.0, "[pure_pursuit] Path con <2 puntos; ignorado.")

    @staticmethod
    def _build_arclength(pts):
        s = [0.0]
        acc = 0.0
        for i in range(len(pts)-1):
            d = _norm2(_sub(pts[i+1], pts[i]))
            acc += d
            s.append(acc)
        return s

    def _project_on_path(self, p):
        """ Proyección de p sobre el segmento más cercano [p_i, p_{i+1}] para obtener s0 (arclength). """
        best = (float('inf'), 0, 0.0, pts_to_point(self.path_pts[0]))  # (dist, idx_i, t, proj_pt)
        for i in range(len(self.path_pts)-1):
            a = self.path_pts[i]
            b = self.path_pts[i+1]
            ab = _sub(b, a)
            ab2 = _dot(ab, ab)
            if ab2 <= 1e-12:
                t = 0.0
                proj = a
            else:
                t = _clamp(_dot(_sub(p, a), ab) / ab2, 0.0, 1.0)
                proj = _add(a, _scale(ab, t))
            d = _norm2(_sub(p, proj))
            if d < best[0]:
                best = (d, i, t, proj)
        _, i, t, proj = best
        s0 = self.s_cum[i] + t * (_norm2(_sub(self.path_pts[i+1], self.path_pts[i])))
        return s0, proj, i, t

    def _interp_at_s(self, s_star):
        """ p* = p_j + λ (p_{j+1}-p_j) con s_j <= s* < s_{j+1}. """
        if not self.s_cum: return None
        if s_star <= self.s_cum[0]:
            return self.path_pts[0]
        if s_star >= self.s_cum[-1]:
            return self.path_pts[-1]
        # búsqueda lineal (N ~ 1e3 típico; si N grande, usar bisect)
        for j in range(len(self.s_cum)-1):
            sj, sj1 = self.s_cum[j], self.s_cum[j+1]
            if sj <= s_star <= sj1:
                ds = sj1 - sj
                lam = 0.0 if ds <= 1e-12 else (s_star - sj) / ds
                return _add(self.path_pts[j], _scale(_sub(self.path_pts[j+1], self.path_pts[j]), lam))
        return self.path_pts[-1]

    def _estimate_path_kappa(self, s0):
        """ Estimación de curvatura local |kappa_path| a partir de tres puntos en una ventana de longitud curv_window_m. """
        if len(self.path_pts) < 3: return 0.0
        # Encuentra índices bracketing de s0 -/+ window/2
        half = 0.5 * self.curv_window_m
        s_lo = _clamp(s0 - half, self.s_cum[0], self.s_cum[-1])
        s_hi = _clamp(s0 + half, self.s_cum[0], self.s_cum[-1])
        p1 = self._interp_at_s(s_lo)
        p2 = self._interp_at_s(s0)
        p3 = self._interp_at_s(s_hi)
        # Curvatura por círculo osculador de 3 puntos
        k = curvature_from_three_points(p1, p2, p3)
        return abs(k)

    def _compute_Ld(self, v, kappa_path):
        mode = self.mode.lower()
        if mode == "constant":
            Ld = self.L0
        elif mode == "speed":
            Ld = self.L0 + self.kv * max(0.0, v)
        elif mode == "curvature":
            Ld = self.kk / (kappa_path + self.eps_kappa)
        elif mode == "hybrid":
            Ld = self.L0 + self.kv * max(0.0, v) + self.kk / (kappa_path + self.eps_kappa)
        else:
            Ld = self.L0
        return _clamp(Ld, self.Lmin, self.Lmax)

    def _on_timer(self, _evt):
        if self.odom is None or len(self.path_pts) < 2:
            return

        # Pose en odom
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        pveh = _p2(px, py)
        v = math.hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y)

        # (1) proyección -> s0
        s0, p_proj, i_min, t_min = self._project_on_path(pveh)

        # (2) Ld (posiblemente adaptativo)
        kappa_path = self._estimate_path_kappa(s0)
        Ld = self._compute_Ld(v, kappa_path)

        # (3) s* = s0 + Ld ; p* por interpolación
        s_star = s0 + Ld
        p_star = self._interp_at_s(s_star)
        if p_star is None:
            return

        # (4) transformar P* (odom -> base_link) en el stamp del odom
        ps = PointStamped()
        ps.header.stamp = self.odom.header.stamp if self.odom.header.stamp.to_sec() > 0 else rospy.Time.now()
        ps.header.frame_id = self.odom_frame
        ps.point = Point(p_star[0], p_star[1], 0.0)

        try:
            ps_bl = self.tfbuf.transform(ps, self.base_link_frame, timeout=rospy.Duration(0.05))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[pure_pursuit] TF odom->%s falló: %s" % (self.base_link_frame, str(e)))
            return

        xL, yL = ps_bl.point.x, ps_bl.point.y
        Ld_geom = max(1e-6, math.hypot(xL, yL))  # distancia geométrica al target

        # (5) curvatura y delta (pure pursuit)
        # kappa = 2*yL / Ld^2 ; delta = atan(L*kappa) ; saturaciones
        kappa = (2.0 * yL) / (Ld_geom * Ld_geom)
        delta = math.atan(self.L * kappa)

        # Saturaciones
        kappa_max = math.tan(self.delta_max) / max(1e-9, self.L)
        if abs(kappa) > kappa_max:
            kappa = _clamp(kappa, -kappa_max, kappa_max)
            delta = math.atan(self.L * kappa)

        if abs(delta) > self.delta_max:
            delta = _clamp(delta, -self.delta_max, self.delta_max)
            kappa = math.tan(delta) / max(1e-9, self.L)

        # Publicaciones
        self._publish_points(ps, ps_bl)
        self.pub_ld.publish(Float64(Ld))
        self.pub_delta.publish(Float64(delta))
        self.pub_kappa.publish(Float64(kappa))

        if self.publish_markers:
            self._publish_marker_target(ps)

        if self.publish_ackermann and self.pub_ack is not None:
            ack = AckermannDriveStamped()
            ack.header.stamp = ps.header.stamp
            ack.header.frame_id = self.base_link_frame
            ack.drive.steering_angle = float(delta)
            ack.drive.speed = float(v)  # pasa-through; tu control puede sobrescribir
            self.pub_ack.publish(ack)

def pts_to_point(p):
    return Point(p[0], p[1], 0.0)

def curvature_from_three_points(p1, p2, p3):
    # Fórmula robusta de curvatura: k = 2*Area / (a*b*c), con signos por orientación (usamos |k|)
    a = _norm2(_sub(p2, p1))
    b = _norm2(_sub(p3, p2))
    c = _norm2(_sub(p3, p1))
    if a < 1e-9 or b < 1e-9 or c < 1e-9:
        return 0.0
    # Doble área del triángulo (shoelace 2D)
    area2 = abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
    k = area2 / (a*b*c)
    return k

    # Nota: si quisieras signo, usa orientación (cross) entre (p2-p1) y (p3-p2)

def main():
    rospy.init_node("pure_pursuit_target_selector")
    node = PurePursuitTargetSelector()
    rospy.spin()

if __name__ == "__main__":
    main()
