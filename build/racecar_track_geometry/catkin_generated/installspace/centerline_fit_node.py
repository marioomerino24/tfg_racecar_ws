#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from racecar_cone_msgs.msg import MidpointArray, Centerline, TrackMetrics

# ===================== utilidades spline =====================

def cum_arclength(pts):
    s = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i].x - pts[i-1].x
        dy = pts[i].y - pts[i-1].y
        s.append(s[-1] + math.hypot(dx, dy))
    return s

def natural_cubic_spline_1d(s, y):
    """
    Devuelve los momentos M[i] = segunda derivada en cada nodo s[i]
    para el spline cúbico natural que interpola (s[i], y[i]).
    """
    n = len(s)
    if n < 3:
        # Con pocos puntos, no tiene sentido un spline completo:
        return [0.0] * n

    h = [s[i+1] - s[i] for i in range(n-1)]

    # RHS del sistema tridiagonal (alpha en muchos textos)
    alpha = [0.0] * n
    for i in range(1, n-1):
        alpha[i] = (3.0/h[i])   * (y[i+1] - y[i])   \
                 - (3.0/h[i-1]) * (y[i]   - y[i-1])

    # Algoritmo de Thomas para condiciones naturales: M[0]=M[n-1]=0
    l  = [0.0] * n
    mu = [0.0] * n
    z  = [0.0] * n

    l[0] = 1.0
    mu[0] = 0.0
    z[0] = 0.0

    for i in range(1, n-1):
        l[i]  = 2.0*(s[i+1] - s[i-1]) - h[i-1]*mu[i-1]
        if abs(l[i]) < 1e-12:
            l[i] = 1e-12
        mu[i] = h[i] / l[i]
        z[i]  = (alpha[i] - h[i-1]*z[i-1]) / l[i]

    l[n-1] = 1.0
    z[n-1] = 0.0

    # back-substitution para M
    M = [0.0] * n
    M[n-1] = 0.0
    for j in reversed(range(0, n-1)):
        M[j] = z[j] - mu[j]*M[j+1]

    return M

def eval_spline_seg(s_i, s_ip1, y_i, y_ip1, M_i, M_ip1, s_q):
    """
    Evalúa el spline cúbico natural en el intervalo [s_i, s_ip1]
    usando la formulación estándar con momentos M_i, M_ip1.
    """
    h = s_ip1 - s_i
    if h <= 1e-9:
        return y_i
    t = (s_q - s_i) / h  # t in [0,1]
    A = 1.0 - t
    B = t
    # términos cúbicos en función de los momentos
    C = (A**3 - A) * h*h / 6.0
    D = (B**3 - B) * h*h / 6.0
    return A*y_i + B*y_ip1 + C*M_i + D*M_ip1

# ===================== nodo principal =====================

class CenterlineFitNode(object):
    def __init__(self):
        # Parámetro: paso de muestreo en arclength para la salida
        self.ds = rospy.get_param("~resample_ds", 0.25)

        # Publicadores principales
        self.pub_centerline = rospy.Publisher("/track/centerline",
                                              Centerline,
                                              queue_size=1)
        self.pub_metrics = rospy.Publisher("/track/metrics",
                                           TrackMetrics,
                                           queue_size=1)

        # Publicador de logs de debug
        self.pub_debug = rospy.Publisher("/track/centerline_debug",
                                         String,
                                         queue_size=10)

        rospy.Subscriber("/track/midpoints",
                         MidpointArray,
                         self.cb,
                         queue_size=1)

    def debug(self, text):
        msg_dbg = String()
        msg_dbg.data = text
        self.pub_debug.publish(msg_dbg)
        # opcional: también al log ROS
        #rospy.loginfo(text)

    def cb(self, msg):
        pts = msg.points
        n_in = len(pts)

        if n_in < 3:
            self.debug("centerline_fit: menos de 3 puntos (%d), no se ajusta spline." % n_in)
            return

        self.debug("centerline_fit: recibido MidpointArray con %d puntos" % n_in)

        # 1) Parametrización por arclength de los midpoints originales
        s_nodes = cum_arclength(pts)
        L = s_nodes[-1]
        if L < 1e-6:
            self.debug("centerline_fit: longitud total L casi cero (%.6f), abortando." % L)
            return

        self.debug("centerline_fit: longitud total L = %.3f m" % L)

        x_nodes = [p.x for p in pts]
        y_nodes = [p.y for p in pts]

        # 2) Calcula momentos de spline cúbico natural en X e Y
        Mx = natural_cubic_spline_1d(s_nodes, x_nodes)
        My = natural_cubic_spline_1d(s_nodes, y_nodes)
        self.debug("centerline_fit: calculados momentos Mx, My para spline cúbico natural.")

        # 3) Construye rejilla de muestreo uniforme en arclength
        ds = self.ds
        s_out = []
        q = 0.0
        while q <= L:
            s_out.append(q)
            q += ds
        # asegura incluir exactamente el final
        if s_out[-1] < L - 1e-6:
            s_out.append(L)

        self.debug("centerline_fit: remuestreando con ds=%.3f -> %d muestras." %
                   (ds, len(s_out)))

        # 4) Evalúa spline en cada s_out
        samples_out = []
        i = 0
        for q in s_out:
            # busca intervalo [s_nodes[i], s_nodes[i+1]] que contiene q
            while i+1 < len(s_nodes) and s_nodes[i+1] < q:
                i += 1
            if i+1 >= len(s_nodes):
                # clamp al último segmento
                i = len(s_nodes) - 2

            xq = eval_spline_seg(s_nodes[i], s_nodes[i+1],
                                 x_nodes[i], x_nodes[i+1],
                                 Mx[i], Mx[i+1],
                                 q)
            yq = eval_spline_seg(s_nodes[i], s_nodes[i+1],
                                 y_nodes[i], y_nodes[i+1],
                                 My[i], My[i+1],
                                 q)
            samples_out.append(Point(xq, yq, 0.0))

        n_out = len(samples_out)
        self.debug("centerline_fit: spline evaluado. Muestras de salida: %d" % n_out)

        # 5) Curvatura discreta sobre la trayectoria suavizada
        kappa = []
        for i in range(n_out):
            if i == 0 or i == n_out-1:
                kappa.append(0.0)
            else:
                x1, y1 = samples_out[i-1].x, samples_out[i-1].y
                x2, y2 = samples_out[i].x,   samples_out[i].y
                x3, y3 = samples_out[i+1].x, samples_out[i+1].y

                a = math.hypot(x2-x1, y2-y1)
                b = math.hypot(x3-x2, y3-y2)
                c = math.hypot(x3-x1, y3-y1)

                denom = a*b*c
                if denom < 1e-9:
                    kappa.append(0.0)
                else:
                    area = abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0
                    kappa.append(4.0 * area / denom)

        # 6) Construye Centerline de salida (ya interpolada y uniforme)
        out = Centerline()
        out.header = msg.header
        out.samples = samples_out
        out.s = s_out
        out.kappa = kappa
        out.is_arclength_uniform = True
        out.ds = ds

        self.pub_centerline.publish(out)
        self.debug("centerline_fit: publicado /track/midline con spline suavizada.")

        # 7) Métricas globales (sobre la trayectoria suavizada)
        met = TrackMetrics()
        met.header = msg.header
        met.cones_raw_count = 0
        met.pairs_count = len(msg.points)       # número de midpoints originales
        met.midpoints_count = len(msg.points)   # puedes cambiarlo si prefieres n_out

        # Estadística de ancho (sobre los msg.width originales)
        if len(msg.width) > 1:
            mu_w = sum(msg.width) / float(len(msg.width))
            var_w = sum((w - mu_w)*(w - mu_w) for w in msg.width) / float(len(msg.width)-1)
            std_w = math.sqrt(max(0.0, var_w))
        else:
            std_w = 0.0
        met.std_width = std_w

        # Curvaturas
        abs_k = [abs(kk) for kk in kappa]
        met.max_kappa = max(abs_k) if abs_k else 0.0

        # integral discreta de kappa^2 * ds sobre la trayectoria suavizada
        integ = 0.0
        for i in range(1, n_out):
            ds_loc = math.hypot(samples_out[i].x - samples_out[i-1].x,
                                samples_out[i].y - samples_out[i-1].y)
            ki = kappa[i]
            integ += ki*ki*ds_loc
        met.integral_kappa_sq = integ

        met.coverage = 1.0
        met.latency_ms = 0.0
        met.notes = "resample_ds=%.3f, N_in=%d, N_out=%d" % (ds, n_in, n_out)

        self.pub_metrics.publish(met)
        self.debug("centerline_fit: publicado /track/metrics (std_width=%.3f, max_kappa=%.4f)." %
                   (std_w, met.max_kappa))


if __name__ == "__main__":
    rospy.init_node("centerline_fit")
    node = CenterlineFitNode()
    rospy.spin()
