#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

# Mensajes de tu paquete
from racecar_cone_msgs.msg import Cone, ConeArray


def dist2(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return dx*dx + dy*dy


class ScanToCones(object):
    def __init__(self):
        # Params
        self.scan_topic  = rospy.get_param("~scan_topic", "/scan")
        self.laser_frame = rospy.get_param("~laser_frame", "laser")
        self.out_frame   = rospy.get_param("~out_frame", "odom")

        # --- Rango útil del láser ---
        self.r_min = float(rospy.get_param("~r_min", 0.35))   # antes 0.20
        self.r_max = float(rospy.get_param("~r_max", 18.0))   # antes 12.0

        # --- Clustering por vecindad euclídea ---
        self.eps        = float(rospy.get_param("~eps_cluster", 0.10))  # antes 0.25
        self.min_points = int(rospy.get_param("~min_points", 2))        # antes 2
        self.max_points = int(rospy.get_param("~max_points", 20))       # antes 50

        # --- “Ancho” estimado del cluster (≈ diámetro aparente del cono) ---
        self.dmin = float(rospy.get_param("~cone_diam_min", 0.05))      # antes 0.12
        self.dmax = float(rospy.get_param("~cone_diam_max", 0.28))      # antes 0.40

        # --- Parámetros específicos del circle fitting Hyper ---
        self.min_circle_points = int(rospy.get_param("~min_circle_points", 3))
        self.svd_eps           = float(rospy.get_param("~svd_eps", 1.0e-12))

        # === NUEVO: parámetros de "región de operación" para Hyper (Estrategia A) ===
        # Distancia máxima a la que permitimos usar Hyper (más allá -> centroide)
        # Ajuste inicial: suficiente para conos relevantes cercanos, se puede tunear.
        self.hyper_r_max = float(rospy.get_param("~hyper_r_max", 10.0))

        # Mínimo de puntos dentro del cluster para plantearse Hyper
        # (algo más estricto que min_circle_points)
        self.hyper_min_points = int(rospy.get_param("~hyper_min_points", 4))

        # Apertura angular mínima (en grados) para plantearse Hyper
        # El caso "malo" que viste tenía ~1º; aquí exigimos algo tipo >=3º.
        self.hyper_min_angle_deg = float(rospy.get_param("~hyper_min_angle_deg", 3.0))
        self.hyper_min_angle_rad = math.radians(self.hyper_min_angle_deg)

        # --- Marcadores RViz ---
        self.pub_markers = bool(rospy.get_param("~pub_markers", True))

        # TF2
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(3.0))
        self.tflis = tf2_ros.TransformListener(self.tfbuf)

        # IO
        self.pub_cones = rospy.Publisher("/cones/raw", ConeArray, queue_size=10)
        self.pub_mk    = rospy.Publisher("/cones/markers_raw", MarkerArray, queue_size=10) if self.pub_markers else None
        self.sub_scan  = rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=5)

        # Publisher exclusivo para el “serial” de conos
        self.pub_debug = rospy.Publisher("/cones/debug_serial", String, queue_size=10)

    # ============================================================
    # Utilidades
    # ============================================================
    def _laser_pts(self, scan):
        """Convierte el LaserScan a puntos en el marco del láser."""
        pts = []
        ang = scan.angle_min
        for r in scan.ranges:
            if not (math.isinf(r) or math.isnan(r)) and (self.r_min <= r <= self.r_max):
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                pts.append((x, y))
            ang += scan.angle_increment
        return pts

    def _clusters(self, pts):
        """Clusterización 1D por vecindad euclídea entre puntos consecutivos."""
        clusters = []
        if not pts:
            return clusters

        cur = [pts[0]]
        for i in range(1, len(pts)):
            if dist2(pts[i], pts[i-1]) <= self.eps*self.eps:
                cur.append(pts[i])
            else:
                clusters.append(cur)
                cur = [pts[i]]
        clusters.append(cur)
        return clusters

    def _cluster_ok(self, cl):
        """Filtra clusters por nº de puntos y ancho geométrico aproximado."""
        n = len(cl)
        if not (self.min_points <= n <= self.max_points):
            return False

        # Ancho = distancia entre extremos del cluster en marco LASER
        w = math.sqrt(dist2(cl[0], cl[-1]))
        return (self.dmin <= w <= self.dmax)

    def _centroid(self, cl):
        """Centroid del cluster en marco LASER."""
        sx = sum(p[0] for p in cl)
        sy = sum(p[1] for p in cl)
        n = float(len(cl))
        return (sx/n, sy/n)

    def _cluster_geom(self, cl):
        """
        MÉTRICAS GEOMÉTRICAS DEL CLUSTER (Estrategia A - Parte 1):
          - r_mean: distancia media al LIDAR
          - dtheta: apertura angular en radianes
          - dtheta_deg: apertura angular en grados
        """
        n = len(cl)
        if n == 0:
            return (0.0, 0.0, 0.0)

        rs = []
        thetas = []
        for (x, y) in cl:
            rs.append(math.hypot(x, y))
            thetas.append(math.atan2(y, x))

        r_mean = sum(rs) / float(n)
        th_min = min(thetas)
        th_max = max(thetas)
        dtheta = th_max - th_min
        dtheta_deg = math.degrees(dtheta)

        return (r_mean, dtheta, dtheta_deg)

    def _circle_fit_hyper(self, cl):
        """
        Ajuste de círculo por método Hyper (Al-Sharadqah & Chernov, resumen Northwestern).
        Entrada: lista de (x,y) en marco LASER.
        Salida: (cx, cy, R, rms) en marco LASER o None si falla.
        """
        n = len(cl)
        if n < self.min_circle_points:
            return None

        # Datos originales
        xs = np.array([p[0] for p in cl], dtype=float)
        ys = np.array([p[1] for p in cl], dtype=float)

        # 1) Centroid de los datos originales (hat{x}, hat{y})
        x_hat = xs.mean()
        y_hat = ys.mean()

        # 2) Cambio de origen al centroid
        x = xs - x_hat
        y = ys - y_hat

        # 3) z_i = x_i^2 + y_i^2
        z = x*x + y*y

        # 4) media de z
        z_bar = z.mean()

        # 5) Matriz Z (n x 4)
        #    [ z_i, x_i, y_i, 1 ]
        one = np.ones(n, dtype=float)
        Z = np.column_stack((z, x, y, one))

        # 9) SVD de Z
        try:
            U, S, Vt = np.linalg.svd(Z, full_matrices=False)
        except Exception:
            return None

        V = Vt.T
        sigma4 = S[-1]

        # 10) Caso casi-degenerado: usar directamente el último vector singular
        if sigma4 < self.svd_eps:
            A = V[:, -1]
        else:
            # 11) Caso general:
            # Y = V Σ V^T  (Σ es diag(S))
            # V * S multiplica cada columna de V por S[i]
            Y = np.dot(V * S, V.T)

            # H^{-1} según la página (ya con datos centrados -> x̄=ȳ=0)
            H_inv = np.array([[0.0, 0.0, 0.0, 0.5],
                              [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.5, 0.0, 0.0, -2.0*z_bar]])

            # Q = Y H^{-1} Y
            Q = np.dot(np.dot(Y, H_inv), Y)

            # Autovalores/vectores de Q (simétrica -> reales)
            try:
                eigvals, eigvecs = np.linalg.eig(Q)
            except Exception:
                return None

            eigvals = eigvals.real
            eigvecs = eigvecs.real

            # Elegir el autovalor positivo mínimo
            positive = eigvals > 0
            if np.any(positive):
                idx_pos = np.arange(len(eigvals))[positive]
                idx = idx_pos[np.argmin(eigvals[positive])]
            else:
                # Fallback: el de menor valor absoluto
                idx = int(np.argmin(np.abs(eigvals)))

            A_star = eigvecs[:, idx]

            # Resolver Y A = A_star
            try:
                A = np.linalg.solve(Y, A_star)
            except np.linalg.LinAlgError:
                return None

        A1, A2, A3, A4 = A

        if abs(A1) < 1.0e-14:
            return None

        # 12) Centro y radio en el sistema centrado (x,y)
        a = -A2 / (2.0 * A1)
        b = -A3 / (2.0 * A1)
        R2 = (A2*A2 + A3*A3 - 4.0*A1*A4) / (4.0*A1*A1)

        if R2 <= 0.0:
            return None

        R = math.sqrt(R2)

        # 13) Deshacer el cambio de origen: centro en coordenadas originales del LIDAR
        cx = a + x_hat
        cy = b + y_hat

        # 14) RMS del error algebraico (opcional)
        dx = x - a
        dy = y - b
        delta = dx*dx + dy*dy - R2
        rms = math.sqrt(float(np.mean(delta*delta)))

        return (cx, cy, R, rms)

    def _to_out_frame(self, x_l, y_l, stamp):
        """Transforma un punto del marco LASER al marco out_frame (odom/map)."""
        pt_l = PointStamped()
        pt_l.header.stamp = stamp
        pt_l.header.frame_id = self.laser_frame
        pt_l.point = Point(x_l, y_l, 0.0)

        try:
            # Intento con el stamp del escaneo (sim_time correcto)
            T = self.tfbuf.lookup_transform(self.out_frame, self.laser_frame,
                                            stamp, rospy.Duration(0.2))
        except Exception:
            try:
                # Fallback: último transform disponible
                T = self.tfbuf.lookup_transform(self.out_frame, self.laser_frame,
                                                rospy.Time(0), rospy.Duration(0.2))
            except Exception:
                return None

        pt_o = tf2_geometry_msgs.do_transform_point(pt_l, T)
        return (pt_o.point.x, pt_o.point.y, pt_o.point.z)

    def _make_cone_msg(self, x, y, stamp, cone_id, source=None):
        """Construye un Cone totalmente consistente con Cone.msg."""
        c = Cone()

        # Header (frame fijo: odom/map)
        c.header.stamp = stamp
        c.header.frame_id = self.out_frame

        # Posición 2D (z=0 si 2D)
        c.p.x = x
        c.p.y = y
        c.p.z = 0.0

        # Covarianza plana 2x2 (xx, xy, yx, yy) -> aquí sin estimar (0)
        c.cov_xy = [0.0, 0.0, 0.0, 0.0]

        # Lateralidad desconocida por ahora
        c.side = Cone.UNKNOWN

        # Confianza y metadatos básicos
        c.confidence = 1.0

        # Fuente: por defecto mantenemos "lidar_hyper" para compatibilidad,
        # pero ahora podemos distinguir si se ha usado Hyper o centroide.
        if source is None:
            c.source = "lidar_hyper"
        else:
            c.source = source

        c.id = int(cone_id)

        return c

    # ============================================================
    # Callback principal
    # ============================================================
    def cb_scan(self, scan):
        # Buffer de debug
        debug_lines = []

        t_sec = scan.header.stamp.to_sec()
        frame = scan.header.frame_id if scan.header.frame_id else self.laser_frame

        debug_lines.append("========== scan_to_cones DEBUG ==========")
        debug_lines.append("t = %.6f  frame = %s" % (t_sec, frame))

        # ---- [1] LaserScan bruto ----
        debug_lines.append("[1] LaserScan input:")
        debug_lines.append("    n_ranges=%d" % len(scan.ranges))
        debug_lines.append("    angle_min=%.3f  angle_max=%.3f  angle_inc=%.5f" %
                           (scan.angle_min, scan.angle_max, scan.angle_increment))
        debug_lines.append("    range_min=%.3f  range_max=%.3f" %
                           (scan.range_min, scan.range_max))
        debug_lines.append("    r_min(filtro)=%.3f  r_max(filtro)=%.3f" %
                           (self.r_min, self.r_max))
        debug_lines.append("    hyper_r_max=%.3f  hyper_min_points=%d  hyper_min_angle=%.2f deg" %
                           (self.hyper_r_max, self.hyper_min_points, self.hyper_min_angle_deg))

        def _fmt_r(r):
            if math.isinf(r):
                return "  inf "
            if math.isnan(r):
                return "  nan "
            return "%6.3f" % r

        debug_lines.append("    ranges (idx: valor):")
        line = "    "
        per_line = 8
        for i, r in enumerate(scan.ranges):
            chunk = "%3d:%s  " % (i, _fmt_r(r))
            line += chunk
            if (i + 1) % per_line == 0:
                debug_lines.append(line)
                line = "    "
        if line.strip():
            debug_lines.append(line)

        # ---- [2] pts en marco LASER ----
        ptsL = self._laser_pts(scan)

        debug_lines.append("[2] Puntos válidos en marco LASER (pts):")
        debug_lines.append("    n_pts=%d (r finitos y %s<=r<=%s)" %
                           (len(ptsL), repr(self.r_min), repr(self.r_max)))
        for i, (x, y) in enumerate(ptsL):
            debug_lines.append("    pt[%4d] = (%.3f, %.3f)" % (i, x, y))

        # ---- [3] Clustering ----
        clusts = self._clusters(ptsL)
        debug_lines.append("[3] Clusters formados a partir de pts:")

        cluster_ok_flags = []
        cluster_geom = []  # (r_mean, dtheta, dtheta_deg, n)

        for j, cl in enumerate(clusts):
            n = len(cl)
            if n >= 2:
                w = math.sqrt(dist2(cl[0], cl[-1]))
            else:
                w = 0.0

            ok = self._cluster_ok(cl)
            cluster_ok_flags.append(ok)

            # Métricas geométricas (Estrategia A - Parte 1)
            r_mean, dtheta, dtheta_deg = self._cluster_geom(cl)
            cluster_geom.append((r_mean, dtheta, dtheta_deg, n))

            debug_lines.append("    cluster[%3d]: n_pts=%3d  width=%.3f  ok=%s" %
                               (j, n, w, str(ok)))
            debug_lines.append("        geom: r_mean=%.3f  dtheta=%.4f rad (%.2f deg)" %
                               (r_mean, dtheta, dtheta_deg))
            debug_lines.append("        puntos (x, y):")
            for (x, y) in cl:
                debug_lines.append("            (%.3f, %.3f)" % (x, y))

        # ---- [4] Transformación y generación de conos ----
        cones = []

        if self.pub_markers:
            markers = MarkerArray()
            # Limpieza para que no queden marcadores viejos
            mclear = Marker()
            mclear.action = Marker.DELETEALL
            markers.markers.append(mclear)
        else:
            markers = None

        mid = 0  # id de Marker

        debug_lines.append("[4] Clusters aceptados -> conos (marco %s):" %
                           self.out_frame)

        for j, cl in enumerate(clusts):
            if not cluster_ok_flags[j]:
                debug_lines.append("    cluster[%3d]: descartado por _cluster_ok" % j)
                continue

            # Geometría para gating Hyper (Estrategia A - Parte 2)
            r_mean, dtheta, dtheta_deg, n = cluster_geom[j]

            cond_n      = (n >= self.hyper_min_points)
            cond_r      = (r_mean <= self.hyper_r_max)
            cond_dtheta = (dtheta >= self.hyper_min_angle_rad)

            use_hyper = (cond_n and cond_r and cond_dtheta)

            debug_lines.append("    cluster[%3d] Hyper gating:" % j)
            debug_lines.append("        n=%d  r_mean=%.3f  dtheta=%.4f rad (%.2f deg)" %
                               (n, r_mean, dtheta, dtheta_deg))
            debug_lines.append("        cond_n=%s  cond_r=%s  cond_dtheta=%s  -> use_hyper=%s" %
                               (str(cond_n), str(cond_r), str(cond_dtheta), str(use_hyper)))

            # Centroid simple en marco LASER (para debug y fallback)
            cxC, cyC = self._centroid(cl)

            # Ajuste de círculo Hyper en marco LASER (sólo si gating lo permite)
            attempted_hyper = False
            if use_hyper:
                attempted_hyper = True
                fit = self._circle_fit_hyper(cl)
                if fit is None:
                    cxL, cyL = cxC, cyC
                    R        = float('nan')
                    rms      = float('nan')
                    used_hyper = False
                else:
                    cxL, cyL, R, rms = fit
                    used_hyper = True
            else:
                # No se intenta Hyper, nos quedamos con el centroide
                cxL, cyL = cxC, cyC
                R        = float('nan')
                rms      = float('nan')
                used_hyper = False

            po = self._to_out_frame(cxL, cyL, scan.header.stamp)
            if po is None:
                debug_lines.append("    cluster[%3d]: TF %s->%s FAILED, descartado" %
                                   (j, self.laser_frame, self.out_frame))
                continue

            xO, yO, zO = po

            idx = len(cones)

            # Etiquetar la fuente según si se ha usado Hyper o centroide
            if used_hyper:
                source_str = "lidar_hyper"
            else:
                # Centroide (bien por gating, bien por fallo de ajuste)
                source_str = "lidar_centroid"

            cone_msg = self._make_cone_msg(xO, yO, scan.header.stamp,
                                           cone_id=idx,
                                           source=source_str)
            cones.append(cone_msg)

            debug_lines.append("    cluster[%3d] -> cone idx=%3d:" % (j, idx))
            debug_lines.append("        centroid_laser      = (%.3f, %.3f)" % (cxC, cyC))

            if used_hyper:
                debug_lines.append("        hyper_center_laser = (%.3f, %.3f)  R=%.3f  rms=%.3e" %
                                   (cxL, cyL, R, rms))
            else:
                if attempted_hyper:
                    debug_lines.append("        hyper_center_laser = FAILED (fit=None), usando centroid")
                else:
                    debug_lines.append("        hyper_center_laser = SKIPPED (gating), usando centroid")

            debug_lines.append("        punto_%s            = (%.3f, %.3f, %.3f)  source=%s" %
                               (self.out_frame, xO, yO, zO, source_str))

            # Marcador RViz del cono detectado (opcional)
                        # Marcador RViz del cono detectado (opcional)
            if self.pub_markers:
                m = Marker()
                m.header.stamp = scan.header.stamp
                m.header.frame_id = self.out_frame
                m.ns = "cones_raw"
                m.id = mid
                mid += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = xO
                m.pose.position.y = yO
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                # Escala fija (puedes, si quieres, escalar x/y con 2*R para visualizar el círculo)
                m.scale.x = 0.15
                m.scale.y = 0.15
                m.scale.z = 0.15

                # === Color según método usado ===
                if used_hyper:
                    # Conos obtenidos por ajuste Hyper -> morado
                    m.color.r = 0.6
                    m.color.g = 0.0
                    m.color.b = 0.8
                else:
                    # Conos obtenidos por centroide -> naranja
                    m.color.r = 1.0
                    m.color.g = 0.55
                    m.color.b = 0.0

                m.color.a = 0.9
                m.lifetime = rospy.Duration(0.0)
                markers.markers.append(m)


        # ---- [5] ConeArray final ----
        debug_lines.append("[5] ConeArray publicado en /cones/raw:")
        debug_lines.append("    n_cones=%d" % len(cones))
        for idx, c in enumerate(cones):
            debug_lines.append(
                "    cone[%3d]: id=%d  p=(%.3f, %.3f, %.3f)  side=%d  conf=%.2f  src=%s" %
                (idx, c.id, c.p.x, c.p.y, c.p.z, c.side, c.confidence, c.source)
            )

        debug_lines.append("========== end scan_to_cones DEBUG ==========")

        # Publicación del bloque de debug (solo al tópico, no a consola)
        msg_debug = String()
        msg_debug.data = "\n".join(debug_lines)
        self.pub_debug.publish(msg_debug)

        # ---- Publicación normal de datos de conos ----
        out = ConeArray()
        out.header.stamp = scan.header.stamp
        out.header.frame_id = self.out_frame
        out.cones = cones

        self.pub_cones.publish(out)
        if self.pub_markers:
            self.pub_mk.publish(markers)


def main():
    rospy.init_node("scan_to_cones")
    ScanToCones()
    rospy.spin()


if __name__ == "__main__":
    main()
