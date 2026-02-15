#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
import tf
from racecar_cone_msgs.msg import ConeArray, MidpointArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String


def median(values):
    vals = sorted(values)
    n = len(vals)
    if n == 0:
        return 0.0
    mid = n // 2
    if (n % 2) == 1:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def robust_var(values, fallback_var):
    n = len(values)
    if n < 2:
        return fallback_var
    med = median(values)
    abs_dev = [abs(v - med) for v in values]
    mad = median(abs_dev)
    # sigma ~= 1.4826 * MAD para distribución normal
    sigma = 1.4826 * mad
    var = sigma * sigma
    if var <= 0.0:
        return fallback_var
    return var


class MidpointsNode(object):
    def __init__(self):
        self.topic_cones = rospy.get_param("~topic_cones", "/perception/lidar/cones")
        self.topic_midpoints = rospy.get_param("~topic_midpoints", "/estimation/track/midpoints")
        self.topic_midpoints_marker = rospy.get_param("~topic_midpoints_marker", "/estimation/track/midpoints/markers")

        # Publicador de la nube de puntos medios
        self.pub_midpoints = rospy.Publisher(self.topic_midpoints,
                                             MidpointArray,
                                             queue_size=1)
        # Publicador del marcador para RViz
        self.pub_marker = rospy.Publisher(self.topic_midpoints_marker,
                                          Marker,
                                          queue_size=1)
        self.debug_topic = rospy.get_param("~debug/topic", "/estimation/track/midpoints/debug")
        self.debug_enable = bool(rospy.get_param("~debug/enable", True))
        self.debug_level = rospy.get_param("~debug/level", "summary")
        self.debug_report_every_n = int(rospy.get_param("~debug/report_every_n", 1))
        self.debug_max_bins = int(rospy.get_param("~debug/max_bins", 12))
        # Salida multilinea real por consola (mas legible que rostopic echo para textos largos).
        self.debug_console_enable = bool(rospy.get_param("~debug/console_enable", True))
        self.pub_debug = rospy.Publisher(self.debug_topic, String, queue_size=10)
        self.debug_counter = 0

        # Fuente de conos detectados (scan_to_cones)
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Binning longitudinal en marco del robot (s, d)
        self.s_min = float(rospy.get_param("~s_min", -0.2))
        self.s_max = float(rospy.get_param("~s_max", 10.0))
        self.ds_bin = float(rospy.get_param("~ds_bin", 0.5))
        self.min_per_side = int(rospy.get_param("~min_per_side", 1))
        # Filtro lateral para binning: descarta conos con |d| mayor que este umbral.
        # Si no se define, usa 3.0 m como anchura limite.
        self.track_width_limit = float(rospy.get_param("~track_width", 3.0))
        # Horizonte maximo hacia delante para generar bins/midpoints.
        # Si no se define, usa 4.0 m.
        self.forward_length_limit = float(rospy.get_param("~forward_length_limit", 4.0))

        # Gating de ancho de pista por bin para evitar outliers.
        # Defaults algo más relajados para mantener continuidad.
        self.width_min = float(rospy.get_param("~width_min", 2.0))
        self.width_max = float(rospy.get_param("~width_max", 4.2))

        # Ruido base cuando no hay suficiente muestra
        self.default_d_std = float(rospy.get_param("~default_d_std", 0.10))

        # Tamaño de la "X" (longitud total de cada brazo en metros)
        self.marker_size = rospy.get_param("~marker_size", 0.4)

        # Estabilización de midpoints:
        # - si hay una estimación buena previa, se mantiene ("latched")
        # - solo se sustituye cuando la nueva estimación mejora de forma clara
        self.enable_latching = bool(rospy.get_param("~stability/enable_latching", True))
        # Valores por defecto relajados para escenarios con visibilidad parcial.
        self.min_points_to_latch = int(rospy.get_param("~stability/min_points_to_latch", 3))
        self.min_mean_conf_to_latch = float(
            rospy.get_param("~stability/min_mean_confidence_to_latch", 0.25)
        )
        self.min_score_to_latch = float(rospy.get_param("~stability/min_score_to_latch", 0.8))
        self.replace_if_score_improves_ratio = float(
            rospy.get_param("~stability/replace_if_score_improves_ratio", 1.35)
        )
        self.replace_if_score_not_worse_ratio = float(
            rospy.get_param("~stability/replace_if_score_not_worse_ratio", 0.90)
        )
        self.max_update_rate_hz = float(rospy.get_param("~stability/max_update_rate_hz", 5.0))
        # 0.0 => sin timeout (mantener indefinidamente mientras no aparezca algo mejor)
        self.max_hold_seconds = float(rospy.get_param("~stability/max_hold_seconds", 0.0))

        # Estado latched
        self.latched_midpoints = None
        self.latched_score = 0.0
        self.latched_time = None
        self.last_replace_time = None
        self.last_latch_reason = "init"
        self.last_latch_metrics = {}

        # Garantía mínima para control:
        # asegurar al menos N puntos medios por delante del coche.
        self.enable_forward_guard = bool(rospy.get_param("~control/enable_forward_guard", True))
        self.min_forward_midpoints = int(rospy.get_param("~control/min_forward_midpoints", 2))
        self.forward_s_min = float(rospy.get_param("~control/forward_s_min", 0.25))
        self.forward_spacing = float(rospy.get_param("~control/forward_spacing", 0.8))
        self.fallback_width = float(rospy.get_param("~control/fallback_width", 3.0))
        # Recorte dinámico de profundidad por curvatura local:
        # más curvatura => menor horizonte de generación/publicación de midpoints.
        self.enable_curvature_depth_limit = bool(
            rospy.get_param("~control/enable_curvature_depth_limit", True)
        )
        self.curvature_depth_kappa_low = float(
            rospy.get_param("~control/curvature_depth_kappa_low", 0.08)
        )
        self.curvature_depth_kappa_high = float(
            rospy.get_param("~control/curvature_depth_kappa_high", 0.30)
        )
        self.curvature_depth_min = float(
            rospy.get_param("~control/curvature_depth_min", 2.0)
        )
        self.curvature_depth_max = float(
            rospy.get_param("~control/curvature_depth_max", 8.0)
        )
        self.curvature_depth_alpha = float(
            rospy.get_param("~control/curvature_depth_alpha", 0.35)
        )
        self.last_depth_limit = self.forward_length_limit
        self.last_kappa_est = 0.0

        # Ajuste de bordes (opcion 3):
        # construir d_left(s), d_right(s) robustos y derivar midpoints desde ahi.
        self.enable_boundary_fit = bool(rospy.get_param("~boundary_fit/enable", True))
        self.boundary_smooth_window_bins = int(
            rospy.get_param("~boundary_fit/smooth_window_bins", 5)
        )
        self.boundary_interp_max_gap_bins = int(
            rospy.get_param("~boundary_fit/interp_max_gap_bins", 3)
        )
        self.boundary_use_single_side_inference = bool(
            rospy.get_param("~boundary_fit/use_single_side_inference", True)
        )
        self.boundary_conf_inferred_penalty = float(
            rospy.get_param("~boundary_fit/conf_inferred_penalty", 0.60)
        )
        self.boundary_min_bins_for_width_ref = int(
            rospy.get_param("~boundary_fit/min_bins_for_width_ref", 2)
        )
        # Evita que la inferencia de un solo lado "estire" la traza lejos en curvas.
        self.boundary_inference_s_max = float(
            rospy.get_param("~boundary_fit/inference_s_max", 4.0)
        )

        self.width_ref_latched = self.fallback_width

        self.tf = tf.TransformListener()

        rospy.Subscriber(self.topic_cones, ConeArray, self.cb, queue_size=1)

    def _robot_pose_in_frame(self, frame_id):
        self.tf.waitForTransform(frame_id, self.base_frame,
                                 rospy.Time(0), rospy.Duration(0.05))
        (trans, rot) = self.tf.lookupTransform(frame_id, self.base_frame,
                                               rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(
            [rot[0], rot[1], rot[2], rot[3]])[2]
        pref = (trans[0], trans[1])
        t = (math.cos(yaw), math.sin(yaw))
        n = (-math.sin(yaw), math.cos(yaw))
        return pref, t, n

    def cb(self, msg):
        try:
            pref, t, n = self._robot_pose_in_frame(msg.header.frame_id)
        except Exception as ex:
            rospy.logwarn_throttle(1.0,
                                   "midpoints: TF %s->%s no disponible (%s)",
                                   msg.header.frame_id, self.base_frame, str(ex))
            return

        # Preparar bins longitudinales
        if self.ds_bin <= 1.0e-6 or self.s_max <= self.s_min:
            rospy.logwarn_throttle(1.0, "midpoints: parametros de binning invalidos")
            return

        s_max_eff = min(self.s_max, self.forward_length_limit)
        if s_max_eff <= self.s_min:
            rospy.logwarn_throttle(
                1.0,
                "midpoints: horizonte frontal invalido (s_min=%.2f, s_max_eff=%.2f).",
                self.s_min, s_max_eff
            )
            return

        n_bins = int(math.ceil((s_max_eff - self.s_min) / self.ds_bin))
        left_bins = [[] for _ in range(n_bins)]   # guarda d de conos lado izquierdo
        right_bins = [[] for _ in range(n_bins)]  # guarda d de conos lado derecho
        left_bin_cone_indices = [[] for _ in range(n_bins)]   # guarda indices RViz por bin (izq)
        right_bin_cone_indices = [[] for _ in range(n_bins)]  # guarda indices RViz por bin (der)
        cones_in_range = 0

        # Proyección de conos a coordenadas del robot (s, d)
        cone_trace = []
        for rviz_idx, cone in enumerate(msg.cones):
            relx = cone.p.x - pref[0]
            rely = cone.p.y - pref[1]
            s = relx * t[0] + rely * t[1]
            d = relx * n[0] + rely * n[1]
            in_track_width = (abs(d) <= self.track_width_limit)
            in_range = (s >= self.s_min and s < s_max_eff and in_track_width)
            b = -1
            side = "left" if d >= 0.0 else "right"

            if in_range:
                b = int((s - self.s_min) / self.ds_bin)
                if b >= 0 and b < n_bins:
                    cones_in_range += 1
                    if d >= 0.0:
                        left_bins[b].append(d)
                        left_bin_cone_indices[b].append(rviz_idx)
                    else:
                        right_bins[b].append(d)
                        right_bin_cone_indices[b].append(rviz_idx)
                else:
                    in_range = False

            cone_trace.append({
                "rviz_idx": rviz_idx,
                "cone_id": int(getattr(cone, "id", -1)),
                "x": cone.p.x,
                "y": cone.p.y,
                "s": s,
                "d": d,
                "in_track_width": in_track_width,
                "in_range": in_range,
                "bin": b,
                "side": side,
                "source": str(getattr(cone, "source", "unknown")),
                "confidence": float(getattr(cone, "confidence", 0.0))
            })

        # ==== Mensaje MidpointArray ====
        out = MidpointArray()
        out.header = msg.header
        out.points = []
        out.width = []
        out.cov_xy_flat = []
        out.id = []
        out.confidence = []

        sigma_s2 = (self.ds_bin * self.ds_bin) / 12.0
        fallback_d_var = self.default_d_std * self.default_d_std

        left_raw = [None] * n_bins
        right_raw = [None] * n_bins
        left_support = [0] * n_bins
        right_support = [0] * n_bins
        for b in range(n_bins):
            lvals = left_bins[b]
            rvals = right_bins[b]

            left_support[b] = len(lvals)
            right_support[b] = len(rvals)

            if len(lvals) >= self.min_per_side:
                left_raw[b] = median(lvals)
            if len(rvals) >= self.min_per_side:
                right_raw[b] = median(rvals)

        left_fit = left_raw
        right_fit = right_raw
        if self.enable_boundary_fit:
            left_fit = self._fit_boundary_series(left_raw)
            right_fit = self._fit_boundary_series(right_raw)

        width_ref = self._estimate_reference_width(left_fit, right_fit)
        self.width_ref_latched = width_ref

        accepted_both = 0
        accepted_inferred = 0
        rejected_width = 0
        bin_trace = []

        for b in range(n_bins):
            dl = left_fit[b]
            dr = right_fit[b]
            s_center = self.s_min + (b + 0.5) * self.ds_bin
            inferred_side = False
            decision = "skip_missing_sides"
            width_eval = None

            if dl is not None and dr is not None:
                width = dl - dr
                width_eval = width
                if width < self.width_min or width > self.width_max:
                    rejected_width += 1
                    decision = "reject_width_out_of_range"
                    bin_trace.append({
                        "bin": b,
                        "left_support": left_support[b],
                        "right_support": right_support[b],
                        "left_raw": left_raw[b],
                        "right_raw": right_raw[b],
                        "left_fit": left_fit[b],
                        "right_fit": right_fit[b],
                        "left_cone_indices": list(left_bin_cone_indices[b]),
                        "right_cone_indices": list(right_bin_cone_indices[b]),
                        "width_eval": width_eval,
                        "inferred": False,
                        "decision": decision
                    })
                    continue
                accepted_both += 1
                decision = "accept_both_sides"
            elif self.boundary_use_single_side_inference and (dl is not None or dr is not None):
                if s_center > self.boundary_inference_s_max:
                    decision = "reject_inferred_beyond_s_max"
                    bin_trace.append({
                        "bin": b,
                        "left_support": left_support[b],
                        "right_support": right_support[b],
                        "left_raw": left_raw[b],
                        "right_raw": right_raw[b],
                        "left_fit": left_fit[b],
                        "right_fit": right_fit[b],
                        "left_cone_indices": list(left_bin_cone_indices[b]),
                        "right_cone_indices": list(right_bin_cone_indices[b]),
                        "width_eval": width_eval,
                        "inferred": True,
                        "decision": decision,
                        "s_center": s_center
                    })
                    continue
                width = width_ref
                width_eval = width
                if width < self.width_min or width > self.width_max:
                    rejected_width += 1
                    decision = "reject_inferred_width_out_of_range"
                    bin_trace.append({
                        "bin": b,
                        "left_support": left_support[b],
                        "right_support": right_support[b],
                        "left_raw": left_raw[b],
                        "right_raw": right_raw[b],
                        "left_fit": left_fit[b],
                        "right_fit": right_fit[b],
                        "left_cone_indices": list(left_bin_cone_indices[b]),
                        "right_cone_indices": list(right_bin_cone_indices[b]),
                        "width_eval": width_eval,
                        "inferred": True,
                        "decision": decision
                    })
                    continue
                if dl is None:
                    dl = dr + width
                    inferred_side = True
                else:
                    dr = dl - width
                    inferred_side = True
                accepted_inferred += 1
                decision = "accept_single_side_inferred"
            else:
                bin_trace.append({
                    "bin": b,
                    "left_support": left_support[b],
                    "right_support": right_support[b],
                    "left_raw": left_raw[b],
                    "right_raw": right_raw[b],
                    "left_fit": left_fit[b],
                    "right_fit": right_fit[b],
                    "left_cone_indices": list(left_bin_cone_indices[b]),
                    "right_cone_indices": list(right_bin_cone_indices[b]),
                    "width_eval": width_eval,
                    "inferred": inferred_side,
                    "decision": decision
                })
                continue

            d_mid = 0.5 * (dl + dr)

            x = pref[0] + s_center * t[0] + d_mid * n[0]
            y = pref[1] + s_center * t[1] + d_mid * n[1]

            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            out.points.append(pt)
            out.width.append(dl - dr)

            var_l = robust_var(left_bins[b], fallback_d_var) if left_support[b] > 0 else fallback_d_var
            var_r = robust_var(right_bins[b], fallback_d_var) if right_support[b] > 0 else fallback_d_var
            sigma_d2 = 0.25 * (var_l + var_r)

            cov_xx = sigma_s2 * t[0] * t[0] + sigma_d2 * n[0] * n[0]
            cov_xy = sigma_s2 * t[0] * t[1] + sigma_d2 * n[0] * n[1]
            cov_yy = sigma_s2 * t[1] * t[1] + sigma_d2 * n[1] * n[1]

            out.cov_xy_flat.extend([cov_xx, cov_xy, cov_xy, cov_yy])
            out.id.append(len(out.id))

            support = left_support[b] + right_support[b]
            conf = min(1.0, float(support) / 6.0)
            if inferred_side:
                conf *= self.boundary_conf_inferred_penalty
            out.confidence.append(conf)
            bin_trace.append({
                "bin": b,
                "left_support": left_support[b],
                "right_support": right_support[b],
                "left_raw": left_raw[b],
                "right_raw": right_raw[b],
                "left_fit": left_fit[b],
                "right_fit": right_fit[b],
                "left_cone_indices": list(left_bin_cone_indices[b]),
                "right_cone_indices": list(right_bin_cone_indices[b]),
                "width_eval": width_eval,
                "inferred": inferred_side,
                "decision": decision,
                "s_center": s_center,
                "d_mid": d_mid,
                "out_idx": len(out.points) - 1,
                "out_x": x,
                "out_y": y,
                "out_conf": conf
            })

        # Recorte de profundidad dinámico por curvatura (preferir pocos puntos cercanos y fiables).
        depth_limit = s_max_eff
        if self.enable_curvature_depth_limit:
            kappa_est = self._estimate_forward_curvature(out, pref, t)
            depth_limit = min(s_max_eff, self._depth_limit_from_curvature(kappa_est, s_max_eff))
            self.last_kappa_est = kappa_est
            self.last_depth_limit = depth_limit
        out = self._clip_midpoints_by_s(out, msg.header, pref, t, depth_limit)

        out_to_publish = self._select_stable_midpoints(out, msg.header)
        out_to_publish, synthetic_added, synthetic_s_list = self._ensure_min_forward_midpoints(
            out_to_publish, msg.header, pref, t, n, sigma_s2, fallback_d_var, depth_limit
        )
        self.pub_midpoints.publish(out_to_publish)
        self._publish_debug_trace(
            msg=msg,
            n_bins=n_bins,
            s_max_eff=s_max_eff,
            depth_limit=depth_limit,
            kappa_est=self.last_kappa_est,
            cones_in_range=cones_in_range,
            left_raw=left_raw,
            right_raw=right_raw,
            left_fit=left_fit,
            right_fit=right_fit,
            width_ref=width_ref,
            out_raw=out,
            out_final=out_to_publish,
            accepted_both=accepted_both,
            accepted_inferred=accepted_inferred,
            rejected_width=rejected_width,
            synthetic_added=synthetic_added,
            synthetic_s_list=synthetic_s_list,
            pref=pref,
            t=t,
            n=n,
            cone_trace=cone_trace,
            bin_trace=bin_trace
        )

        # ==== Marker para RViz (X verde en cada midpoint) ====
        marker = Marker()
        marker.header = out_to_publish.header
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

        for pt in out_to_publish.points:
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

    def _mean_confidence(self, out):
        if not out.confidence:
            return 0.0
        return sum(out.confidence) / float(len(out.confidence))

    def _quality_score(self, out):
        # Score simple y robusto:
        # más puntos y más confianza media => mejor estabilidad geométrica.
        n = len(out.points)
        mean_conf = self._mean_confidence(out)
        return float(n) * mean_conf

    def _curvature_from_three_points(self, p1, p2, p3):
        a = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        b = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
        c = math.hypot(p3[0] - p1[0], p3[1] - p1[1])
        denom = a * b * c
        if denom < 1.0e-9:
            return 0.0
        area2 = abs(
            p1[0] * (p2[1] - p3[1]) +
            p2[0] * (p3[1] - p1[1]) +
            p3[0] * (p1[1] - p2[1])
        )
        return area2 / denom

    def _estimate_forward_curvature(self, out, pref, t):
        samples = []
        for p in out.points:
            relx = p.x - pref[0]
            rely = p.y - pref[1]
            s = relx * t[0] + rely * t[1]
            if s >= self.forward_s_min:
                samples.append((s, p.x, p.y))
        if len(samples) < 3:
            return 0.0
        samples.sort(key=lambda v: v[0])
        p1 = (samples[0][1], samples[0][2])
        p2 = (samples[1][1], samples[1][2])
        p3 = (samples[2][1], samples[2][2])
        return abs(self._curvature_from_three_points(p1, p2, p3))

    def _depth_limit_from_curvature(self, kappa_abs, s_max_eff):
        lo = max(1.0e-6, self.curvature_depth_kappa_low)
        hi = max(lo + 1.0e-6, self.curvature_depth_kappa_high)
        dmin = max(self.forward_s_min, self.curvature_depth_min)
        dmax = max(dmin, min(self.curvature_depth_max, s_max_eff))

        if kappa_abs <= lo:
            target = dmax
        elif kappa_abs >= hi:
            target = dmin
        else:
            alpha = (kappa_abs - lo) / (hi - lo)
            target = dmax + (dmin - dmax) * alpha

        # Suavizado temporal simple para evitar cambios bruscos de horizonte.
        alpha_lp = min(1.0, max(0.0, self.curvature_depth_alpha))
        blended = (1.0 - alpha_lp) * self.last_depth_limit + alpha_lp * target
        return max(self.forward_s_min, min(s_max_eff, blended))

    def _clip_midpoints_by_s(self, out, header, pref, t, s_limit):
        clipped = MidpointArray()
        clipped.header = header
        clipped.points = []
        clipped.width = []
        clipped.cov_xy_flat = []
        clipped.id = []
        clipped.confidence = []

        for i, p in enumerate(out.points):
            relx = p.x - pref[0]
            rely = p.y - pref[1]
            s = relx * t[0] + rely * t[1]
            if s > s_limit:
                continue

            q = Point()
            q.x = p.x
            q.y = p.y
            q.z = p.z
            clipped.points.append(q)

            clipped.width.append(out.width[i] if i < len(out.width) else 0.0)
            clipped.id.append(out.id[i] if i < len(out.id) else len(clipped.id))
            clipped.confidence.append(out.confidence[i] if i < len(out.confidence) else 0.0)

            cidx = 4 * i
            if cidx + 3 < len(out.cov_xy_flat):
                clipped.cov_xy_flat.extend(out.cov_xy_flat[cidx:cidx+4])
            else:
                clipped.cov_xy_flat.extend([0.0, 0.0, 0.0, 0.0])
        return clipped

    def _copy_series(self, values):
        copied = []
        for v in values:
            copied.append(v)
        return copied

    def _interpolate_small_gaps(self, values, max_gap):
        out = self._copy_series(values)
        n = len(out)
        i = 0
        while i < n:
            if out[i] is not None:
                i += 1
                continue
            j = i
            while j < n and out[j] is None:
                j += 1
            gap_len = j - i
            left_i = i - 1
            right_i = j
            can_fill = (
                left_i >= 0 and right_i < n and
                out[left_i] is not None and out[right_i] is not None and
                gap_len <= max_gap
            )
            if can_fill:
                lv = out[left_i]
                rv = out[right_i]
                for k in range(gap_len):
                    alpha = float(k + 1) / float(gap_len + 1)
                    out[i + k] = (1.0 - alpha) * lv + alpha * rv
            i = j
        return out

    def _smooth_series(self, values, window_bins):
        out = self._copy_series(values)
        n = len(values)
        if n == 0:
            return out
        half = max(0, int(window_bins // 2))
        for i in range(n):
            if values[i] is None:
                continue
            acc = 0.0
            cnt = 0
            j0 = max(0, i - half)
            j1 = min(n - 1, i + half)
            for j in range(j0, j1 + 1):
                if values[j] is None:
                    continue
                acc += values[j]
                cnt += 1
            if cnt > 0:
                out[i] = acc / float(cnt)
        return out

    def _fit_boundary_series(self, raw_values):
        interp = self._interpolate_small_gaps(
            raw_values,
            max(0, self.boundary_interp_max_gap_bins)
        )
        return self._smooth_series(interp, max(1, self.boundary_smooth_window_bins))

    def _estimate_reference_width(self, left_fit, right_fit):
        widths = []
        n = min(len(left_fit), len(right_fit))
        for i in range(n):
            dl = left_fit[i]
            dr = right_fit[i]
            if dl is None or dr is None:
                continue
            w = dl - dr
            if w < self.width_min or w > self.width_max:
                continue
            widths.append(w)
        if len(widths) >= self.boundary_min_bins_for_width_ref:
            return median(widths)
        return max(self.width_min, min(self.width_max, self.width_ref_latched))

    def _is_good_candidate(self, out):
        n = len(out.points)
        if n < self.min_points_to_latch:
            return False
        mean_conf = self._mean_confidence(out)
        if mean_conf < self.min_mean_conf_to_latch:
            return False
        if self._quality_score(out) < self.min_score_to_latch:
            return False
        return True

    def _copy_midpoints(self, out, header):
        copied = MidpointArray()
        copied.header = header
        copied.points = []
        copied.width = list(out.width)
        copied.cov_xy_flat = list(out.cov_xy_flat)
        copied.id = list(out.id)
        copied.confidence = list(out.confidence)

        for p in out.points:
            q = Point()
            q.x = p.x
            q.y = p.y
            q.z = p.z
            copied.points.append(q)
        return copied

    def _latched_expired(self, now):
        if self.max_hold_seconds <= 0.0:
            return False
        if self.latched_time is None:
            return False
        age = (now - self.latched_time).to_sec()
        return age > self.max_hold_seconds

    def _forward_s_values(self, out, pref, t):
        s_vals = []
        for p in out.points:
            relx = p.x - pref[0]
            rely = p.y - pref[1]
            s = relx * t[0] + rely * t[1]
            if s >= self.forward_s_min:
                s_vals.append(s)
        return sorted(s_vals)

    def _estimate_width_for_fallback(self, out):
        if out.width:
            return max(self.width_min, min(self.width_max, median(out.width)))
        if self.latched_midpoints is not None and self.latched_midpoints.width:
            return max(self.width_min, min(self.width_max, median(self.latched_midpoints.width)))
        return max(self.width_min, min(self.width_max, self.fallback_width))

    def _append_synthetic_midpoint(self, out, s_target, width, pref, t, n, sigma_s2, fallback_d_var):
        # Punto medio sintético en d=0 respecto al coche, para mantener control mínimo.
        x = pref[0] + s_target * t[0]
        y = pref[1] + s_target * t[1]

        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.0
        out.points.append(pt)
        out.width.append(width)

        sigma_d2 = fallback_d_var
        cov_xx = sigma_s2 * t[0] * t[0] + sigma_d2 * n[0] * n[0]
        cov_xy = sigma_s2 * t[0] * t[1] + sigma_d2 * n[0] * n[1]
        cov_yy = sigma_s2 * t[1] * t[1] + sigma_d2 * n[1] * n[1]
        out.cov_xy_flat.extend([cov_xx, cov_xy, cov_xy, cov_yy])
        out.id.append(len(out.id))
        out.confidence.append(0.15)  # baja confianza: es fallback de control, no dato fuerte.

    def _ensure_min_forward_midpoints(self, out, header, pref, t, n, sigma_s2, fallback_d_var, s_max_eff):
        if not self.enable_forward_guard or self.min_forward_midpoints <= 0:
            return out, 0, []

        ensured = self._copy_midpoints(out, header)
        s_vals = [s for s in self._forward_s_values(ensured, pref, t) if s <= s_max_eff]
        if len(s_vals) >= self.min_forward_midpoints:
            return ensured, 0, []

        width = self._estimate_width_for_fallback(ensured)
        spacing = max(0.1, self.forward_spacing)
        next_s = max(self.forward_s_min, self.s_min + 0.5 * self.ds_bin)
        if s_vals:
            next_s = max(next_s, s_vals[-1] + spacing)

        missing = self.min_forward_midpoints - len(s_vals)
        synthetic_s_list = []
        for _ in range(missing):
            if next_s > s_max_eff:
                break
            self._append_synthetic_midpoint(
                ensured, next_s, width, pref, t, n, sigma_s2, fallback_d_var
            )
            synthetic_s_list.append(next_s)
            next_s += spacing

        added = len(synthetic_s_list)
        if added < missing:
            rospy.logwarn_throttle(
                1.0,
                "midpoints: limite frontal (%.2f m) impide alcanzar min_forward=%d (solo +%d sinteticos).",
                s_max_eff, self.min_forward_midpoints, added
            )

        rospy.logwarn_throttle(
            1.0,
            "midpoints: forward-guard activo, anadidos %d midpoint(s) sinteticos para control minimo.",
            added
        )
        return ensured, added, synthetic_s_list

    def _publish_debug_trace(self, msg, n_bins, s_max_eff, depth_limit, kappa_est, cones_in_range,
                             left_raw, right_raw, left_fit, right_fit,
                             width_ref, out_raw, out_final,
                             accepted_both, accepted_inferred, rejected_width,
                             synthetic_added, synthetic_s_list, pref, t, n,
                             cone_trace, bin_trace):
        if not self.debug_enable:
            return
        self.debug_counter += 1
        if self.debug_report_every_n > 1 and (self.debug_counter % self.debug_report_every_n) != 0:
            return

        left_raw_cnt = sum(1 for v in left_raw if v is not None)
        right_raw_cnt = sum(1 for v in right_raw if v is not None)
        left_fit_cnt = sum(1 for v in left_fit if v is not None)
        right_fit_cnt = sum(1 for v in right_fit if v is not None)
        forward_cnt = len(self._forward_s_values(out_final, pref, t))

        lines = []
        lines.append("[MIDPOINTS TRACE] seq=%d t=%.3f frame=%s" %
                     (msg.header.seq, msg.header.stamp.to_sec(), msg.header.frame_id))
        lines.append("1) Conos de entrada: total=%d, usados_en_rango=%d, bins=%d" %
                     (len(msg.cones), cones_in_range, n_bins))
        lines.append("2) Bordes raw por bin: left=%d, right=%d (min_per_side=%d)" %
                     (left_raw_cnt, right_raw_cnt, self.min_per_side))
        lines.append("3) Bordes ajustados: left=%d, right=%d, width_ref=%.3f m" %
                     (left_fit_cnt, right_fit_cnt, width_ref))
        lines.append(
            "4) Midpoints candidatos: total=%d (ambos_lados=%d, inferidos=%d, rechazados_por_ancho=%d)" %
            (len(out_raw.points), accepted_both, accepted_inferred, rejected_width)
        )
        lines.append(
            "5) Estabilidad: latch='%s', score=%.3f, salida=%d" %
            (self.last_latch_reason, self.latched_score, len(out_final.points))
        )
        lines.append(
            "6) Control minimo delante: forward=%d, sinteticos_anadidos=%d (min=%d, s_min=%.2f m)" %
            (forward_cnt, synthetic_added, self.min_forward_midpoints, self.forward_s_min)
        )
        lines.append(
            "6b) Profundidad dinamica: kappa_est=%.4f  depth_limit=%.2f m" %
            (kappa_est, depth_limit)
        )

        if self.last_latch_metrics:
            lines.append(
                "7) Latch vars: cand_score=%.3f, latched_score=%.3f, cand_good=%s, "
                "thr_improve=%.3f, thr_not_worse=%.3f, can_refresh=%s, expired=%s, update_period=%.3f" %
                (
                    self.last_latch_metrics.get("candidate_score", 0.0),
                    self.last_latch_metrics.get("latched_score", 0.0),
                    str(self.last_latch_metrics.get("candidate_good", False)),
                    self.last_latch_metrics.get("improve_threshold", 0.0),
                    self.last_latch_metrics.get("not_worse_threshold", 0.0),
                    str(self.last_latch_metrics.get("can_refresh_by_rate", False)),
                    str(self.last_latch_metrics.get("expired", False)),
                    self.last_latch_metrics.get("update_period", 0.0),
                )
            )

        if self.debug_level == "full":
            lines.append("")
            lines.append("===== 8) PARAMETROS ACTIVOS =====")
            lines.append("- Espacio de busqueda:")
            lines.append("  - s_min=%.2f, s_max=%.2f, ds_bin=%.2f, min_per_side=%d" %
                         (self.s_min, self.s_max, self.ds_bin, self.min_per_side))
            lines.append("  - s_max_eff=%.2f (limitado por forward_length_limit=%.2f)" %
                         (s_max_eff, self.forward_length_limit))
            lines.append("  - depth_limit=%.2f (por curvatura local)" % depth_limit)
            lines.append("  - width_min=%.2f, width_max=%.2f, width_ref=%.3f" %
                         (self.width_min, self.width_max, width_ref))
            lines.append("  - track_width_limit=%.2f (filtro |d| para incluir cono en bin)" %
                         self.track_width_limit)
            lines.append("- Ajuste de bordes:")
            lines.append("  - enable=%s, smooth_window_bins=%d, interp_max_gap_bins=%d" %
                         (str(self.enable_boundary_fit),
                          self.boundary_smooth_window_bins,
                          self.boundary_interp_max_gap_bins))
            lines.append("  - single_side_inference=%s, inferred_penalty=%.2f" %
                         (str(self.boundary_use_single_side_inference),
                          self.boundary_conf_inferred_penalty))
            lines.append("  - inference_s_max=%.2f" % self.boundary_inference_s_max)
            lines.append("- Estabilidad:")
            lines.append("  - enable=%s, min_pts=%d, min_mean_conf=%.2f, min_score=%.2f" %
                         (str(self.enable_latching),
                          self.min_points_to_latch,
                          self.min_mean_conf_to_latch,
                          self.min_score_to_latch))
            lines.append("  - improve_ratio=%.2f, not_worse_ratio=%.2f, max_update_hz=%.2f, max_hold_s=%.2f" %
                         (self.replace_if_score_improves_ratio,
                          self.replace_if_score_not_worse_ratio,
                          self.max_update_rate_hz,
                          self.max_hold_seconds))
            lines.append("- Guard de control:")
            lines.append("  - enable=%s, min_forward=%d, forward_s_min=%.2f, forward_spacing=%.2f, fallback_width=%.2f" %
                         (str(self.enable_forward_guard),
                          self.min_forward_midpoints,
                          self.forward_s_min,
                          self.forward_spacing,
                          self.fallback_width))
            lines.append("  - curvature_depth_limit: enable=%s kappa_low=%.3f kappa_high=%.3f depth_min=%.2f depth_max=%.2f alpha=%.2f" %
                         (str(self.enable_curvature_depth_limit),
                          self.curvature_depth_kappa_low,
                          self.curvature_depth_kappa_high,
                          self.curvature_depth_min,
                          self.curvature_depth_max,
                          self.curvature_depth_alpha))
            lines.append("  - synthetic_s=%s" % str(["%.2f" % s for s in synthetic_s_list]))

            lines.append("")
            lines.append("===== 9) LISTA VERTICAL DE CONOS (indice RViz) =====")
            for c in cone_trace:
                lines.append(
                    "- cone[%d]" %
                    c["rviz_idx"]
                )
                lines.append(
                    "  - id=%d, source=%s, confidence=%.2f" %
                    (c["cone_id"], c["source"], c["confidence"])
                )
                lines.append(
                    "  - xy=(%.3f, %.3f), s=%.3f, d=%.3f" %
                    (c["x"], c["y"], c["s"], c["d"])
                )
                lines.append(
                    "  - side=%s, in_range=%s, bin=%d" %
                    (
                        c["side"], str(c["in_range"]), c["bin"]
                    )
                )
                lines.append(
                    "  - in_track_width=%s (|d|<=%.2f)" %
                    (str(c.get("in_track_width", True)), self.track_width_limit)
                )
            lines.append("")
            lines.append("===== 10) PROCESO POR BIN =====")
            for bt in bin_trace:
                lines.append(
                    "- bin %d" % bt.get("bin", -1)
                )
                lines.append(
                    "  - soportes: L=%d, R=%d" %
                    (bt.get("left_support", 0), bt.get("right_support", 0))
                )
                lines.append(
                    "  - conos por indice RViz: L=%s, R=%s" %
                    (str(bt.get("left_cone_indices", [])), str(bt.get("right_cone_indices", [])))
                )
                lines.append(
                    "  - raw:  L=%s, R=%s" %
                    (str(bt.get("left_raw", None)), str(bt.get("right_raw", None)))
                )
                lines.append(
                    "  - fit:  L=%s, R=%s" %
                    (str(bt.get("left_fit", None)), str(bt.get("right_fit", None)))
                )
                lines.append(
                    "  - width_eval=%s, inferred=%s" %
                    (str(bt.get("width_eval", None)), str(bt.get("inferred", False)))
                )
                lines.append(
                    "  - decision=%s" % bt.get("decision", "none")
                )
                lines.append(
                    "  - salida parcial: s_center=%s, d_mid=%s, out_idx=%s, out_xy=(%s,%s), out_conf=%s" %
                    (
                        str(bt.get("s_center", None)),
                        str(bt.get("d_mid", None)),
                        str(bt.get("out_idx", None)),
                        str(bt.get("out_x", None)),
                        str(bt.get("out_y", None)),
                        str(bt.get("out_conf", None))
                    )
                )
            lines.append("")
            lines.append("===== 11) MIDPOINTS DE SALIDA FINAL =====")
            for i, p in enumerate(out_final.points):
                w = out_final.width[i] if i < len(out_final.width) else 0.0
                c = out_final.confidence[i] if i < len(out_final.confidence) else 0.0
                relx = p.x - pref[0]
                rely = p.y - pref[1]
                s = relx * t[0] + rely * t[1]
                d = relx * n[0] + rely * n[1]
                cidx = 4 * i
                cov = [0.0, 0.0, 0.0, 0.0]
                if cidx + 3 < len(out_final.cov_xy_flat):
                    cov = out_final.cov_xy_flat[cidx:cidx+4]
                lines.append(
                    "- mp[%d]" % i
                )
                lines.append("  - s=%.3f, d=%.3f" % (s, d))
                lines.append("  - xy=(%.3f, %.3f)" % (p.x, p.y))
                lines.append("  - width=%.3f, confidence=%.3f" % (w, c))
                lines.append("  - cov=[%.5f, %.5f, %.5f, %.5f]" % (cov[0], cov[1], cov[2], cov[3]))

            lines.append("")
            lines.append("===== 12) EXPLICACION DEL PRIMER MIDPOINT =====")
            if len(out_final.points) == 0:
                lines.append("- No hay midpoint final; no se puede explicar mp[0].")
            else:
                p0 = out_final.points[0]
                relx0 = p0.x - pref[0]
                rely0 = p0.y - pref[1]
                s0 = relx0 * t[0] + rely0 * t[1]
                d0 = relx0 * n[0] + rely0 * n[1]
                src_bin = None
                for bt in bin_trace:
                    if bt.get("out_idx", None) == 0:
                        src_bin = bt
                        break
                lines.append("- mp[0] final:")
                lines.append("  - xy=(%.3f, %.3f), s=%.3f, d=%.3f" % (p0.x, p0.y, s0, d0))
                if src_bin is None:
                    lines.append("  - origen: no encontrado en trazas por bin (posible punto sintetico de guard).")
                else:
                    lines.append("  - origen: bin=%d, decision=%s" %
                                 (src_bin.get("bin", -1), src_bin.get("decision", "unknown")))
                    lines.append("  - valores de decision:")
                    lines.append("    - left_raw=%s, right_raw=%s" %
                                 (str(src_bin.get("left_raw", None)), str(src_bin.get("right_raw", None))))
                    lines.append("    - left_fit=%s, right_fit=%s" %
                                 (str(src_bin.get("left_fit", None)), str(src_bin.get("right_fit", None))))
                    lines.append("    - width_eval=%s (umbral [%0.2f, %0.2f])" %
                                 (str(src_bin.get("width_eval", None)), self.width_min, self.width_max))
                    lines.append("    - inferred=%s, s_center=%s, d_mid=%s, out_conf=%s" %
                                 (str(src_bin.get("inferred", False)),
                                  str(src_bin.get("s_center", None)),
                                  str(src_bin.get("d_mid", None)),
                                  str(src_bin.get("out_conf", None))))

        if self.debug_level == "detailed":
            samples = []
            max_show = min(self.debug_max_bins, len(out_final.points))
            for i in range(max_show):
                p = out_final.points[i]
                w = out_final.width[i] if i < len(out_final.width) else 0.0
                c = out_final.confidence[i] if i < len(out_final.confidence) else 0.0
                relx = p.x - pref[0]
                rely = p.y - pref[1]
                s = relx * t[0] + rely * t[1]
                samples.append("#%d s=%.2f x=%.2f y=%.2f w=%.2f conf=%.2f" %
                               (i, s, p.x, p.y, w, c))
            if samples:
                lines.append("7) Muestras salida: " + " | ".join(samples))

        dbg_text = "\n".join(lines)
        dbg = String()
        dbg.data = dbg_text
        self.pub_debug.publish(dbg)
        if self.debug_console_enable:
            rospy.loginfo("\n%s", dbg_text)

    def _select_stable_midpoints(self, candidate, header):
        if not self.enable_latching:
            self.last_latch_reason = "latching_disabled"
            return candidate

        now = rospy.Time.now()
        candidate_good = self._is_good_candidate(candidate)
        candidate_score = self._quality_score(candidate)

        # Si no hay latched previo, aceptar solo candidatos "buenos".
        if self.latched_midpoints is None:
            if candidate_good:
                self.latched_midpoints = self._copy_midpoints(candidate, header)
                self.latched_score = candidate_score
                self.latched_time = now
                self.last_replace_time = now
                self.last_latch_reason = "init_accept_good_candidate"
                return self._copy_midpoints(self.latched_midpoints, header)
            self.last_latch_reason = "init_no_latch_candidate_not_good"
            return candidate

        # Si hay latched, mantener por defecto salvo mejora clara.
        improve_threshold = self.latched_score * self.replace_if_score_improves_ratio
        not_worse_threshold = self.latched_score * self.replace_if_score_not_worse_ratio
        update_period = 0.2
        if self.max_update_rate_hz > 1e-6:
            update_period = 1.0 / self.max_update_rate_hz
        can_refresh_by_rate = False
        if self.last_replace_time is None:
            can_refresh_by_rate = True
        else:
            can_refresh_by_rate = (now - self.last_replace_time).to_sec() >= update_period

        replace_with_candidate = (
            candidate_good and
            (
                candidate_score >= improve_threshold or
                self._latched_expired(now) or
                (candidate_score >= not_worse_threshold and can_refresh_by_rate)
            )
        )
        self.last_latch_metrics = {
            "candidate_score": candidate_score,
            "latched_score": self.latched_score,
            "candidate_good": candidate_good,
            "improve_threshold": improve_threshold,
            "not_worse_threshold": not_worse_threshold,
            "can_refresh_by_rate": can_refresh_by_rate,
            "expired": self._latched_expired(now),
            "update_period": update_period
        }

        if replace_with_candidate:
            reason = "replace_improved"
            if self._latched_expired(now):
                reason = "replace_timeout_expired"
            elif candidate_score >= not_worse_threshold and can_refresh_by_rate and candidate_score < improve_threshold:
                reason = "replace_periodic_refresh_not_worse"
            prev_score = self.latched_score
            self.latched_midpoints = self._copy_midpoints(candidate, header)
            self.latched_score = candidate_score
            self.latched_time = now
            self.last_replace_time = now
            self.last_latch_reason = reason
            rospy.loginfo_throttle(
                2.0,
                "midpoints: actualizado latched (score %.2f -> %.2f)",
                prev_score, candidate_score
            )
            return self._copy_midpoints(self.latched_midpoints, header)

        # Mantener estable el conjunto previo (actualizando solo header/timestamp).
        self.last_latch_reason = "keep_previous_latched"
        return self._copy_midpoints(self.latched_midpoints, header)


if __name__ == "__main__":
    rospy.init_node("midpoints")
    MidpointsNode()
    rospy.spin()
