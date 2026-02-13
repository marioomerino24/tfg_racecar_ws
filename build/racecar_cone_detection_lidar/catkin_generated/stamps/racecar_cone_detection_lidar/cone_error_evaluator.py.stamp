#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy

from racecar_cone_msgs.msg import ConeArray, Cone


class ConeErrorEvaluator(object):
    def __init__(self):
        # Parámetros (por si quieres cambiarlos por rosparam más adelante)
        self.raw_topic   = rospy.get_param("~raw_topic", "/cones/raw")
        self.gt_topic    = rospy.get_param("~gt_topic", "/track/cones_zigzag_cones")
        self.min_gt_src  = rospy.get_param("~gt_source", "spawn")  # fuente "real" preferida
        self.eps_rel     = rospy.get_param("~eps_rel", 1e-6)

        self.last_raw = None
        self.last_gt  = None

        self.sub_raw = rospy.Subscriber(self.raw_topic,
                                        ConeArray,
                                        self.raw_cb,
                                        queue_size=1)

        self.sub_gt  = rospy.Subscriber(self.gt_topic,
                                        ConeArray,
                                        self.gt_cb,
                                        queue_size=1)

        rospy.loginfo("[cone_error_evaluator] Nodo iniciado.")
        rospy.loginfo("  raw_topic  = %s", self.raw_topic)
        rospy.loginfo("  gt_topic   = %s", self.gt_topic)
        rospy.loginfo("  gt_source  = %s", self.min_gt_src)

    # ===================== callbacks =====================

    def raw_cb(self, msg):
        self.last_raw = msg
        self.process_if_ready()

    def gt_cb(self, msg):
        self.last_gt = msg
        self.process_if_ready()

    # ===================== lógica principal =====================

    def process_if_ready(self):
        if self.last_raw is None or self.last_gt is None:
            return
        self.compute_errors(self.last_raw, self.last_gt)

    def compute_errors(self, raw_msg, gt_msg):
        # Filtrar conos medidos (LiDAR)
        meas_cones = [c for c in raw_msg.cones
                      if c.source in ("lidar_hyper", "lidar_centroid")]

        # Filtrar conos "reales"
        gt_cones = [c for c in gt_msg.cones if c.source == self.min_gt_src]
        # Si no hay con source=spawn, usamos todos como fallback
        if not gt_cones:
            gt_cones = list(gt_msg.cones)

        if not meas_cones or not gt_cones:
            rospy.logwarn_throttle(1.0,
                "[cone_error_evaluator] No hay suficientes conos para comparar "
                "(meas=%d, gt=%d).",
                len(meas_cones), len(gt_cones))
            return

        rospy.loginfo("========== NUEVA EVALUACIÓN DE ERRORES ==========")
        rospy.loginfo("  #meas (lidar) = %d,  #gt (spawn) = %d",
                      len(meas_cones), len(gt_cones))

        for mc in meas_cones:
            best_gt, dist_err = self.find_closest_gt(mc, gt_cones)
            if best_gt is None:
                continue

            # Posiciones
            mx, my = mc.p.x, mc.p.y
            gx, gy = best_gt.p.x, best_gt.p.y

            # Diferencias absolutas
            dx = mx - gx
            dy = my - gy

            # Distancias absolutas
            d_gt = math.hypot(gx, gy)

            # Errores relativos (con protección frente a división por ~0)
            ex_rel = float('nan')
            ey_rel = float('nan')
            d_rel  = float('nan')

            if abs(gx) > self.eps_rel:
                ex_rel = dx / gx
            if abs(gy) > self.eps_rel:
                ey_rel = dy / gy
            if d_gt > self.eps_rel:
                d_rel = dist_err / d_gt

            # Modo de estimación (hyper/centroid)
            if mc.source == "lidar_hyper":
                mode_str = "HYPER"
            elif mc.source == "lidar_centroid":
                mode_str = "CENTROID"
            else:
                mode_str = mc.source

            cone_id = mc.id if mc.id is not None else -1

            # Línea única con toda la información
            # id, modo, pos medida, pos real, difs, errores relativos y distancia
            rospy.loginfo(
                "[id=%3d, mode=%-8s] "
                "meas=(%.3f, %.3f)  real=(%.3f, %.3f)  "
                "dx=%.3f  dy=%.3f  "
                "err_rel_x=%.3f  err_rel_y=%.3f  "
                "dist_err=%.3f  dist_rel=%.3f",
                cone_id, mode_str,
                mx, my,
                gx, gy,
                dx, dy,
                ex_rel, ey_rel,
                dist_err, d_rel
            )

    # ===================== utilidades =====================

    def find_closest_gt(self, meas_cone, gt_cones):
        """
        Empareja cada cono LiDAR con el cono "real" más cercano (mínima
        distancia euclídea en el plano x-y). No se prohíbe reutilizar el
        mismo cono real para varios medidos (simple y suficiente para evaluar
        error por método).
        """
        mx, my = meas_cone.p.x, meas_cone.p.y
        best = None
        best_d2 = float('inf')

        for gc in gt_cones:
            gx, gy = gc.p.x, gc.p.y
            dx = mx - gx
            dy = my - gy
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best = gc

        if best is None:
            return None, 0.0

        return best, math.sqrt(best_d2)


def main():
    rospy.init_node("cone_error_evaluator", anonymous=True)
    node = ConeErrorEvaluator()
    rospy.spin()


if __name__ == "__main__":
    main()
