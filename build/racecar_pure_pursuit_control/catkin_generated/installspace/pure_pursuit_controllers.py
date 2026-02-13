#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class PPCtrl(object):
    def __init__(self):
        # -------- Params --------
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))
        self.delta_topic = rospy.get_param("~delta_topic", "/pure_pursuit_target_selector/delta")
        self.kappa_topic = rospy.get_param("~kappa_topic", "/pure_pursuit_target_selector/kappa")
        self.ld_topic    = rospy.get_param("~ld_topic",    "/pure_pursuit_target_selector/Ld")
        self.odom_topic  = rospy.get_param("~odom_topic",  "/vesc/odom")
        self.ack_topic   = rospy.get_param("~ackermann_topic", "/navigation/ackermann_cmd")

        self.L = float(rospy.get_param("~wheelbase_L", 0.325))
        self.delta_max = math.radians(float(rospy.get_param("~delta_max_deg", 28.0)))

        self.v_max = float(rospy.get_param("~v_max", 3.0))
        self.v_min = float(rospy.get_param("~v_min", 0.2))
        self.a_acc = float(rospy.get_param("~a_long_acc_max", 1.2))
        self.a_dec = float(rospy.get_param("~a_long_dec_max", 2.0))

        self.a_lat_max = float(rospy.get_param("~a_lat_max", 4.0))
        self.eps_kappa = float(rospy.get_param("~eps_kappa", 0.05))

        self.use_headway = bool(rospy.get_param("~use_headway", True))
        self.T_h = float(rospy.get_param("~T_h", 0.7))

        self.use_lowpass = bool(rospy.get_param("~use_lowpass", True))
        self.tau_v_ref = float(rospy.get_param("~tau_v_ref", 0.2))

        self.pass_through_when_idle = bool(rospy.get_param("~pass_through_when_idle", False))

        # -------- State --------
        self.delta = None         # [rad]
        self.kappa = None         # [1/m]
        self.Ld    = None         # [m]
        self.v_meas = 0.0         # [m/s] (de odom)
        self.v_cmd  = 0.0         # [m/s] (estado interno tras rampas/filtro)
        self.last_t = rospy.Time.now()

        # -------- IO --------
        rospy.Subscriber(self.delta_topic, Float64, self._cb_delta, queue_size=10)
        rospy.Subscriber(self.kappa_topic, Float64, self._cb_kappa, queue_size=10)
        rospy.Subscriber(self.ld_topic,    Float64, self._cb_ld,    queue_size=10)
        rospy.Subscriber(self.odom_topic,  Odometry, self._cb_odom, queue_size=20)
        self.pub_ack = rospy.Publisher(self.ack_topic, AckermannDriveStamped, queue_size=20)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self._on_timer)

        rospy.loginfo("[pp_control] ready: v_max=%.2f a_lat_max=%.2f T_h=%.2f" %
                      (self.v_max, self.a_lat_max, self.T_h))

    # ---- Callbacks ----
    def _cb_delta(self, msg): self.delta = float(msg.data)
    def _cb_kappa(self, msg): self.kappa = float(msg.data)
    def _cb_ld(self, msg):    self.Ld    = float(msg.data)

    def _cb_odom(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v_meas = math.hypot(vx, vy)

    # ---- Core ----
    def _on_timer(self, _evt):
        now = rospy.Time.now()
        dt = (now - self.last_t).to_sec()
        if dt <= 0.0: dt = 1.0 / self.rate_hz
        self.last_t = now

        if self.delta is None or self.kappa is None or self.Ld is None:
            return  # aún no hay datos

        # 1) Saturación coherente de steering
        delta_cmd = clamp(self.delta, -self.delta_max, self.delta_max)

        # 2) Límites de velocidad seguros
        # 2a) Curvatura → límite por aceleración lateral: v_kappa_max = sqrt(a_lat_max / (|kappa| + eps))
        kappa_abs = abs(self.kappa) if self.kappa is not None else 0.0
        v_kappa_max = math.sqrt(max(0.0, self.a_lat_max / (kappa_abs + self.eps_kappa))) if self.a_lat_max > 0.0 else self.v_max

        # 2b) Time-headway usando Ld: v_h_max = Ld / T_h
        if self.use_headway and self.T_h > 1e-6 and self.Ld is not None:
            v_h_max = max(0.0, self.Ld / self.T_h)
        else:
            v_h_max = self.v_max

        # 2c) Límite absoluto
        v_target = min(self.v_max, v_kappa_max, v_h_max)

        # 3) Rampa por límites de aceleración (modelo bang-bang saturado)
        dv = v_target - self.v_cmd
        dv_clip = clamp(dv, -self.a_dec * dt, self.a_acc * dt)
        v_cmd_ramped = clamp(self.v_cmd + dv_clip, self.v_min, self.v_max)

        # 4) Filtro 1er orden opcional (para eliminar jitter)
        if self.use_lowpass and self.tau_v_ref > 1e-6:
            alpha = dt / (self.tau_v_ref + dt)
            self.v_cmd = (1.0 - alpha) * self.v_cmd + alpha * v_cmd_ramped
        else:
            self.v_cmd = v_cmd_ramped

        # 5) Componer y publicar Ackermann
        ack = AckermannDriveStamped()
        ack.header.stamp = now
        ack.header.frame_id = "base_link"

        ack.drive.steering_angle = float(delta_cmd)
        # Si se desea "passthrough" cuando el target es mínimo:
        if self.pass_through_when_idle and v_target <= self.v_min:
            ack.drive.speed = float(self.v_meas)
        else:
            ack.drive.speed = float(self.v_cmd)

        self.pub_ack.publish(ack)

def main():
    rospy.init_node("pure_pursuit_controller")
    PPCtrl()
    rospy.spin()

if __name__ == "__main__":
    main()
