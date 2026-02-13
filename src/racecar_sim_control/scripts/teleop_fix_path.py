#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import math
import select
import termios
import tty

import rospy
import tf.transformations as tft

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String


def yaw_from_quat(q):
    e = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]


def getKey(settings, timeout):
    """
    Lee una tecla del stdin en modo raw con timeout.
    Devuelve '' si no se ha pulsado nada en 'timeout' segundos.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class CenterlineTeleop(object):
    def __init__(self):
        ns = "~"

        # --- Parámetros ---
        self.path_topic   = rospy.get_param(ns + "path_topic",
                                            "/track/centerline_zigzag")
        self.model_name   = rospy.get_param(ns + "model_name", "racecar")
        self.rate_hz      = float(rospy.get_param(ns + "rate", 50.0))

        # Parámetro de control: frecuencia de avance sobre los puntos [puntos/s]
        self.freq_step    = float(rospy.get_param(ns + "freq_step", 1.0))
        self.freq_max     = float(rospy.get_param(ns + "freq_max", 20.0))
        self.loop         = bool(rospy.get_param(ns + "loop", False))

        # Tópico de debug (String)
        self.debug_topic  = rospy.get_param(ns + "debug_topic",
                                            "/teleop/ackermann_cmd_debug")

        # Estado del recorrido sobre el Path
        self.path_poses = []   # lista de PoseStamped
        self.N = 0
        self.ds = 0.1          # estimación de espaciado [m]
        self.idx_f = 0.0       # índice continuo sobre la lista de puntos
        self.freq = 0.0        # frecuencia de avance [puntos/s] (controlada con teclas)

        self.last_time = rospy.Time.now()

        # Subscripción al Path
        self.sub_path = rospy.Subscriber(self.path_topic, Path,
                                         self.path_callback, queue_size=1)

        # Cliente al servicio de Gazebo
        rospy.loginfo("[centerline_teleop] Esperando /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state",
                                                SetModelState)

        # Publisher de debug (String)
        self.pub_debug = rospy.Publisher(self.debug_topic,
                                         String,
                                         queue_size=10)

        rospy.loginfo("[centerline_teleop] Inicializado. path_topic=%s, modelo=%s",
                      self.path_topic, self.model_name)
        self.print_help()

    def print_help(self):
        rospy.loginfo("=== TECLAS ===")
        rospy.loginfo("  w : aumentar frecuencia de avance (puntos/s)")
        rospy.loginfo("  s : reducir frecuencia de avance (puntos/s)")
        rospy.loginfo("  ESPACIO : parar (freq=0)")
        rospy.loginfo("  q / CTRL-C : salir")

    def path_callback(self, msg):
        self.path_poses = list(msg.poses)
        self.N = len(self.path_poses)
        self.idx_f = 0.0

        if self.N >= 2:
            # Estimamos ds promedio (distancia entre puntos)
            dsum = 0.0
            nseg = 0
            for i in range(self.N - 1):
                p0 = self.path_poses[i].pose.position
                p1 = self.path_poses[i + 1].pose.position
                dx = p1.x - p0.x
                dy = p1.y - p0.y
                d = math.hypot(dx, dy)
                if d > 1e-4:
                    dsum += d
                    nseg += 1
            if nseg > 0:
                self.ds = dsum / float(nseg)
        rospy.loginfo("[centerline_teleop] Recibido Path: N=%d, ds≈%.3f m",
                      self.N, self.ds)

    def update_freq_from_key(self, key):
        # w para subir, s para bajar, espacio para parar
        if key == 'w':
            self.freq += self.freq_step
            if self.freq > self.freq_max:
                self.freq = self.freq_max
            rospy.loginfo("freq = %.3f puntos/s", self.freq)
        elif key == 's':
            self.freq -= self.freq_step
            if self.freq < -self.freq_max:
                self.freq = -self.freq_max
            rospy.loginfo("freq = %.3f puntos/s", self.freq)
        elif key == ' ':
            self.freq = 0.0
            rospy.loginfo("PARADO: freq = 0.0 puntos/s")

    def step(self):
        """
        Un paso de control:
        - avanza índice en función de freq
        - manda set_model_state
        - publica debug (String con freq, v y yaw)
        """
        # Si no hay path, nada que hacer
        if self.N < 1:
            return

        # Avance en la traza
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt < 0.0:
            dt = 0.0
        self.last_time = now

        # velocidad lineal aproximada (m/s) a partir de freq [puntos/s]
        v = self.freq * self.ds

        if abs(self.freq) >= 1e-6 and dt > 0.0:
            # incremento de índice (freq puntos/s -> puntos en dt)
            d_idx = self.freq * dt
            self.idx_f += d_idx

            # Manejo de límites
            if self.loop:
                # Modo loop: wrap-around
                while self.idx_f >= self.N:
                    self.idx_f -= self.N
                while self.idx_f < 0.0:
                    self.idx_f += self.N
            else:
                # Clamping y parar si llega al final
                if self.idx_f >= self.N - 1:
                    self.idx_f = self.N - 1
                    self.freq = 0.0
                    v = 0.0
                    rospy.loginfo("Fin de trayectoria, paro.")
                if self.idx_f <= 0.0:
                    self.idx_f = 0.0
                    self.freq = 0.0
                    v = 0.0
                    rospy.loginfo("Inicio de trayectoria, paro.")

        # Pose objetivo (usamos el punto más cercano al índice actual)
        i0 = int(round(self.idx_f))
        if i0 < 0:
            i0 = 0
        if i0 >= self.N:
            i0 = self.N - 1
        pose = self.path_poses[i0].pose

        self.send_state_and_debug(pose, v, self.freq)

    def send_state_and_debug(self, pose, v, freq):
        """
        - Envía ModelState a Gazebo (posición + orientación)
        - Publica String de debug con frecuencia, velocidad y yaw
        """
        # --- set_model_state ---
        state = ModelState()
        state.model_name = self.model_name
        state.pose = pose

        yaw = yaw_from_quat(pose.orientation)
        vx = v * math.cos(yaw)
        vy = v * math.sin(yaw)

        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.linear.z = 0.0

        try:
            self.set_state_srv(state)
        except Exception as e:
            rospy.logwarn_throttle(1.0,
                "[centerline_teleop] Error en set_model_state: %s", str(e))

        # --- debug String ---
        yaw_deg = math.degrees(yaw)
        dbg = String()
        dbg.data = ("freq=%.3f pts/s, v=%.3f m/s, yaw=%.3f rad (%.1f deg)"
                    % (freq, v, yaw, yaw_deg))
        self.pub_debug.publish(dbg)


def main():
    # Guardamos la configuración del terminal (como en tu teleop que funciona)
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("centerline_teleop")
    node = CenterlineTeleop()

    loop_hz = node.rate_hz
    timeout = 1.0 / loop_hz
    rate = rospy.Rate(loop_hz)

    try:
        while not rospy.is_shutdown():
            # Leer tecla (no bloqueante)
            key = getKey(settings, timeout)
            if key:
                if key in ('w', 's', ' '):
                    node.update_freq_from_key(key)
                elif key == 'q' or key == '\x03':
                    break

            # Paso de control
            node.step()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Restaurar configuración del terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
