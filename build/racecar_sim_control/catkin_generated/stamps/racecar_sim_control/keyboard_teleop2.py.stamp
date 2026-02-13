#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import select
import termios
import tty

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


BANNER = """
Teleop Ackermann avanzado (teclado)
-----------------------------------
Controles:
  Aumentar velocidad lineal :   w
  Disminuir velocidad lineal:   x
  Girar a la izquierda      :   a
  Girar a la derecha        :   d
  Centrar dirección         :   s
  Parada de emergencia      : <barra espaciadora>
  Salir                     : q  o  CTRL-C

La velocidad NUNCA será negativa (no hay marcha atrás).
"""


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


def clamp(value, vmin, vmax):
    return max(vmin, min(value, vmax))


def publish_cmd(pub_drive, pub_debug, frame_id,
                speed, steering_angle,
                acceleration, jerk, steering_angle_velocity):
    """
    Publica:
      - AckermannDriveStamped en el topic de mando.
      - Twist en el topic de debug (velocidad lineal / 'angular').
        * dbg.linear.x  = speed (m/s)
        * dbg.angular.z = steering_angle (rad)
    """
    now = rospy.Time.now()

    msg = AckermannDriveStamped()
    msg.header.stamp = now
    msg.header.frame_id = frame_id
    msg.drive.speed = speed
    msg.drive.acceleration = acceleration
    msg.drive.jerk = jerk
    msg.drive.steering_angle = steering_angle
    msg.drive.steering_angle_velocity = steering_angle_velocity
    pub_drive.publish(msg)

    dbg = Twist()
    dbg.linear.x = speed
    dbg.angular.z = steering_angle
    pub_debug.publish(dbg)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("ackermann_teleop_keyboard")

    ns = "~"

    # === Parámetros ROS (reconfigurables por .launch / .yaml) ===
    cmd_topic   = rospy.get_param(ns + "cmd_topic",
                                  "/vesc/ackermann_cmd_mux/input/teleop")
    debug_topic = rospy.get_param(ns + "debug_topic",
                                  "/teleop/ackermann_cmd_debug")
    frame_id    = rospy.get_param(ns + "frame_id", "base_link")

    max_speed   = float(rospy.get_param(ns + "max_speed", 3.0))   # [m/s]
    min_speed   = float(rospy.get_param(ns + "min_speed", 0.0))   # sin marcha atrás
    speed_step  = float(rospy.get_param(ns + "speed_step", 0.1))  # incremento por pulsación

    max_steer   = float(rospy.get_param(ns + "max_steering_angle", 2.0))   # [rad]
    steer_step  = float(rospy.get_param(ns + "steering_step", 0.1))       # [rad] por pulsación

    acceleration = float(rospy.get_param(ns + "acceleration", 0.0))
    jerk         = float(rospy.get_param(ns + "jerk", 0.0))
    steer_vel    = float(rospy.get_param(ns + "steering_angle_velocity", 0.0))

    loop_hz     = float(rospy.get_param(ns + "loop_hz", 20.0))
    timeout     = 1.0 / loop_hz

    pub_drive = rospy.Publisher(cmd_topic, AckermannDriveStamped, queue_size=1)
    pub_debug = rospy.Publisher(debug_topic, Twist, queue_size=1)

    rospy.loginfo(BANNER)
    rospy.loginfo("Publicando comandos en: %s", cmd_topic)
    rospy.loginfo("Publicando debug en   : %s (Twist: linear.x=v, angular.z=steering_angle)",
                  debug_topic)

    # Estado interno del teleop
    speed = 0.0         # v comandada [m/s]
    steering = 0.0      # ángulo de dirección [rad]

    rate = rospy.Rate(loop_hz)

    try:
        while not rospy.is_shutdown():
            key = getKey(settings, timeout)

            if key:
                # === Lógica de teleoperación ===
                if key == 'w':
                    # Incremento de velocidad hacia delante, saturado y sin marcha atrás
                    speed = clamp(speed + speed_step, min_speed, max_speed)
                elif key == 'x':
                    # Decremento de velocidad, pero nunca < min_speed (0 por defecto)
                    speed = clamp(speed - speed_step, min_speed, max_speed)
                elif key == 'a':
                    # Giro incremental a la izquierda
                    steering = clamp(steering + steer_step, -max_steer, max_steer)
                elif key == 'd':
                    # Giro incremental a la derecha
                    steering = clamp(steering - steer_step, -max_steer, max_steer)
                elif key == 's':
                    # Centrar la dirección manteniendo la velocidad actual
                    steering = 0.0
                elif key == ' ':
                    # Parada de emergencia con una sola tecla
                    speed = 0.0
                    steering = 0.0
                    rospy.logwarn("PARADA DE EMERGENCIA: v=0, steering=0")
                elif key == 'q' or key == '\x03':  # 'q' o CTRL-C
                    break

                rospy.loginfo("cmd -> v = %.3f [m/s], steering = %.3f [rad]",
                              speed, steering)

            # Publica continuamente el último comando (útil para mux con timeout)
            publish_cmd(pub_drive, pub_debug, frame_id,
                        speed, steering,
                        acceleration, jerk, steer_vel)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Error en teleop:", e)

    finally:
        # Enviar un último comando de parada segura
        publish_cmd(pub_drive, pub_debug, frame_id,
                    0.0, 0.0,
                    acceleration, jerk, steer_vel)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
