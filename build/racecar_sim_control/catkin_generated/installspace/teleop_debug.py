#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist


class TeleopAckermannDebugNode(object):
    """
    Nodo de solo-monitorización:
      - Suscribe a un Twist de debug (v, steering) publicado por el teleop.
      - Imprime en consola la velocidad lineal y el ángulo de giro comandados.
    """
    def __init__(self):
        ns = "~"
        self.topic_in = rospy.get_param(ns + "topic_in",
                                        "/teleop/ackermann_cmd_debug")

        rospy.loginfo("[teleop_ackermann_debug] Suscrito a: %s", self.topic_in)

        self.sub = rospy.Subscriber(self.topic_in,
                                    Twist,
                                    self.cb_debug,
                                    queue_size=20)

    def cb_debug(self, msg):
        """
        Callback del Twist de debug.
        Se asume el convenio:
          - linear.x  = v_cmd   [m/s]
          - angular.z = delta   [rad] (steering_angle)
        """
        v = msg.linear.x
        delta = msg.angular.z
        t = rospy.get_time()

        # Mensaje compacto y fácil de ver en consola / rosout
        rospy.loginfo("[teleop_debug] t=%.3f  v=%.3f [m/s]  steering=%.3f [rad]",
                      t, v, delta)


def main():
    rospy.init_node("teleop_ackermann_debug", anonymous=False)
    TeleopAckermannDebugNode()
    rospy.spin()


if __name__ == "__main__":
    main()
