#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class Source(object):
    def __init__(self, name, topic, timeout, priority, shortcircuit):
        self.name = name
        self.topic = topic
        self.timeout = float(timeout)
        self.priority = int(priority)
        self.shortcircuit = bool(shortcircuit)
        self.last_msg = None
        self.last_t = 0.0


class AckermannMux(object):
    def __init__(self):
        ns = "~"

        # Parámetros básicos
        self.output_topic = rospy.get_param(ns + "output", "/ackermann_cmd")
        self.rate_hz = float(rospy.get_param(ns + "rate_hz", 50.0))
        self.idle_publish_stop = bool(rospy.get_param(ns + "idle_publish_stop", True))

        topics_cfg = rospy.get_param(ns + "topics", [])

        if not topics_cfg:
            rospy.logwarn("ackermann_cmd_mux: ~topics vacío, no hay entradas configuradas.")

        # Cargar fuentes y suscribirse
        self.sources = []
        for cfg in topics_cfg:
            # cfg es un dict con al menos "topic"
            name         = cfg.get("name", "src")
            topic        = cfg["topic"]
            timeout      = cfg.get("timeout", 0.5)
            priority     = cfg.get("priority", 0)
            shortcircuit = cfg.get("shortcircuit", False)

            src = Source(name, topic, timeout, priority, shortcircuit)
            self.sources.append(src)

            rospy.Subscriber(topic,
                             AckermannDriveStamped,
                             self._mk_cb(src),
                             queue_size=10)

            rospy.loginfo("mux: input '%s' (%s) prio=%d timeout=%.2fs shortcircuit=%s",
                          name, topic, int(priority), float(timeout), bool(shortcircuit))

        # Ordenar por prioridad descendente
        self.sources.sort(key=lambda s: s.priority, reverse=True)

        # Publicador de salida
        self.pub = rospy.Publisher(self.output_topic,
                                   AckermannDriveStamped,
                                   queue_size=20)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_timer)
        rospy.loginfo("mux: output -> %s  (rate=%.1f Hz)", self.output_topic, self.rate_hz)

    # ==== Callbacks y lógica interna ====

    def _mk_cb(self, src):
        def cb(msg):
            src.last_msg = msg
            src.last_t = time.time()
            if src.shortcircuit:
                # Publicación inmediata (e-stop, por ejemplo)
                self._publish(msg)
        return cb

    def _alive(self, src, now):
        return (src.last_msg is not None) and ((now - src.last_t) <= src.timeout)

    def _select(self, now):
        for src in self.sources:
            if self._alive(src, now):
                return src
        return None

    def _publish(self, msg):
        # Actualiza el stamp para que sea "reciente"
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def _on_timer(self, _evt):
        now = time.time()
        src = self._select(now)

        if src is not None:
            # Publica el último comando válido de la fuente seleccionada
            self._publish(src.last_msg)
        elif self.idle_publish_stop:
            # Nadie vivo → publica stop
            stop = AckermannDriveStamped()
            stop.header.stamp = rospy.Time.now()
            stop.header.frame_id = "base_link"
            stop.drive.steering_angle = 0.0
            stop.drive.speed = 0.0
            self.pub.publish(stop)


def main():
    rospy.init_node("ackermann_cmd_mux", anonymous=False)
    AckermannMux()
    rospy.spin()


if __name__ == "__main__":
    main()
