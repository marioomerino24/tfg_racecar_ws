#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def debug_callback(msg):
    # Imprime tal cual el string recibido
    rospy.loginfo("[centerline_debug] %s", msg.data)

def main():
    rospy.init_node("centerline_debug_listener")

    ns = "~"
    topic = rospy.get_param(ns + "debug_topic",
                            "/teleop/ackermann_cmd_debug")

    rospy.Subscriber(topic, String, debug_callback, queue_size=10)

    rospy.loginfo("Escuchando debug en: %s", topic)
    rospy.spin()

if __name__ == "__main__":
    main()
