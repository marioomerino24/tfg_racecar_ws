#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def cb(msg):
    # Imprime el bloque tal cual, con saltos de línea reales
    print(msg.data)
    print("")  # línea en blanco entre bloques (opcional)

def main():
    rospy.init_node("lr_pairing_debug_viewer")
    topic = rospy.get_param("~topic_dbg", "/cones/pairs_debug")
    rospy.Subscriber(topic, String, cb, queue_size=10)
    #rospy.loginfo("[lr_pairing_debug_viewer] escuchando en %s", topic)
    rospy.spin()

if __name__ == "__main__":
    main()
