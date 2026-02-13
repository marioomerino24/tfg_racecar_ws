#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def debug_cb(msg):
    # Imprime directamente el contenido de los logs publicados por centerline_fit
    rospy.loginfo("CENTERLINE_DEBUG: %s", msg.data)

if __name__ == "__main__":
    rospy.init_node("centerline_debug_listener")
    rospy.Subscriber("/track/centerline_debug",
                     String,
                     debug_cb,
                     queue_size=10)
    rospy.spin()
