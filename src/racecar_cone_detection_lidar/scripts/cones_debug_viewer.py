#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def cb_debug(msg):
    # Imprime el string tal cual, respetando \n y \t
    print(msg.data)

def main():
    rospy.init_node("cones_debug_viewer")
    rospy.Subscriber("/cones/debug_serial", String, cb_debug, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
