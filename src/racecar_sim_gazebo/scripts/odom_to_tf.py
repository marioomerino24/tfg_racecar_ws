#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, tf, math
from nav_msgs.msg import Odometry

class OdomToTF(object):
    def __init__(self):
        self.parent = rospy.get_param("~odom_frame",  "odom")
        self.child  = rospy.get_param("~base_frame",  "base_link")
        topic       = rospy.get_param("~odom_topic",  "/vesc/odom")
        self.br     = tf.TransformBroadcaster()
        self.warned = False
        rospy.Subscriber(topic, Odometry, self.cb, queue_size=20)

    def cb(self, msg):
        # Chequea que la odometría viene en el frame esperado
        if msg.header.frame_id != self.parent and not self.warned:
            rospy.logwarn("Odom frame_id (%s) != %s; revisa tu publisher.",
                          msg.header.frame_id, self.parent)
            self.warned = True

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if n < 1e-9:
            rospy.logwarn_throttle(2.0, "Cuaternión inválido en Odometry; se omite TF.")
            return

        self.br.sendTransform((p.x, p.y, p.z),
                              (q.x/n, q.y/n, q.z/n, q.w/n),
                              msg.header.stamp,
                              self.child, self.parent)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    OdomToTF()
    rospy.spin()
