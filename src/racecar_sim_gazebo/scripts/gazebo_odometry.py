#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import LinkStates

class OdometryNode:
    def __init__(self):
        self.pub_odom = rospy.Publisher('/vesc/odom', Odometry, queue_size=10)
        self.last_pose  = None
        self.last_twist = None
        self.last_stamp = None

        rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_link_states, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.on_timer)  # 20 Hz

    def cb_link_states(self, msg):
        try:
            i = msg.name.index('racecar::base_link')
        except ValueError:
            return
        self.last_pose  = msg.pose[i]
        self.last_twist = msg.twist[i]
        self.last_stamp = rospy.Time.now()

    def on_timer(self, _):
        if self.last_pose is None or self.last_stamp is None:
            return
        odom = Odometry()
        odom.header.stamp = self.last_stamp
        odom.header.frame_id = 'odom'       # marco fijo
        odom.child_frame_id  = 'base_link'  # cuerpo del robot
        odom.pose.pose  = self.last_pose
        odom.twist.twist = self.last_twist
        self.pub_odom.publish(odom)

if __name__ == '__main__':
    rospy.init_node('gazebo_odometry_node')
    OdometryNode()
    rospy.spin()
