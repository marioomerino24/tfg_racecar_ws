#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from racecar_cone_msgs.msg import Centerline


def yaw_to_quat(yaw):
    qz = math.sin(0.5 * yaw)
    qw = math.cos(0.5 * yaw)
    return qz, qw


class CenterlineToPathNode(object):
    def __init__(self):
        self.topic_centerline = rospy.get_param("~topic_centerline", "/estimation/track/centerline")
        self.topic_path = rospy.get_param("~topic_path", "/planning/track/centerline_path")
        self.set_orientation = bool(rospy.get_param("~set_orientation", True))
        self.min_points = int(rospy.get_param("~min_points", 2))

        self.pub_path = rospy.Publisher(self.topic_path, Path, queue_size=1)
        rospy.Subscriber(self.topic_centerline, Centerline, self.cb, queue_size=1)

        rospy.loginfo("centerline_to_path: %s -> %s", self.topic_centerline, self.topic_path)

    def _heading(self, pts, i):
        n = len(pts)
        if n < 2:
            return 0.0
        if i <= 0:
            dx = pts[1].x - pts[0].x
            dy = pts[1].y - pts[0].y
        elif i >= n - 1:
            dx = pts[n - 1].x - pts[n - 2].x
            dy = pts[n - 1].y - pts[n - 2].y
        else:
            dx = pts[i + 1].x - pts[i - 1].x
            dy = pts[i + 1].y - pts[i - 1].y
        return math.atan2(dy, dx)

    def cb(self, msg):
        samples = list(msg.samples)
        if len(samples) < self.min_points:
            rospy.logwarn_throttle(
                2.0,
                "centerline_to_path: centerline con pocos puntos (%d < %d), no publica Path.",
                len(samples), self.min_points
            )
            return

        out = Path()
        out.header = msg.header
        if out.header.stamp.to_sec() <= 0.0:
            out.header.stamp = rospy.Time.now()

        out.poses = []
        for i, p in enumerate(samples):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.position.z = p.z
            ps.pose.orientation.w = 1.0

            if self.set_orientation:
                yaw = self._heading(samples, i)
                qz, qw = yaw_to_quat(yaw)
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw

            out.poses.append(ps)

        self.pub_path.publish(out)


if __name__ == "__main__":
    rospy.init_node("centerline_to_path")
    CenterlineToPathNode()
    rospy.spin()
