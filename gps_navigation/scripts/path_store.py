#! /usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerRequest
from gps_navigation.srv import GetPoses

import numpy as np


if __name__ == '__main__':
    rospy.init_node('path_store')

    odom_history = []

    def odom_cb(msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose

        # There's no fix yet, don't record this
        if ps.pose.position.x == 0.0 and ps.pose.position.y == 0:
            return

        if len(odom_history) == 0:
            odom_history.append(ps)
            return

        pos = msg.pose.pose.position
        last_pos = odom_history[-1].pose.position
        dx = pos.x-last_pos.x
        dy = pos.y-last_pos.y

        # only add a point if it's far enough away from the previous saved point
        dist = np.sqrt(dx*dx + dy*dy)

        if dist > 3e-6:
            odom_history.append(ps)

    rospy.Subscriber('/nav/odom_throttle', Odometry, odom_cb)

    samples = []
    nir_spectra = []

    def clear_cb(req):
        odom_history.clear()
        return [True, '']

    rospy.Service('/clear_trail', Trigger, clear_cb)

    def trail_cb(req):
        return [odom_history]

    rospy.Service('/get_trail', GetPoses, trail_cb)
    rospy.spin()

