#! /usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry

import hebi

if __name__ == '__main__':
    rospy.init_node('nav_printer')

    def odom_cb(msg):
        dv = msg.twist.twist.linear
        dtheta = msg.twist.twist.angular
        vel = np.sqrt(dv.x*dv.x+dv.y*dv.y*+dv.z*dv.z)
        theta = np.sqrt(dtheta.x*dtheta.x+dtheta.y*dtheta.y+dtheta.z*dtheta.z)
        print(f'vel: {vel:.2f} | turn: {theta*1000:.2f} mrad')

    rospy.Subscriber('/nav/odom', Odometry, odom_cb)
    rospy.spin()
