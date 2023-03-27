#! /usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64, ColorRGBA
from gps_navigation.srv import GetSamples, GetSamplesRequest

import numpy as np
import hebi

if __name__ == '__main__':
    rospy.init_node('sample_store')

    samples = []
    def sample_cb(msg):
        p = PointStamped()
        p.header = msg.header
        p.point = msg.pose.position
        samples.append(p)

    rospy.Subscriber('/sample_location', PoseStamped, sample_cb)

    def samples_srv(req: GetSamplesRequest):
        return [samples]

    rospy.Service('/get_all_samples', GetSamples, samples_srv)

    rospy.spin()

