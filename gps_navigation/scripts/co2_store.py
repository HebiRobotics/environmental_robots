#! /usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerRequest
from gps_navigation.srv import GetFloats

import numpy as np


if __name__ == '__main__':
    rospy.init_node('co2_store')

    history = []

    def co2_cb(msg: Float64):
        history.append(msg.data)

    rospy.Subscriber('/co2_sensor', Float64, co2_cb)

    def clear_cb(req):
        history.clear()
        return [True, '']

    rospy.Service('/clear_co2_history', Trigger, clear_cb)

    def trail_cb(req):
        return [history]

    rospy.Service('/get_co2_history', GetFloats, trail_cb)
    rospy.spin()

