#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import numpy as np
import hebi


def volts_to_vwc(v):
    if v < 0.0:
        return np.nan
    elif v < 1.1:
        return 10*v - 1
    elif v < 1.3:
        return 25*v - 17.5
    elif v < 1.82:
        return 48.08*v - 47.5
    elif v < 2.2:
        return 26.32*v - 7.89
    elif v < 3:
        return 62.5*v - 87.5

    return np.nan


if __name__ == '__main__':
    rospy.init_node('environmental_sensors')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'Chevron-IO'
    names = ['Sensors']
    sensor_board = lookup.get_group_from_names(family, names)
    while sensor_board is None:
        sensor_board = lookup.get_group_from_names(family, names)


    sensor_board_fbk = sensor_board.get_next_feedback()

    t = rospy.get_time()

    co2_publisher = rospy.Publisher('co2_sensor', Float64, queue_size=1)
    moisture_publisher = rospy.Publisher('moisture_sensor', Float64, queue_size=1)

    co2_ppm = Float64(0.0)
    moisture_vwc = Float64(0.0)

    def send_vwc(evt):
        moisture_publisher.publish(moisture_vwc)

    def send_co2(evt):
        co2_publisher.publish(co2_ppm)

    rospy.Timer(rospy.Duration(0.1), send_vwc)
    rospy.Timer(rospy.Duration(0.1), send_co2)

    while not rospy.is_shutdown():
        t = rospy.get_time()
        sensor_board.get_next_feedback(reuse_fbk=sensor_board_fbk)
        co2_ppm.data = sensor_board_fbk.io.a.get_float(1)
        moisture_vwc.data = volts_to_vwc(sensor_board_fbk.io.a.get_float(2))


