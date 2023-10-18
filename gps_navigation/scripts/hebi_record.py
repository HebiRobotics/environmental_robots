#! /usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA

import hebi

if __name__ == '__main__':
    rospy.init_node('color_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = '*'
    robot = lookup.get_group_from_family(family)

    def stop_logging():
        robot.stop_log()


    rospy.on_shutdown(stop_logging)
    robot.start_log('/home/hebi/hebilogs')

    rospy.spin()
