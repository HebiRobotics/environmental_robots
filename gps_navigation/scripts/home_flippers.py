#! /usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool

import hebi

if __name__ == '__main__':
    rospy.init_node('home_flippers')

    rospy.sleep(3)
    home_flippers = rospy.ServiceProxy('/home_flippers', SetBool)
    home_flippers(True)
    rospy.spin()
