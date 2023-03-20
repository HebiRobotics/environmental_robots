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
    cmd = hebi.GroupCommand(robot.size)

    def color_cb(msg):
        c = hebi.Color(msg.r, msg.g, msg.b, msg.a)
        cmd.led.color = c
        print(f'updating color to {c}')
        robot.send_command(cmd)

    rospy.Subscriber('robot_color', ColorRGBA, color_cb)
    rospy.spin()