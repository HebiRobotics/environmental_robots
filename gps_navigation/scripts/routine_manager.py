#! /usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64, ColorRGBA

import numpy as np
import hebi

if __name__ == '__main__':
    rospy.init_node('routine_manager')

    sample_pub = rospy.Publisher('/sample_location', PoseStamped, queue_size=5, latch=True)
    sample_loc = PoseStamped()

    color_pub = rospy.Publisher('/robot_color', ColorRGBA, queue_size=5)
    robot_color = ColorRGBA()

    drill_pub = rospy.Publisher('/auger_velocity', Float64, queue_size=5)
    depth_pub = rospy.Publisher('/auger_z_offset', Float64, queue_size=5)

    in_control = False

    plunge_depth = 0.0
    def dig_cb(msg):
        if not in_control:
            global plunge_depth
            plunge_depth = msg.data
            depth_pub.publish(msg)

    def drill_cb(msg):
        if not in_control:
            drill_pub.publish(msg)

    rospy.Subscriber('~auger_velocity', Float64, drill_cb)
    rospy.Subscriber('~auger_z_offset', Float64, dig_cb)

    def odom_cb(msg: Odometry):
        sample_loc.header = msg.header
        sample_loc.pose = msg.pose


    rospy.Subscriber('/nav/odom', Odometry, odom_cb)

    trigger = False
    # take over drill control
    def drill_srv(req: TriggerRequest):
        global trigger
        print('Routine Triggered!')
        trigger = True
        return []

    rospy.Service('~drill', Trigger, drill_srv)

    def build_drill_trajectory():
        start_time = rospy.get_time()
        # 0 to -.35
        depths = [plunge_depth, 0, -0.19, -0.19,  0, -0.35, -0.35,  0]
        times =  [0,            2,    7,    9, 12,    17,    19, 22]
        times = [start_time + t for t in times]
        return hebi.trajectory.create_trajectory(times, depths)

    speed = Float64(0)
    depth = Float64(0)

    traj = None
    while not rospy.is_shutdown():
        if trigger:
            robot_color.r = 0.0
            robot_color.g = 0.0
            robot_color.b = 1.0
            robot_color.a = 1.0
            color_pub.publish(robot_color)
            trigger = False
            in_control = True
            speed.data = 5.0
            traj = build_drill_trajectory()

        if in_control:
            t = rospy.get_time()
            if t < traj.end_time:
                depth.data = traj.get_state(t)[0]
            else:
                robot_color.r = 0.0
                robot_color.g = 0.0
                robot_color.b = 0.0
                robot_color.a = 0.0
                color_pub.publish(robot_color)
                in_control = False
                speed.data = 0.0
                sample_pub.publish(sample_loc)

            depth_pub.publish(depth)
            drill_pub.publish(speed)

        rospy.sleep(rospy.Duration(0.01))
