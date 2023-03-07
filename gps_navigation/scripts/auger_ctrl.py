#! /usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64

import numpy as np
import hebi

from enum import Enum, auto


if __name__ == '__main__':
    rospy.init_node('auger_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'AugerArm'
    names = ['J1_base', 'J2_shoulder', 'J3_elbow']
    this_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(this_dir, "Chevron-Auger-Arm.hrdf")
    gains_file = os.path.join(this_dir, "ChevronAugerArmGains_STRATEGY4.xml")

    # Create Arm object
    auger_arm = hebi.arm.create([family],
                          names=names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    
    auger_arm.load_gains(gains_file)

    auger_goal = hebi.arm.Goal(auger_arm.size)
    auger_home = np.array([-1.57, -2.4, 2.25], dtype=np.float64)
    auger_home_xyz = auger_arm.FK(auger_home)
    auger_goal.add_waypoint(t=5, position=auger_home)

    auger_arm.update()
    auger_arm.set_goal(auger_goal)

    # Motor Driver
    auger_motor = lookup.get_group_from_names(family, 'MD-208')
    while auger_motor is None:
        auger_motor = lookup.get_group_from_names(family, 'MD-208')

    auger_cmd = hebi.GroupCommand(1)

    auger_fbk = auger_motor.get_next_feedback()

    #rospy.Service('deploy_tool', SetBool, tool_arm_cb)
    #rospy.Service('deploy_sensor', SetBool, sensor_arm_cb)

    auger_velocity = 0.0
    def drill_cb(msg):
        global auger_velocity
        auger_velocity = msg.data

    # -0.35 to 0 (low to high)
    auger_z_offset = 0.0
    prev_z_offset = 0.0
    def dig_cb(msg):
        global auger_z_offset
        auger_z_offset = msg.data

    rospy.Subscriber('auger_velocity', Float64, drill_cb)
    rospy.Subscriber('auger_z_offset', Float64, dig_cb)

    while not rospy.is_shutdown():
        auger_motor.get_next_feedback(reuse_fbk=auger_fbk)
        auger_arm.update()
        if auger_z_offset != prev_z_offset:
            prev_z_offset = auger_z_offset
            new_xyz = auger_home_xyz.copy()
            new_xyz[2] += auger_z_offset
            new_joints = auger_arm.ik_target_xyz_tip_axis(
                             auger_arm.last_feedback.position,
                             new_xyz,
                             [0, 0, -1])
            print(new_joints)
            auger_goal.clear()
            auger_goal.add_waypoint(t=1, position=new_joints)
            auger_arm.set_goal(auger_goal)

        auger_arm.send()

        auger_cmd[0].velocity = auger_velocity
        auger_motor.send_command(auger_cmd)

