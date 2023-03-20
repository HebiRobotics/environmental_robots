#! /usr/bin/env python3

import os
import rospy
from rospy.timer import TimerEvent

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import hebi
from hebi.robot_model import PositionObjective

from enum import Enum, auto


def get_arm_output_xyz(arm, angles):
    frames = arm.robot_model.get_forward_kinematics('output', angles)
    return frames[6][:3, 3]


if __name__ == '__main__':
    rospy.init_node('scoop_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'ScoopArm'
    names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_scoop']
    this_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(this_dir, "Chevron-Scoop-Arm.hrdf")
    gains_file = os.path.join(this_dir, "ChevronScoopArm-FullGains_STRATEGY4.xml")

    # Create Arm object
    scoop_arm = hebi.arm.create([family],
                          names=names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    
    scoop_arm.load_gains(gains_file)
    scoop_arm.update()

    scoop_stow = np.array([0, 2.8, 2.7, -2.7], dtype=np.float64)
    scoop_mid_stow = np.array([0, 2.2, 2.10, -2.7], dtype=np.float64)
    scoop_home = np.array([0, 0.7, 1.4, -2.1], dtype=np.float64)

    # we have to get the xyz position of the frame used for IK (not end effector)
    scoop_home_xyz = get_arm_output_xyz(scoop_arm, scoop_home)

    scoop_goal = hebi.arm.Goal(scoop_arm.size)
    scoop_stow_goal = hebi.arm.Goal(scoop_arm.size)
    scoop_stow_goal.add_waypoint(t=4, position=scoop_mid_stow)
    scoop_stow_goal.add_waypoint(t=4, position=scoop_stow)
    scoop_deploy_goal = hebi.arm.Goal(scoop_arm.size)
    scoop_deploy_goal.add_waypoint(t=4, position=scoop_mid_stow)
    scoop_deploy_goal.add_waypoint(t=4, position=scoop_home)

    scoop_arm.update()
    scoop_arm.set_goal(scoop_stow_goal)

    stow_complete = False
    def srv_done_cb(evt: TimerEvent):
        global stow_complete
        stow_complete = True

    rospy.Timer(rospy.Duration.from_sec(9), srv_done_cb, oneshot=True)

    scoop_srv_called = False
    scoop_deploy = False
    def scoop_cb(request: SetBoolRequest):
        global scoop_srv_called, scoop_deploy
        scoop_srv_called = True
        scoop_deploy = request.data
        print(f'Scoop deployed: {request.data}')
        return [True, 'Deployed']

    rospy.Service('deploy_scoop', SetBool, scoop_cb)

    # -0.35 to 0 (low to high)
    scoop_delta = Twist()

    def scoop_delta_cb(msg):
        global scoop_delta
        scoop_delta = msg

    rospy.Subscriber('scoop_pose_delta', Twist, scoop_delta_cb)

    target_joints = np.empty(scoop_arm.size, dtype=np.float64)

    while not rospy.is_shutdown():
        scoop_arm.update()

        diff_x = scoop_delta.linear.x != 0.0
        diff_y = scoop_delta.linear.y != 0.0
        diff_z = scoop_delta.linear.z != 0.0
        diff_theta = scoop_delta.angular.z != 0.0

        if not stow_complete:
            pass
        elif scoop_srv_called:
            stow_complete=False
            if scoop_deploy:
                rospy.Timer(rospy.Duration.from_sec(8), srv_done_cb, oneshot=True)
                scoop_arm.set_goal(scoop_deploy_goal)
            else:
                rospy.Timer(rospy.Duration.from_sec(8), srv_done_cb, oneshot=True)
                scoop_arm.set_goal(scoop_stow_goal)

            scoop_srv_called = False

        elif scoop_deploy and (diff_x or diff_y or diff_z or diff_theta):
            curr_xyz = get_arm_output_xyz(scoop_arm, scoop_arm.last_feedback.position_command)
            new_xyz = curr_xyz.copy()
            new_xyz[0] += scoop_delta.linear.x
            new_xyz[1] += scoop_delta.linear.y
            new_xyz[2] += scoop_delta.linear.z

            curr_radius = math.sqrt(curr_xyz[0]*curr_xyz[0] + curr_xyz[1]*curr_xyz[1])
            new_radius = math.sqrt(new_xyz[0]*new_xyz[0] + new_xyz[1]*new_xyz[1])

            # avoid issues with elbow singularity
            seed_joints = scoop_arm.last_feedback.position
            seed_joints[2] = abs(seed_joints[2])
            if seed_joints[2] > 0.2 or new_radius < curr_radius:
                pos_obj = PositionObjective('output', xyz=new_xyz, idx=6, weight=1.0)

                scoop_arm.robot_model.solve_inverse_kinematics(
                        seed_joints,
                        pos_obj,
                        output=target_joints)

                scoop_arm.pending_command.led.color = 'transparent'
            else:
                scoop_arm.pending_command.led.color = 'magenta'

            # offset scoop angle based on gravity vector
            o = scoop_arm.last_feedback[3].orientation
            # reorder wxyz to xyzw
            o = [*o[1:], o[0]]
            # get x axis vector in global frame
            x_l = R.from_quat(o).as_matrix()[:, 0]
            # get angle from level
            theta = math.acos(x_l @ [0, 0, -1])
            #print(f'Theta offset: {np.rad2deg(theta)}')

            target_joints[3] = scoop_delta.angular.z - theta + np.pi/12
            #print(target_joints)
            scoop_goal.clear()
            scoop_goal.add_waypoint(t=0.5, position=target_joints)
            scoop_arm.set_goal(scoop_goal)


        scoop_arm.send()

