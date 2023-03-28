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


def get_arm_output_frame(arm, angles):
    frames = arm.robot_model.get_forward_kinematics('output', angles)
    return frames[6]

def get_arm_output_xyz(arm, angles):
    frames = arm.robot_model.get_forward_kinematics('output', angles)
    return frames[6][:3, 3]


if __name__ == '__main__':
    rospy.init_node('probe_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'ScoopArm'
    names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_scoop']
    this_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(this_dir, "Chevron-Scoop-Arm.hrdf")
    gains_file = os.path.join(this_dir, "ChevronScoopArm-FullGains_STRATEGY4.xml")

    # Create Arm object
    probe_arm = hebi.arm.create([family],
                          names=names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    
    probe_arm.load_gains(gains_file)
    probe_arm.update()

    probe_stow = np.array([0, 2.8, 2.7, 0.0], dtype=np.float64)
    probe_mid_stow = np.array([0, 2.2, 2.10, 0.0], dtype=np.float64)
    probe_home = np.array([0, 0.7, 1.4, 0.95], dtype=np.float64)

    # we have to get the xyz position of the frame used for IK (not end effector)
    probe_home_xyz = get_arm_output_xyz(probe_arm, probe_home)

    probe_goal = hebi.arm.Goal(probe_arm.size)
    probe_stow_goal = hebi.arm.Goal(probe_arm.size)
    probe_stow_goal.add_waypoint(t=4, position=probe_mid_stow)
    probe_stow_goal.add_waypoint(t=4, position=probe_stow)
    probe_deploy_goal = hebi.arm.Goal(probe_arm.size)
    probe_deploy_goal.add_waypoint(t=4, position=probe_mid_stow)
    probe_deploy_goal.add_waypoint(t=4, position=probe_home)

    probe_arm.update()
    probe_arm.set_goal(probe_stow_goal)

    stow_complete = False
    def srv_done_cb(evt: TimerEvent):
        global stow_complete
        stow_complete = True

    rospy.Timer(rospy.Duration.from_sec(9), srv_done_cb, oneshot=True)

    probe_srv_called = False
    probe_deploy = False
    def probe_cb(request: SetBoolRequest):
        global probe_srv_called, probe_deploy
        if probe_deploy == request.data:
            return [True, 'Already Deployed']
        probe_srv_called = True
        probe_deploy = request.data
        print(f'Probe deployed: {request.data}')
        return [True, 'Deployed']

    rospy.Service('deploy_sample_arm', SetBool, probe_cb)

    # -0.35 to 0 (low to high)
    probe_delta = Twist()

    def probe_delta_cb(msg):
        global probe_delta
        probe_delta = msg

    rospy.Subscriber('probe_pose_delta', Twist, probe_delta_cb)

    target_joints = np.empty(probe_arm.size, dtype=np.float64)

    while not rospy.is_shutdown():
        probe_arm.update()

        diff_x = probe_delta.linear.x != 0.0
        diff_y = probe_delta.linear.y != 0.0
        diff_z = probe_delta.linear.z != 0.0
        diff_theta = probe_delta.angular.z != 0.0

        if not stow_complete:
            pass
        elif probe_srv_called:
            stow_complete=False
            if probe_deploy:
                rospy.Timer(rospy.Duration.from_sec(8), srv_done_cb, oneshot=True)
                probe_arm.set_goal(probe_deploy_goal)
            else:
                rospy.Timer(rospy.Duration.from_sec(8), srv_done_cb, oneshot=True)
                probe_arm.set_goal(probe_stow_goal)

            probe_srv_called = False

        elif probe_deploy and (diff_x or diff_y or diff_z or diff_theta):
            angles = probe_arm.last_feedback.position_command
            frames = probe_arm.robot_model.get_forward_kinematics('output', angles)
            curr_xyz = frames[6][:3, 3]
            new_xyz = curr_xyz.copy()
            new_xyz[0] += probe_delta.linear.x
            new_xyz[1] += probe_delta.linear.y
            new_xyz[2] += probe_delta.linear.z

            curr_radius = math.sqrt(curr_xyz[0]*curr_xyz[0] + curr_xyz[1]*curr_xyz[1])
            new_radius = math.sqrt(new_xyz[0]*new_xyz[0] + new_xyz[1]*new_xyz[1])

            # avoid issues with elbow singularity
            seed_joints = probe_arm.last_feedback.position
            seed_joints[2] = abs(seed_joints[2])
            if seed_joints[2] > 0.2 or new_radius < curr_radius:
                pos_obj = PositionObjective('output', xyz=new_xyz, idx=6, weight=1.0)

                probe_arm.robot_model.solve_inverse_kinematics(
                        seed_joints,
                        pos_obj,
                        output=target_joints)

                probe_arm.pending_command.led.color = 'transparent'
            else:
                probe_arm.pending_command.led.color = 'magenta'

            # offset scoop angle based on gravity vector
            #o = probe_arm.last_feedback[3].orientation
            # reorder wxyz to xyzw
            #o = [*o[1:], o[0]]
            # get x axis vector in global frame
            #x_l = R.from_quat(o).as_matrix()[:, 0]
            x_l = frames[5][:3, 0]
            # get angle from level
            theta = math.acos(x_l @ [0, 0, -1])
            new_theta = math.acos(new_x_l @ [0, 0, -1])
            #print(f'Theta offset: {np.rad2deg(theta)} vs {np.rad2deg(new_theta)}')

            target_joints[3] = probe_delta.angular.z - theta
            #print(target_joints)
            probe_goal.clear()
            probe_goal.add_waypoint(t=0.5, position=target_joints)
            probe_arm.set_goal(probe_goal)


        probe_arm.send()

