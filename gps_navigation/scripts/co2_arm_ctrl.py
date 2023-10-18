#! /usr/bin/env python3

import os
import rospy
from rospy.timer import TimerEvent

from geometry_msgs.msg import Twist, Point
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64, Bool
from gps_navigation.srv import SetFloat, SetFloatRequest

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import hebi
from hebi.robot_model import PositionObjective, endeffector_tipaxis_objective, endeffector_position_objective

from enum import Enum, auto


def get_arm_output_frame(arm, angles):
    frames = arm.robot_model.get_forward_kinematics('output', angles)
    return frames[6]

def get_arm_output_xyz(arm, angles):
    frames = arm.robot_model.get_forward_kinematics('output', angles)
    return frames[6][:3, 3]


if __name__ == '__main__':
    rospy.init_node('co2_arm_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'AugerArm'
    names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist']
    this_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(this_dir, "Chevron-CO2-Arm.hrdf")
    gains_file = os.path.join(this_dir, "ChevronScoopArm-FullGains_STRATEGY4.xml")

    # Create Arm object
    probe_arm = hebi.arm.create([family],
                          names=names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    
    probe_arm.load_gains(gains_file)
    probe_arm.update()

    #probe_mid_home = np.array([0, -1.2, -2.0, 0.8], dtype=np.float64)
    probe_home = np.array([1.57, -1.2, -2.0, -0.8], dtype=np.float64)

    # we have to get the xyz position of the frame used for IK (not end effector)
    probe_home_xyz = get_arm_output_xyz(probe_arm, probe_home)

    probe_goal = hebi.arm.Goal(probe_arm.size)
    #probe_stow_goal = hebi.arm.Goal(probe_arm.size)
    #probe_stow_goal.add_waypoint(t=4, position=probe_mid_stow)
    #probe_stow_goal.add_waypoint(t=4, position=probe_stow)
    probe_home_goal = hebi.arm.Goal(probe_arm.size)
    #probe_home_goal.add_waypoint(t=4, position=probe_mid_stow)
    probe_home_goal.add_waypoint(t=6, position=probe_home)

    probe_arm.update()
    probe_arm.set_goal(probe_home_goal)

    #stow_complete = False
    #def srv_done_cb(evt: TimerEvent):
    #    print('Deploy/Stow complete!')
    #    global stow_complete
    #    stow_complete = True

    #rospy.Timer(rospy.Duration.from_sec(9), srv_done_cb, oneshot=True)

    #probe_srv_called = False
    #probe_deploy = False
    #def probe_cb(request: SetBoolRequest):
    #    global probe_srv_called, probe_deploy
    #    if probe_deploy == request.data:
    #        return [True, 'Already In Position']
    #    probe_srv_called = True
    #    probe_deploy = request.data
    #    print(f'Probe deployed: {request.data}')
    #    return [True, 'Deployed']

    #rospy.Service('deploy_sample_arm', SetBool, probe_cb)

    #is_deployed_pub = rospy.Publisher('~is_deployed', Bool, queue_size=3)
    #def status_cb(evt):
    #    global probe_deploy
    #    is_deployed_pub.publish(Bool(probe_deploy))
    #rospy.Timer(rospy.Duration.from_sec(1), status_cb)

    # -0.35 to 0 (low to high)
    probe_delta = Point()

    def probe_delta_cb(msg):
        global probe_delta
        probe_delta = msg

    rospy.Subscriber('co2_probe_pose_delta', Point, probe_delta_cb)

    prev_tool_theta = 0.0
    tool_theta = 0.0

    def tool_angle_cb(request: SetFloatRequest):
        global tool_theta
        out_xyz = probe_arm.FK(probe_arm.last_feedback.position)
        if out_xyz[2] < 0.0:
            return [False, 'Too close to ground']

        tool_theta = request.data
        return [True, 'Tool angle set']

    rospy.Service('~set_tool_angle', SetFloat, tool_angle_cb)

    target_joints = np.empty(probe_arm.size, dtype=np.float64)
    test_joints = np.empty(probe_arm.size, dtype=np.float64)

    while not rospy.is_shutdown():
        probe_arm.update()

        diff_x = probe_delta.x != 0.0
        diff_y = probe_delta.y != 0.0
        diff_z = probe_delta.z != 0.0
        diff_theta = tool_theta != prev_tool_theta 

        if diff_x or diff_y or diff_z or diff_theta:
            angles = probe_arm.last_feedback.position_command
            frames = probe_arm.robot_model.get_forward_kinematics('output', angles)
            curr_xyz = frames[6][:3, 3]
            new_xyz = curr_xyz.copy()
            new_xyz[0] += probe_delta.x
            new_xyz[1] += probe_delta.y
            new_xyz[2] += probe_delta.z

            curr_radius = math.sqrt(curr_xyz[0]*curr_xyz[0] + curr_xyz[1]*curr_xyz[1])
            new_radius = math.sqrt(new_xyz[0]*new_xyz[0] + new_xyz[1]*new_xyz[1])

            seed_joints = probe_arm.last_feedback.position
            # avoid issues with elbow singularity

            at_limits = new_radius < 0.2

            if seed_joints[2] < 0.0 or new_radius < curr_radius:
                pos_obj = PositionObjective('output', xyz=new_xyz, idx=6, weight=1.0)

                # if it's not negative make it so
                seed_joints[2] = -1.0 * abs(seed_joints[2])

                probe_arm.robot_model.solve_inverse_kinematics(
                        seed_joints,
                        pos_obj,
                        output=test_joints)

                #if abs(test_joints[0]) > 0.35:
                #    at_limits = True
            else:
                at_limits = True

            if at_limits:
                probe_arm.pending_command.led.color = 'magenta'
            else: 
                target_joints = test_joints.copy()
                probe_arm.pending_command.led.color = 'transparent'

            x_l = frames[5][:3, 0]
            # get angle from level
            theta = math.acos(x_l @ [0, 0, -1])
            print(f'Theta offset: {np.rad2deg(theta)}')

            target_joints[3] =  theta - np.pi/2 + tool_theta
            #print(target_joints)
            probe_goal.clear()
            probe_goal.add_waypoint(t=0.5, position=target_joints)
            probe_arm.set_goal(probe_goal)
            prev_tool_theta = tool_theta


        probe_arm.send()

