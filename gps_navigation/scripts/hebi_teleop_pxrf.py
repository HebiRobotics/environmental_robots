#! /usr/bin/env python3

import numpy as np
import hebi

import rospy
from rospy.timer import TimerEvent
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger
from gps_navigation.srv import SetString, SetFloat
from hebi_cpp_api_examples.msg import FlipperVelocityCommand


GOTO_BTN = 1
LIGHT_BTN = 2
DEPLOY_PXRF_BTN = 3
CTRL_CO2_BTN = 4
BTN5 = 5
SAMPLE_BTN = 6
BTN7 = 7
BTN8 = 8

PROBE_Y_AXIS = 1
PROBE_X_AXIS = 2
PROBE_Z_AXIS = 3

FRONT_FLIPPER_AXIS = 5
REAR_FLIPPER_AXIS = 6

TURN_AXIS = 7
FWD_AXIS = 8


def setup_mobile_io(m: 'MobileIO'):
    for i in range(8):
        m.set_button_label(i+1, '')
        m.set_axis_label(i+1, '')

    m.set_button_label(GOTO_BTN, 'view')
    m.set_button_label(LIGHT_BTN, '\U0001F4A1')
    m.set_button_mode(LIGHT_BTN, 1)

    m.set_button_label(DEPLOY_PXRF_BTN, 'pxrf')
    m.set_button_mode(DEPLOY_PXRF_BTN, 1)
    m.set_button_label(CTRL_CO2_BTN, 'co2')
    m.set_button_mode(CTRL_CO2_BTN, 1)
    m.set_button_label(SAMPLE_BTN, 'sample')


    m.set_axis_label(PROBE_X_AXIS, 'pan/tilt')
    m.set_axis_label(PROBE_Z_AXIS, 'Z', blocking=False)
    m.set_snap(PROBE_Z_AXIS, 0)

    m.set_axis_label(FRONT_FLIPPER_AXIS, 'F')
    m.set_snap(FRONT_FLIPPER_AXIS, 0)
    m.set_axis_label(REAR_FLIPPER_AXIS, 'R')
    m.set_snap(REAR_FLIPPER_AXIS, 0)

    m.set_axis_label(FWD_AXIS, 'drive')


if __name__ == '__main__':

    lookup = hebi.Lookup()
    rospy.sleep(2)
    family = 'Chevron'
    mio = hebi.util.create_mobile_io(lookup, family)
    while mio is None:
        mio = hebi.util.create_mobile_io(lookup, family)
        rospy.logwarn(f"Can't find mobileIO device {family}/mobileIO, trying again...")
        rospy.sleep(1)

    mio.resetUI()
    setup_mobile_io(mio)

    rospy.init_node('teleop')

    last_fbk_mio = rospy.get_time()
    last_time_active_teleop = last_fbk_mio

    light_level = Float64(0.0)

    co2_pose_delta = Point()
    pxrf_pose_delta = Point()
    pan_tilt_vel = Twist()

    twist = Twist()
    flipper_vels = FlipperVelocityCommand()

    gps_nav_twist = Twist()
    def nav_cmd_cb(msg):
        global gps_nav_twist
        gps_nav_twist = msg
    rospy.Subscriber('/cmd_vel/managed', Twist, nav_cmd_cb)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    flipper_pub = rospy.Publisher('/flipper_vel', FlipperVelocityCommand, queue_size=5)

    pxrf_delta_pub = rospy.Publisher('/pxrf_probe_pose_delta', Point, queue_size=1)
    co2_delta_pub = rospy.Publisher('/co2_probe_pose_delta', Point, queue_size=1)
    set_tool_angle = rospy.ServiceProxy('/probe_ctrl/set_tool_angle', SetFloat)

    light_pub = rospy.Publisher('/pan_tilt_ctrl/light', Float64, queue_size=1)
    pan_tilt_pub = rospy.Publisher('/pan_tilt_ctrl/pan_tilt_vel', Twist, queue_size=1)
    camera_goto = rospy.ServiceProxy('/pan_tilt_ctrl/goto_pose', SetString)

    camera_views = ['drive', 'front', 'rear']
    curr_view_idx = 0

    def pose_cb(msg):
        global curr_view_idx
        try:
            curr_view_idx = camera_views.index(msg.data)
        except ValueError:
            curr_view_idx = -1
        mio.set_button_label(GOTO_BTN, msg.data, blocking=False)

    rospy.Subscriber('pan_tilt_ctrl/pose', String, pose_cb)

    deploy_probe = rospy.ServiceProxy('/deploy_sample_arm', SetBool)
    #deploy_sensor = rospy.ServiceProxy('/deploy_sensor', SetBool)

    sample_pxrf = lambda: print('Hookup PXRF') #rospy.ServiceProxy('/routine_manager/sample_nir', Trigger)

    def publish_twist(evt: TimerEvent):
        global last_time_active_teleop
        if evt.current_real.to_time() - last_time_active_teleop > 2.0:
            cmd_pub.publish(gps_nav_twist)
        else:
            cmd_pub.publish(twist)


    def publish_co2_delta(evt: TimerEvent):
        global co2_pose_delta
        co2_delta_pub.publish(co2_pose_delta)

    def publish_pxrf_delta(evt: TimerEvent):
        global pxrf_pose_delta
        pxrf_delta_pub.publish(pxrf_pose_delta)


    def publish_light(evt: TimerEvent):
        global light_level
        light_pub.publish(light_level)


    rospy.Timer(rospy.Duration.from_sec(0.2), publish_twist)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_pxrf_delta)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_co2_delta)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_light)


    is_autonomous = False
    def status_cb(msg):
        global is_autonomous
        if msg.data != is_autonomous:
            if msg.data:
                mio.set_button_label(AUTO_DRILL_BTN, '🛑')
            else:
                mio.set_button_label(AUTO_DRILL_BTN, 'auto')
        is_autonomous = msg.data

    rospy.Subscriber('/routine_manager/status', Bool, status_cb)

    probe_deployed = False
    def is_deployed_cb(msg):
        global probe_deployed
        if msg.data != probe_deployed:
            on_probe_deploy(msg.data)
        probe_deployed = msg.data

    rospy.Subscriber('/probe_ctrl/is_deployed', Bool, is_deployed_cb)

    def on_probe_deploy(deploy):
        global probe_deployed
        try:
            deploy_probe(deploy)
        except rospy.service.ServiceException:
            pass
        probe_deployed = deploy
        if deploy:
            mio.set_button_label(DEPLOY_PXRF_BTN, 'stow', blocking=False)
            # clear co2 selected 
            mio.set_button_mode(CTRL_CO2_BTN, 0)
            mio.set_button_mode(CTRL_CO2_BTN, 1)
        else:
            mio.set_button_label(DEPLOY_PXRF_BTN, 'pxrf', blocking=False)
            mio.set_button_mode(DEPLOY_PXRF_BTN, 0)
            mio.set_button_mode(DEPLOY_PXRF_BTN, 1)

            pxrf_pose_delta.x = 0.0
            pxrf_pose_delta.y = 0.0
            pxrf_pose_delta.z = 0.0
            try:
                set_tool_angle(np.pi)
            except rospy.service.ServiceException:
                pass

    disconnected = False
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if not mio.update(0.0):
            if not disconnected and now - last_fbk_mio > 1.0:
                disconnected = True
                rospy.logwarn('mobileIO connection lost, stopping robot')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            continue

        last_fbk_mio = now
        # if we've just reconnected
        if disconnected:
            setup_mobile_io(mio)
        disconnected = False

        light_level.data = 0.8 if mio.get_button_state(LIGHT_BTN) else 0.0

        if mio.get_button_state(GOTO_BTN):
            curr_view_idx += 1
            curr_view_idx = curr_view_idx % 3
            try:
                resp = camera_goto(camera_views[curr_view_idx])
                if not resp.success:
                    curr_view_idx -= 1
            except rospy.service.ServiceException:
                pass

        dx = mio.get_axis_state(FWD_AXIS)
        drz = mio.get_axis_state(TURN_AXIS)
        twist.linear.x = 0.14 * np.sign(dx) * dx ** 2
        twist.angular.z = 0.7 * np.sign(drz) * drz ** 2

        if twist.linear.x != 0.0 and twist.angular.z != 0:
            last_time_active_teleop = last_fbk_mio

        pxrf_pressed = mio.get_button_state(DEPLOY_PXRF_BTN)
        co2_pressed = mio.get_button_state(CTRL_CO2_BTN)

        changed = mio.get_button_diff(DEPLOY_PXRF_BTN) or mio.get_button_diff(CTRL_CO2_BTN)
        unselected = (not pxrf_pressed) and (not co2_pressed)

        if mio.get_button_diff(DEPLOY_PXRF_BTN) != 0:
            on_probe_deploy(pxrf_pressed)
            if unselected:
                mio.set_axis_label(PROBE_X_AXIS, 'pan/tilt', blocking=False)
                mio.set_axis_label(PROBE_Z_AXIS, '', blocking=False)
            else:
                mio.set_axis_label(PROBE_X_AXIS, 'probe ctrl', blocking=False)
                mio.set_axis_label(PROBE_Z_AXIS, 'Z', blocking=False)
        elif mio.get_button_diff(CTRL_CO2_BTN) != 0:
            if co2_pressed:
                on_probe_deploy(False)
                mio.set_axis_label(PROBE_X_AXIS, 'probe ctrl', blocking=False)
                mio.set_axis_label(PROBE_Z_AXIS, 'Z', blocking=False)
                mio.set_button_mode(DEPLOY_PXRF_BTN, 0)
                mio.set_button_mode(DEPLOY_PXRF_BTN, 1)

            elif unselected:
                mio.set_axis_label(PROBE_X_AXIS, 'pan/tilt', blocking=False)
                mio.set_axis_label(PROBE_Z_AXIS, '', blocking=False)


        if pxrf_pressed:
            dx = 0.02 * mio.get_axis_state(PROBE_X_AXIS)
            dy = -0.02 * mio.get_axis_state(PROBE_Y_AXIS)
            dz = 0.05 * mio.get_axis_state(PROBE_Z_AXIS)

            pxrf_pose_delta.x = dx 
            pxrf_pose_delta.y = dy
            pxrf_pose_delta.z = dz 

            try:
                set_tool_angle(np.pi/2)
            except rospy.service.ServiceException:
                pass

            if mio.get_button_state(SAMPLE_BTN):
                try:
                    sample_pxrf()
                except rospy.service.ServiceException:
                    pass
        elif co2_pressed:
            dy = 0.02 * mio.get_axis_state(PROBE_X_AXIS)
            dx = 0.02 * mio.get_axis_state(PROBE_Y_AXIS)
            dz = 0.05 * mio.get_axis_state(PROBE_Z_AXIS)

            co2_pose_delta.x = dx 
            co2_pose_delta.y = dy
            co2_pose_delta.z = dz 
        else:
            pan_tilt_vel.angular.x = mio.get_axis_state(PROBE_X_AXIS)
            pan_tilt_vel.angular.z = -1 * mio.get_axis_state(PROBE_Y_AXIS)
            pan_tilt_pub.publish(pan_tilt_vel)

            front_flipper_vel = 0.8 * mio.get_axis_state(FRONT_FLIPPER_AXIS)
            rear_flipper_vel = 0.8 * mio.get_axis_state(REAR_FLIPPER_AXIS)
            flipper_vels.front_left  = front_flipper_vel
            flipper_vels.front_right = -front_flipper_vel
            flipper_vels.back_left   = rear_flipper_vel
            flipper_vels.back_right  = -rear_flipper_vel
            flipper_pub.publish(flipper_vels)
            rospy.Publisher('/flipper_vel', FlipperVelocityCommand, queue_size=5)

