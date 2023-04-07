#! /usr/bin/env python3

import numpy as np
import hebi

import rospy
from rospy.timer import TimerEvent
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger
from gps_navigation.srv import SetString, SetFloat


GOTO_BTN = 1
PROBE_BTN = 3
#PXRF_BTN = 4
DEPLOY_PROBE_BTN = 4
AUTO_DRILL_BTN = 5
SAMPLE_BTN = 6
AUGER_BTN = 7
CLEAN_BTN = 8

PROBE_Y_AXIS = 1
PROBE_X_AXIS = 2
PROBE_Z_AXIS = 3

LIGHT_AXIS = 4

AUGER_DEPTH_SLIDER = 5
AUGER_SPEED_SLIDER = 6

TURN_AXIS = 7
FWD_AXIS = 8


def setup_mobile_io(m: 'MobileIO'):
    for i in range(8):
        m.set_button_label(i+1, '')
        m.set_axis_label(i+1, '')

    m.set_button_label(GOTO_BTN, 'view')
    m.set_axis_label(LIGHT_AXIS, '\U0001F4A1')
    # Since this value is rescaled -1:1 -> 0:1, set to "0" position to start
    m.set_axis_value(LIGHT_AXIS, -1.0)

    m.set_button_label(PROBE_BTN, 'vwc')
    m.set_button_label(AUGER_BTN, 'drill')
    m.set_button_label(DEPLOY_PROBE_BTN, 'dploy')
    m.set_button_label(AUTO_DRILL_BTN, 'auto')
    m.set_button_label(SAMPLE_BTN, 'sample')
    m.set_button_label(CLEAN_BTN, 'clean')

    m.set_button_mode(PROBE_BTN, 1)
    m.set_button_mode(AUGER_BTN, 1)
    m.set_button_mode(DEPLOY_PROBE_BTN, 0)

    m.set_axis_label(PROBE_X_AXIS, 'pan/tilt')
    m.set_axis_label(PROBE_Z_AXIS, 'Z')
    m.set_snap(PROBE_Z_AXIS, 0)

    m.set_axis_label(FWD_AXIS, 'drive')

    m.set_axis_value(AUGER_DEPTH_SLIDER, 1.0)
    m.set_axis_label(AUGER_DEPTH_SLIDER, '\u21f3')
    m.set_axis_label(AUGER_SPEED_SLIDER, '\U0001F300')


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

    dig_vel = Float64(0.0)
    dig_z_offset = Float64(0.0)

    light_level = Float64(0.0)

    probe_pose_delta = Point()
    pan_tilt_vel = Twist()

    twist = Twist()

    gps_nav_twist = Twist()
    def nav_cmd_cb(msg):
        global gps_nav_twist
        gps_nav_twist = msg
    rospy.Subscriber('/cmd_vel/managed', Twist, nav_cmd_cb)

    cmd_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    drill_pub = rospy.Publisher('/routine_manager/auger_velocity', Float64, queue_size=1)
    depth_pub = rospy.Publisher('/routine_manager/auger_z_offset', Float64, queue_size=1)
    probe_pub = rospy.Publisher('/probe_pose_delta', Point, queue_size=1)
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
    auto_drill = rospy.ServiceProxy('/routine_manager/drill', Trigger)
    cancel_drill = rospy.ServiceProxy('/routine_manager/cancel', Trigger)
    #deploy_sensor = rospy.ServiceProxy('/deploy_sensor', SetBool)

    sample_vwc = rospy.ServiceProxy('/routine_manager/sample_vwc', Trigger)
    sample_nir = rospy.ServiceProxy('/routine_manager/sample_nir', Trigger)
    clean_drill = rospy.ServiceProxy('/auger_ctrl/clean', Trigger)

    def publish_twist(evt: TimerEvent):
        global last_time_active_teleop
        if evt.current_real.to_time() - last_time_active_teleop > 2.0:
            cmd_pub.publish(gps_nav_twist)
        else:
            cmd_pub.publish(twist)


    def publish_drill(evt: TimerEvent):
        global dig_vel, dig_z_offset
        depth_pub.publish(dig_z_offset)
        drill_pub.publish(dig_vel)


    def publish_probe(evt: TimerEvent):
        global probe_pose_delta
        probe_pub.publish(probe_pose_delta)


    def publish_light(evt: TimerEvent):
        global light_level
        light_pub.publish(light_level)


    rospy.Timer(rospy.Duration.from_sec(0.2), publish_twist)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_drill)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_probe)
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

    def on_probe_deploy(probe_deployed):
        deploy_probe(probe_deployed)
        if probe_deployed:
            mio.set_button_label(DEPLOY_PROBE_BTN, 'stow', blocking=False)
            mio.set_axis_label(PROBE_X_AXIS, 'probe ctrl', blocking=False)
        else:
            mio.set_button_label(DEPLOY_PROBE_BTN, 'dploy', blocking=False)
            mio.set_axis_label(PROBE_X_AXIS, 'pan/tilt', blocking=False)
            probe_pose_delta.x = 0.0
            probe_pose_delta.y = 0.0
            probe_pose_delta.z = 0.0
            mio.set_button_mode(PROBE_BTN, 0)
            mio.set_button_mode(PROBE_BTN, 1)
            set_tool_angle(np.pi)

    disconnected = False
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if not mio.update(0.0):
            if not disconnected and now - last_fbk_mio > 1.0:
                disconnected = True
                rospy.logwarn('mobileIO connection lost, stopping robot')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                dig_vel.data = 0.0
            continue

        last_fbk_mio = now
        # if we've just reconnected
        if disconnected:
            setup_mobile_io(mio)
        disconnected = False

        light_level.data = (mio.get_axis_state(LIGHT_AXIS) + 1.0) / 2.0

        if mio.get_button_state(GOTO_BTN):
            curr_view_idx += 1
            curr_view_idx = curr_view_idx % 3
            resp = camera_goto(camera_views[curr_view_idx])
            if not resp.success:
                curr_view_idx -= 1

        dx = mio.get_axis_state(FWD_AXIS)
        drz = mio.get_axis_state(TURN_AXIS)
        twist.linear.x = 0.35 * np.sign(dx) * dx ** 2
        twist.angular.z = -1.30 * np.sign(drz) * drz ** 2

        if twist.linear.x != 0.0 and twist.angular.z != 0:
            last_time_active_teleop = last_fbk_mio

        if mio.get_button_diff(AUTO_DRILL_BTN) == 1:
            if not is_autonomous:
                auto_drill()
            else:
                cancel_drill()

        if mio.get_button_diff(DEPLOY_PROBE_BTN) == 1:
            probe_deployed = not probe_deployed
            on_probe_deploy(probe_deployed)

        if mio.get_button_diff(PROBE_BTN) == 1:
            mio.set_button_label(PROBE_BTN, 'nir', blocking=False)
        elif mio.get_button_diff(PROBE_BTN) == -1:
            mio.set_button_label(PROBE_BTN, 'vwc', blocking=False)

        if mio.get_button_state(CLEAN_BTN):
            clean_drill()

        if probe_deployed:
            dx = 0.02 * mio.get_axis_state(PROBE_X_AXIS)
            dy = -0.02 * mio.get_axis_state(PROBE_Y_AXIS)
            dz = 0.05 * mio.get_axis_state(PROBE_Z_AXIS)

            probe_pose_delta.x = dx
            probe_pose_delta.y = dy
            probe_pose_delta.z = dz

            using_probe = mio.get_button_state(PROBE_BTN)
            if using_probe:
                set_tool_angle(np.pi/2)
            else:
                set_tool_angle(np.pi)

            if mio.get_button_state(SAMPLE_BTN):
                if using_probe:
                    sample_nir()
                else:
                    sample_vwc()
        else:
            pan_tilt_vel.angular.x = mio.get_axis_state(PROBE_X_AXIS)
            pan_tilt_vel.angular.z = -1 * mio.get_axis_state(PROBE_Y_AXIS)
            pan_tilt_pub.publish(pan_tilt_vel)

        dig_z_offset.data = 0.55 * (mio.get_axis_state(AUGER_DEPTH_SLIDER) - 1.0) / 2.0 + 0.2
        if mio.get_button_state(AUGER_BTN) == 1:
            slider_val = mio.get_axis_state(AUGER_SPEED_SLIDER)
            if abs(slider_val) < 0.1:
                slider_val = 0.0
            dig_vel.data = 6.0 * slider_val
        elif mio.get_button_diff(AUGER_BTN) == -1:
            dig_vel.data = 0.0
