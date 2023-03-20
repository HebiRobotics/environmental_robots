#! /usr/bin/env python3

import numpy as np
import hebi

import rospy
from rospy.timer import TimerEvent
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolRequest


SCOOP_BTN = 3
#PXRF_BTN = 4
DEPLOY_SCOOP_BTN = 4
AUGER_BTN = 7

SCOOP_Y_AXIS = 1
SCOOP_X_AXIS = 2
SCOOP_Z_AXIS = 3

AUGER_DEPTH_SLIDER = 5
AUGER_SPEED_SLIDER = 6

TURN_AXIS = 7
FWD_AXIS = 8


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(1, 'view')
    m.set_button_mode(1, 1)
    m.set_button_label(2, 'light')
    m.set_button_mode(2, 1)

    m.set_axis_label(4, '\U0001F4A1')

    # Since this value is rescaled -1:1 -> 0:1, set to "0" position to start
    m.set_axis_value(4, -1.0)


if __name__ == '__main__':
    rospy.init_node('teleop')

    dig_vel = Float64(0.0)
    dig_z_offset = Float64(0.0)

    scoop_pose_delta = Twist()

    twist = Twist()

    park_twist = Twist()
    park_twist.linear.x = 0.0
    park_twist.angular.z = 0.0

    parking_brake = False
    def parking_cb(req: SetBoolRequest):
        global parking_brake
        parking_brake = req.data
        msg = 'Brake On' if parking_brake else 'Brake Off'
        rospy.loginfo(msg)
        return True, msg

    parking_brake_srv = rospy.Service('/parking_brake', SetBool, parking_cb)

    gps_nav_twist = Twist()
    def nav_cmd_cb(msg):
        global gps_nav_twist
        gps_nav_twist = msg
    rospy.Subscriber('/cmd_vel/managed', Twist, nav_cmd_cb)

    cmd_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    drill_pub = rospy.Publisher('/auger_velocity', Float64, queue_size=1)
    depth_pub = rospy.Publisher('/auger_z_offset', Float64, queue_size=1)
    scoop_pub = rospy.Publisher('/scoop_pose_delta', Twist, queue_size=1)

    deploy_scoop = rospy.ServiceProxy('/deploy_scoop', SetBool)
    #deploy_sensor = rospy.ServiceProxy('/deploy_sensor', SetBool)

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

    #mio.set_button_label(PXRF_BTN, 'pxrf')
    mio.set_button_label(SCOOP_BTN, 'dump')
    mio.set_button_label(AUGER_BTN, 'drill')
    mio.set_button_label(DEPLOY_SCOOP_BTN, 'dploy')

    #mio.set_button_mode(PXRF_BTN, 1)
    mio.set_button_mode(SCOOP_BTN, 1)
    mio.set_button_mode(AUGER_BTN, 1)
    mio.set_button_mode(DEPLOY_SCOOP_BTN, 1)

    mio.set_axis_label(SCOOP_X_AXIS, 'scoop ctrl')
    mio.set_axis_label(SCOOP_Y_AXIS, '')
    mio.set_axis_label(SCOOP_Z_AXIS, 'Z')
    mio.set_snap(SCOOP_Z_AXIS, 0)

    mio.set_axis_label(TURN_AXIS, '')
    mio.set_axis_label(FWD_AXIS, 'drive')

    mio.set_axis_value(AUGER_DEPTH_SLIDER, 1.0)
    mio.set_axis_value(AUGER_SPEED_SLIDER, 0.0)
    mio.set_axis_label(AUGER_DEPTH_SLIDER, '\u21f3')
    mio.set_axis_label(AUGER_SPEED_SLIDER, '\U0001F300')

    last_fbk_mio = rospy.get_time()
    last_time_active_teleop = last_fbk_mio


    def publish_twist(evt: TimerEvent):
        global last_time_active_teleop, parking_brake
        if parking_brake:
            cmd_pub.publish(park_twist)
        elif evt.current_real.to_time() - last_time_active_teleop > 2.0:
            cmd_pub.publish(gps_nav_twist)
        else:
            cmd_pub.publish(twist)


    def publish_drill(evt: TimerEvent):
        global dig_vel, dig_z_offset
        depth_pub.publish(dig_z_offset)
        drill_pub.publish(dig_vel)


    def publish_scoop(evt: TimerEvent):
        global scoop_pose_delta
        scoop_pub.publish(scoop_pose_delta)


    rospy.Timer(rospy.Duration.from_sec(0.2), publish_twist)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_drill)
    rospy.Timer(rospy.Duration.from_sec(0.2), publish_scoop)


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
        disconnected = False

        dx = mio.get_axis_state(FWD_AXIS)
        drz = mio.get_axis_state(TURN_AXIS)
        twist.linear.x = 0.25 * np.sign(dx) * dx ** 2
        twist.angular.z = -0.75 * np.sign(drz) * drz ** 2

        if twist.linear.x != 0.0 and twist.angular.z != 0:
            last_time_active_teleop = last_fbk_mio

        if mio.get_button_diff(DEPLOY_SCOOP_BTN) == 1:
            deploy_scoop(True)
        elif mio.get_button_diff(DEPLOY_SCOOP_BTN) == -1:
            deploy_scoop(False)
            scoop_pose_delta.linear.x = 0.0
            scoop_pose_delta.linear.y = 0.0
            scoop_pose_delta.linear.z = 0.0
            mio.set_button_mode(SCOOP_BTN, 0)
            mio.set_button_mode(SCOOP_BTN, 1)
            scoop_pose_delta.angular.z = -np.pi / 2.0

        if mio.get_button_diff(SCOOP_BTN) == 1:
            mio.set_button_label(SCOOP_BTN, 'scoop')
        elif mio.get_button_diff(SCOOP_BTN) == -1:
            mio.set_button_label(SCOOP_BTN, 'dump')

        if mio.get_button_state(DEPLOY_SCOOP_BTN) == 1:
            dx = 0.02 * mio.get_axis_state(SCOOP_X_AXIS)
            dy = -0.02 * mio.get_axis_state(SCOOP_Y_AXIS)
            dz = 0.05 * mio.get_axis_state(SCOOP_Z_AXIS)

            scoop_pose_delta.linear.x = dx 
            scoop_pose_delta.linear.y = dy
            scoop_pose_delta.linear.z = dz 

            if mio.get_button_state(SCOOP_BTN):
                scoop_pose_delta.angular.z = np.pi/2.0 
            else:
                scoop_pose_delta.angular.z = -np.pi/2.0

        dig_z_offset.data = 0.35 * (mio.get_axis_state(AUGER_DEPTH_SLIDER) - 1.0) / 2.0
        if mio.get_button_state(AUGER_BTN) == 1:
            slider_val = mio.get_axis_state(AUGER_SPEED_SLIDER)
            if abs(slider_val) < 0.1:
                slider_val = 0.0
            dig_vel.data = 6.0 * slider_val
        elif mio.get_button_diff(AUGER_BTN) == -1:
            dig_vel.data = 0.0
