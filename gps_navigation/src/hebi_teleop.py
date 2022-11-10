#! /usr/bin/env python3

import numpy as np
import hebi

import rospy
from rospy.timer import TimerEvent
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest


TOOL_BTN = 3
PXRF_BTN = 4

FWD_AXIS = 8
TURN_AXIS = 7


if __name__ == '__main__':
    rospy.init_node('teleop')

    twist = Twist()

    park_twist = Twist()
    park_twist.linear.x = 0.0
    park_twist.angular.z = 0.0

    parking_brake = False
    def parking_cb(req: SetBoolRequest):
        global parking_brake
        parking_brake = req.data
        return True

    parking_brake_srv = rospy.Service('/parking_brake', SetBool, parking_cb)

    gps_nav_twist = Twist()
    def nav_cmd_cb(msg):
        global gps_nav_twist
        gps_nav_twist = msg
    rospy.Subscriber('/cmd_vel/managed', Twist, nav_cmd_cb)

    cmd_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)

    deploy_tool = rospy.ServiceProxy('/deploy_tool', SetBool)
    deploy_sensor = rospy.ServiceProxy('/deploy_sensor', SetBool)

    lookup = hebi.Lookup()
    rospy.sleep(2)
    family = 'Chevron'
    mio = hebi.util.create_mobile_io(lookup, family)
    while mio is None:
        mio = hebi.util.create_mobile_io(lookup, family)
        rospy.logwarn(f"Can't find mobileIO device {family}/mobileIO, trying again...")
        rospy.sleep(1)

    mio.set_button_label(PXRF_BTN, 'pxrf')
    mio.set_button_label(TOOL_BTN, 'tool')

    mio.set_button_mode(PXRF_BTN, 1)
    mio.set_button_mode(TOOL_BTN, 1)

    mio.set_axis_label(TURN_AXIS, '')
    mio.set_axis_label(FWD_AXIS, 'drive')

    last_fbk_mio = rospy.get_time()
    last_time_active_teleop = last_fbk_mio

    def publish_twist(evt: TimerEvent):
        global last_time_active_teleop
        if parking_brake:
            cmd_pub.publish(park_twist)
        if evt.current_real.to_time() - last_time_active_teleop > 2.0:
            cmd_pub.publish(gps_nav_twist)
        else:
            cmd_pub.publish(twist)

    rospy.Timer(rospy.Duration.from_sec(0.2), publish_twist)

    while not rospy.is_shutdown():
        now = rospy.get_time()
        if not mio.update(0.0):
            if now - last_fbk_mio > 1.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            continue

        last_fbk_mio = now

        dx = mio.get_axis_state(FWD_AXIS)
        drz = mio.get_axis_state(TURN_AXIS)
        twist.linear.x = np.sign(dx) * dx ** 2
        twist.angular.z = -1.0 * np.sign(drz) * drz ** 2

        if twist.linear.x != 0.0 and twist.angular.z != 0:
            last_time_active_teleop = last_fbk_mio

        if mio.get_button_diff(PXRF_BTN) == 1:
            deploy_sensor(True)
        elif mio.get_button_diff(PXRF_BTN) == -1:
            deploy_sensor(False)

        if mio.get_button_diff(TOOL_BTN) == 1:
            deploy_tool(True)
        elif mio.get_button_diff(TOOL_BTN) == -1:
            deploy_tool(False)
