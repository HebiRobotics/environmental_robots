#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

import hebi

TOOL_BTN = 1
PXRF_BTN = 3

FWD_AXIS = 8
TURN_AXIS = 7

if __name__ == '__main__':
    rospy.init_node('teleop')

    cmd_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    twist = Twist()

    deploy_tool = rospy.ServiceProxy('/deploy_tool', SetBool)
    deploy_sensor = rospy.ServiceProxy('/deploy_sensor', SetBool)

    lookup = hebi.Lookup()
    rospy.sleep(2)
    mio = hebi.util.create_mobile_io(lookup, 'Chevron')

    mio.set_button_label(PXRF_BTN, 'pxrf')
    mio.set_button_label(TOOL_BTN, 'tool')

    mio.set_button_mode(PXRF_BTN, 1)
    mio.set_button_mode(TOOL_BTN, 1)

    mio.set_axis_label(TURN_AXIS, '')
    mio.set_axis_label(FWD_AXIS, 'drive')

    def publish_twist(evt):
        cmd_pub.publish(twist)

    rospy.Timer(rospy.Duration.from_sec(0.2), publish_twist)

    while not rospy.is_shutdown():
        mio.update()

        twist.linear.x = mio.get_axis_state(FWD_AXIS)
        twist.angular.z = mio.get_axis_state(TURN_AXIS)

        if mio.get_button_diff(PXRF_BTN) == 1:
            deploy_sensor(True)
        elif mio.get_button_diff(PXRF_BTN) == -1:
            deploy_sensor(False)

        if mio.get_button_diff(TOOL_BTN) == 1:
            deploy_tool(True)
        elif mio.get_button_diff(TOOL_BTN) == -1:
            deploy_tool(False)
