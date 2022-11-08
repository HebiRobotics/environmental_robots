#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import hebi


if __name__ == '__main__':
    rospy.init_node('teleop')

    cmd_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    twist = Twist()

    lookup = hebi.Lookup()
    rospy.sleep(2)
    mio = hebi.util.create_mobile_io(lookup, 'HEBI')

    def publish_twist(evt):
        twist.linear.x = mio.get_axis_state(2)
        twist.angular.z = mio.get_axis_state(1)
        cmd_pub.publish(twist)

    rospy.Timer(rospy.Duration.from_sec(0.1), publish_twist)

    while not rospy.is_shutdown():
        mio.update()

        twist.linear.x = mio.get_axis_state(2)
        twist.angular.z = mio.get_axis_state(1)

        rospy.spin()
