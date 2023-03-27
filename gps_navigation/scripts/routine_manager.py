#! /usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import FilterHeading
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger, TriggerRequest, SetBool
from std_msgs.msg import Float64, ColorRGBA

import numpy as np
import hebi

from enum import Enum, auto

class RoutineStates(Enum):
    INACTIVE = auto()
    DRILLING = auto()
    TURNING = auto()
    DEPLOYING = auto()
    SCOOPING = auto()
    STOWING = auto()

class RoutineInputs:
    trigger = False

class RoutineManager:
    def __init__(self):
        self.state = RoutineStates.INACTIVE
        self.augur_depth = np.nan
        self.depth = Float64(0.0)
        self.current_loc = PoseStamped()
        self.raw_heading = np.nan
        self.heading_offset = 0.0
        self.target_heading = np.nan
        self.vel_cmd = Twist()

        self.trigger = False
        self.keyframes = None
    
    @property
    def curr_heading(self):
        return self.raw_heading + self.heading_offset


    def update(self):
        t = rospy.get_time()
        if self.state == RoutineStates.INACTIVE:
            if self.trigger:
                self.trigger = False
                self.transition_to(RoutineStates.DRILLING)

        elif self.state == RoutineStates.DRILLING:
            if t < self.keyframes['t'][-1]:
                self.depth.data = np.interp(t, self.keyframes['t'], self.keyframes['depth'])
                #depth_pub.publish(self.depth)
            else:
                self.transition_to(RoutineStates.TURNING)

        elif self.state == RoutineStates.TURNING:
            err = self.target_heading - self.curr_heading
            # dumb hacks because 359 and 0 are right next to each other
            if abs(err) > 4:
                err = np.sign(err) * (2 * np.pi - err)
            print(f'Turn P ctrl err: {err}')
            self.vel_cmd.angular.z = err * -0.75
            twist_pub.publish(self.vel_cmd)
            if abs(err) < 0.02:
                self.transition_to(RoutineStates.DEPLOYING)
        elif self.state == RoutineStates.DEPLOYING:
            print('Done!')
            sample_pub.publish(self.current_loc)
            self.transition_to(RoutineStates.INACTIVE)

    def transition_to(self, new_state, *args):
        if new_state == self.state:
            return

        # if we are coming from inactive, set robot to blue
        if self.state == RoutineStates.INACTIVE:
            color_robot_blue()

        # if we are going inactive, set robot to not blue
        if new_state == RoutineStates.INACTIVE:
            clear_robot_color()

        elif new_state == RoutineStates.DRILLING:
            drill_pub.publish(Float64(5.0))
            self.keyframes = self.build_drill_trajectory()
            
        elif new_state == RoutineStates.TURNING:
            drill_pub.publish(Float64(0.0))
            # pick which way we turn to minimize heading windup
            turn_angle = -np.pi
            if self.curr_heading < 0:
                turn_angle *= -1

            self.target_heading = self.curr_heading + turn_angle

        elif self.state == RoutineStates.DEPLOYING:
            deploy_scoop(True)

        self.state = new_state

    def build_drill_trajectory(self):
        start_time = rospy.get_time()
        # 0 to -.35
        depths =           [self.augur_depth, 0.0, -0.22, -0.22, 0.0, -0.22, -0.35, -0.35, 0.0]
        #times =  np.cumsum([start_time,   2.0,   5.0,   2.0, 2.0,   2.0,  10.0,   4.0, 5.0])
        times =  np.cumsum([start_time,   0.2,   .50,   .20, .20,   .20,  .100,   .4, .50])
        print(times)

        return {'t': times, 'depth': depths}

if __name__ == '__main__':
    manager = RoutineManager()
    rospy.init_node('routine_manager')

    sample_pub = rospy.Publisher('/sample_location', PoseStamped, queue_size=5, latch=True)

    color_pub = rospy.Publisher('/robot_color', ColorRGBA, queue_size=5)

    drill_pub = rospy.Publisher('/auger_velocity', Float64, queue_size=5)
    depth_pub = rospy.Publisher('/auger_z_offset', Float64, queue_size=5)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def clear_robot_color():
        c = ColorRGBA()
        c.r = 0.0
        c.g = 0.0
        c.b = 0.0
        c.a = 0.0
        color_pub.publish(c)


    def color_robot_blue():
        c = ColorRGBA()
        c.r = 0.0
        c.g = 0.0
        c.b = 1.0
        c.a = 1.0
        color_pub.publish(c)


    def dig_cb(msg):
        if manager.state == RoutineStates.INACTIVE:
            manager.augur_depth = msg.data
            depth_pub.publish(msg)

    def drill_cb(msg):
        if manager.state == RoutineStates.INACTIVE:
            drill_pub.publish(msg)

    def twist_cb(msg):
        if manager.state == RoutineStates.INACTIVE:
            twist_pub.publish(msg)

    rospy.Subscriber('~auger_velocity', Float64, drill_cb)
    rospy.Subscriber('~auger_z_offset', Float64, dig_cb)
    rospy.Subscriber('~cmd_vel', Twist, twist_cb)


    def odom_cb(msg: Odometry):
        manager.current_loc.header = msg.header
        manager.current_loc.pose = msg.pose.pose

    rospy.Subscriber('/nav/odom', Odometry, odom_cb)


    def heading_cb(msg: FilterHeading):
        prev_heading = manager.raw_heading
        manager.raw_heading = msg.heading_rad
        # probably discontinuity
        if abs(manager.raw_heading - prev_heading) > 1:
            # goal is to keep the signal smooth for pid/etc.
            # 358 359 | 0 1
            # clockwise
            if manager.raw_heading < prev_heading:
                manager.heading_offset += 2*np.pi
            # counterclockwise
            else:
                manager.heading_offset -= 2*np.pi




    rospy.Subscriber('/nav/heading', FilterHeading, heading_cb)

    deploy_scoop = rospy.ServiceProxy('/deploy_scoop', SetBool)


    # take over drill control
    def drill_srv(req: TriggerRequest):
        manager.trigger = True
        return []

    rospy.Service('~drill', Trigger, drill_srv)

    while not rospy.is_shutdown():
        manager.update()
        rospy.sleep(rospy.Duration(0.01))
