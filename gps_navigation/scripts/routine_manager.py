#! /usr/bin/env python3

import os
from enum import Enum, auto
import numpy as np
import hebi

import rospy
from rospy.timer import TimerEvent
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import FilterHeading
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger, TriggerRequest, SetBool
from std_msgs.msg import Float64, ColorRGBA, Bool
from gps_navigation.msg import Sample
from gps_navigation.srv import SetString, GetSamples, SaveSample, SaveNIRSample, GetNIRSample
from neospectra.msg import NirReading

AUGER_HOME_Z = 0.2

def set_imu_zupts():
    pass


def build_sample(location, sample_type, value=np.nan):
    sample = Sample()
    sample.header.stamp = rospy.Time.now()
    sample.location = location
    sample.sample_type = sample_type
    sample.scalar_value = value 
    return sample


class RoutineStates(Enum):
    STARTUP = auto()
    INITIALIZE = auto()
    INACTIVE = auto()
    DRILLING = auto()
    RESET_DRILL = auto()
    STOP_DRILL = auto()
    CLEAN_DRILL = auto()
    POSITION_SENSOR = auto()
    DEPLOYING = auto()
    COLLECTING = auto()
    STOWING = auto()

class SampleTypes(Enum):
    DIRT = auto()
    VWC = auto()
    NIR = auto()


class RoutineManager:
    def __init__(self):
        self.state = RoutineStates.STARTUP
        self.augur_depth = np.nan
        self.augur_pos_err = 0.0
        self.auger_base_torque = 0.0
        self.depth = Float64(0.0)
        self.current_loc = PoseStamped()
        self.raw_heading = np.nan
        self.heading_offset = 0.0
        self.target_heading = np.nan
        self.heading_integral = 0.0
        self.vel_cmd = Twist()

        self.start_drill = False
        self.cancel = False

        self.start_nir = False
        self.nir_spectrum = None

        self.start_vwc = False
        self.vwc_readings = []

        self.keyframes = None
    
    @property
    def curr_heading(self):
        return self.raw_heading + self.heading_offset


    def update(self):
        t = rospy.get_time()
        if self.state == RoutineStates.STARTUP:
            try:
                set_imu_zupts()
                home_flippers(True)
                start_nir_background()
                self.transition_to(RoutineStates.INITIALIZE)
            except rospy.service.ServiceException:
                print('waiting for NIR background service to become available...')
                pass
        elif self.state == RoutineStates.INITIALIZE:
            if self.state_end_time < t:
                self.transition_to(RoutineStates.INACTIVE)
        if self.state == RoutineStates.INACTIVE:
            if self.start_drill:
                self.start_drill = False
                self.transition_to(RoutineStates.DRILLING)
            
            if self.start_vwc:
                self.start_vwc = False
                self.transition_to(RoutineStates.COLLECTING, SampleTypes.VWC)
            
            if self.start_nir:
                self.start_nir = False
                self.transition_to(RoutineStates.COLLECTING, SampleTypes.NIR)

        elif self.state == RoutineStates.DRILLING:
            if self.cancel:
                self.cancel = False
                self.transition_to(RoutineStates.STOP_DRILL)
            elif self.auger_pos_err < -0.4 or self.auger_base_torque < -4.0:
                self.transition_to(RoutineStates.RESET_DRILL)
            elif t < self.keyframes['t'][-1]:
                self.depth.data = np.interp(t, self.keyframes['t'], self.keyframes['depth'])
                depth_pub.publish(self.depth)
                drill_speed = np.interp(t, self.drill_keyframes['t'], self.drill_keyframes['speed'])
                drill_pub.publish(Float64(drill_speed))
            else:
                self.transition_to(RoutineStates.CLEAN_DRILL)

        elif self.state == RoutineStates.STOP_DRILL:
            if t < self.keyframes['t'][-1]:
                self.depth.data = np.interp(t, self.keyframes['t'], self.keyframes['depth'])
                depth_pub.publish(self.depth)
                drill_speed = np.interp(t, self.drill_keyframes['t'], self.drill_keyframes['speed'])
                drill_pub.publish(Float64(drill_speed))
            else:
                self.transition_to(RoutineStates.INACTIVE)

        elif self.state == RoutineStates.RESET_DRILL:
            if t < self.keyframes['t'][-1]:
                self.depth.data = np.interp(t, self.keyframes['t'], self.keyframes['depth'])
                depth_pub.publish(self.depth)
                drill_speed = np.interp(t, self.drill_keyframes['t'], self.drill_keyframes['speed'])
                drill_pub.publish(Float64(drill_speed))
            else:
                self.transition_to(RoutineStates.DRILLING)

        elif self.state == RoutineStates.CLEAN_DRILL:
            if self.state_end_time < t:
                self.transition_to(RoutineStates.POSITION_SENSOR)

        elif self.state == RoutineStates.POSITION_SENSOR:
            #err = self.curr_heading - self.target_heading
            #self.heading_integral += err
            #print(f'kp* err: {np.round(0.75 * err, 2)}, ki * integral: {np.round(0.001 * self.heading_integral, 2)}')
            #self.vel_cmd.angular.z = 0.75 * err + 0.001 * self.heading_integral

            self.vel_cmd.linear.x = 0.5
            twist_pub.publish(self.vel_cmd)
            if self.state_end_time < t:
                self.vel_cmd.linear.x = 0.0
                twist_pub.publish(self.vel_cmd)
                self.transition_to(RoutineStates.DEPLOYING)

        elif self.state == RoutineStates.DEPLOYING:
            self.transition_to(RoutineStates.INACTIVE)

        elif self.state == RoutineStates.COLLECTING:
            if self.sample_type == SampleTypes.DIRT:
                sample = build_sample(self.current_loc.pose.position, 'dirt')
                save_sample(sample)
                self.transition_to(RoutineStates.INACTIVE)

            elif self.sample_type == SampleTypes.VWC:
                if self.state_end_time < t:
                    avg = np.mean(self.vwc_readings)
                    sample = build_sample(self.current_loc.pose.position, 'vwc', avg)
                    save_sample(sample)
                    self.transition_to(RoutineStates.INACTIVE)

            elif self.sample_type == SampleTypes.NIR:
                if self.state_end_time < t and self.nir_spectrum is not None:
                    sample = build_sample(self.current_loc.pose.position, 'nir')
                    save_nir_sample(sample, self.nir_spectrum)
                    self.transition_to(RoutineStates.INACTIVE)


    def transition_to(self, new_state, *args):
        if new_state == self.state:
            return

        # if we are coming from inactive, set robot to blue
        if self.state == RoutineStates.INACTIVE:
            color_robot_blue()

        if new_state == RoutineStates.INITIALIZE:
            color_robot_blue()
            self.state_end_time = rospy.get_time() + 15
        # if we are going inactive, set robot to not blue
        elif new_state == RoutineStates.INACTIVE:
            clear_robot_color()

        elif new_state == RoutineStates.DRILLING:
            self.keyframes = self.build_drill_trajectory()
            self.drill_keyframes = self.build_speed_trajectory(0.0, 2.5)

        elif new_state == RoutineStates.STOP_DRILL:
            self.keyframes = self.build_reverse_trajectory()
            self.drill_keyframes = self.build_speed_trajectory(2.5, -2.5)

        elif new_state == RoutineStates.RESET_DRILL:
            self.keyframes = self.build_reverse_trajectory()
            self.drill_keyframes = self.build_speed_trajectory(2.5, -2.5)

        elif new_state == RoutineStates.CLEAN_DRILL:
            clean_drill()
            self.state_end_time = rospy.get_time() + 9
            
        elif new_state == RoutineStates.POSITION_SENSOR:
            drill_pub.publish(Float64(0.0))
            camera_goto('rear')
            self.state_end_time = rospy.get_time() + 3.2
            #self.heading_integral = 0.0
            # pick which way we turn to minimize heading windup
            #turn_angle = -np.pi
            #if self.curr_heading < 0:
            #    turn_angle *= -1

            #self.target_heading = self.curr_heading + turn_angle

        elif new_state == RoutineStates.DEPLOYING:
            self.vel_cmd.angular.z = 0.0
            twist_pub.publish(self.vel_cmd)
            deploy_scoop(True)

        elif new_state == RoutineStates.COLLECTING:
            print('transition to collecting')
            self.sample_type = args[0]

            self.state_end_time = rospy.get_time()
            if self.sample_type == SampleTypes.DIRT:
                pass
            elif self.sample_type == SampleTypes.VWC:
                self.state_end_time += 2
                self.vwc_readings = []
            elif self.sample_type == SampleTypes.NIR:
                self.state_end_time += 12
                self.nir_spectrum = None
                start_nir_measurement()

        self.state = new_state

    def build_drill_trajectory(self):
        start_time = rospy.get_time()
        # 0 to -.35
        # ground is at -0.05
        #                                          -0.18, -0.18, 0.05, -0.18, -0.3, -0.3, 0.05
        #                                          -0.09, -0.09, 0.05, -0.09, -0.15, -0.09, 0.05
        #                                          -0.14, -0.14, 0.0, -0.14, -0.2, -0.2, 0.0
        #depths =          [self.augur_depth, 0.0, -0.22, -0.22, 0.0, -0.22, -0.35, -0.35, 0.0]
        depths = [self.augur_depth, AUGER_HOME_Z, -0.18, -0.18, AUGER_HOME_Z, -0.18, -0.25, -0.25, AUGER_HOME_Z]
        times = np.cumsum([start_time,       2.0,   3.0,   3.0,          2.0,   2.0,   6.0,   5.0, 5.0])
        #times =  np.cumsum([start_time,   0.2,   .50,   .20, .20,   .20,  .100,   .4, .50])
        return {'t': times, 'depth': depths}


    def build_reverse_trajectory(self):
        start_time = rospy.get_time()
        depths = [self.augur_depth, AUGER_HOME_Z]
        times = [start_time,start_time+7.0]
        return {'t': times, 'depth': depths}


    def build_speed_trajectory(self, v_from, v_to):
        start_time = rospy.get_time()
        speeds = [v_from, v_to]
        times = [start_time,start_time+2.0]
        return {'t': times, 'speed': speeds}

if __name__ == '__main__':
    manager = RoutineManager()
    rospy.init_node('routine_manager')

    color_pub = rospy.Publisher('/robot_color', ColorRGBA, queue_size=5)

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

    drill_pub = rospy.Publisher('/auger_velocity', Float64, queue_size=5)
    depth_pub = rospy.Publisher('/auger_z_offset', Float64, queue_size=5)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    save_sample = rospy.ServiceProxy('/sample_store/save_sample', SaveSample)
    save_nir_sample = rospy.ServiceProxy('/sample_store/save_nir_sample', SaveNIRSample)
    clean_drill = rospy.ServiceProxy('/auger_ctrl/clean', Trigger)


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

    def err_cb(msg):
        manager.auger_pos_err = msg.data

    def torque_cb(msg):
        manager.auger_base_torque = msg.data

    rospy.Subscriber('~auger_velocity', Float64, drill_cb)
    rospy.Subscriber('~auger_z_offset', Float64, dig_cb)
    rospy.Subscriber('/auger_ctrl/position_error', Float64, err_cb)
    rospy.Subscriber('/auger_ctrl/base_torque', Float64, torque_cb)
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


    def vwc_cb(msg):
        manager.vwc_readings.append(msg.data)

    rospy.Subscriber('/moisture_sensor', Float64, vwc_cb)


    def nir_cb(msg):
        manager.nir_spectrum = msg

    rospy.Subscriber('/neospectra/measurement', NirReading, nir_cb)
    start_nir_measurement = rospy.ServiceProxy('/neospectra/take_reading', Trigger)
    start_nir_background = rospy.ServiceProxy('/neospectra/take_background', Trigger)

    deploy_scoop = rospy.ServiceProxy('/deploy_sample_arm', SetBool)
    camera_goto = rospy.ServiceProxy('/pan_tilt_ctrl/goto_pose', SetString)

    home_flippers = rospy.ServiceProxy('/home_flippers', SetBool)

    # take over drill control
    def drill_srv(req: TriggerRequest):
        if manager.state == RoutineStates.INACTIVE:
            manager.start_drill = True
        return []


    def cancel_srv(req: TriggerRequest):
        if manager.state == RoutineStates.DRILLING:
            manager.cancel = True
        return []

    rospy.Service('~drill', Trigger, drill_srv)
    rospy.Service('~cancel', Trigger, cancel_srv)


    def sample_nir_cb(req: TriggerRequest):
        if manager.state == RoutineStates.INACTIVE:
            manager.start_nir = True
        return []
    rospy.Service('~sample_nir', Trigger, sample_nir_cb)

    def sample_vwc_cb(req: TriggerRequest):
        if manager.state == RoutineStates.INACTIVE:
            manager.start_vwc = True
        return []
    rospy.Service('~sample_vwc', Trigger, sample_vwc_cb)


    status_pub = rospy.Publisher('~status', Bool, queue_size=5)

    def publish_status(evt: TimerEvent):
        is_active = manager.state != RoutineStates.INACTIVE
        status_pub.publish(Bool(is_active))

    rospy.Timer(rospy.Duration.from_sec(0.2), publish_status)

    while not rospy.is_shutdown():
        manager.update()
        rospy.sleep(rospy.Duration(0.01))
