#! /usr/bin/env python3

import os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64, ColorRGBA
from gps_navigation.srv import GetSamples, GetSamplesRequest, SaveSample, SaveNIRSample, GetNIRSample, GetNIRSampleRequest

import numpy as np
import hebi

if __name__ == '__main__':
    rospy.init_node('sample_store')

    samples = []
    nir_spectra = []

    def sample_cb(req):
        print('saving sample')
        samples.append(req.sample)
        return []
    rospy.Service('~save_sample', SaveSample, sample_cb)

    def nir_sample_cb(req):
        print('saving nir sample')
        req.sample.sample_type = 'nir'
        req.sample.sample_idx = len(nir_spectra)
        samples.append(req.sample)
        nir_spectra.append(req.spectrum)
        return []
    rospy.Service('~save_nir_sample', SaveNIRSample, nir_sample_cb)

    def samples_srv(req: GetSamplesRequest):
        return [samples]
    rospy.Service('~get_all_samples', GetSamples, samples_srv)

    def nir_sample_srv(req: GetNIRSampleRequest):
        spectrum = nir_spectra[req.sample_num]
        return [spectrum]
    rospy.Service('~get_nir_spectrum', GetNIRSample, nir_sample_srv)

    rospy.spin()

