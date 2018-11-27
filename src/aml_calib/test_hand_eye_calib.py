#!/usr/bin/env python
from __future__ import absolute_import

from aml_calib import HandEyeCalib
from aml_calib.utils import posediff
import numpy as np
import copy
import scipy.io as spio
from os.path import exists, join, dirname, abspath
import os
from pkg_resources import Requirement, resource_filename


if __name__ == "__main__":
    he_calib = HandEyeCalib()

    resources_path = filename = resource_filename(Requirement.parse("aml_tools"),"aml_calib/resources")
    test_data_path = abspath(resources_path)

    wHc = spio.loadmat(test_data_path + '/wHc.mat')
    bHg = spio.loadmat(test_data_path + '/bHg.mat')

    transform_gd = np.array([[0.939849549927233, -0.106333628946128, -0.324616670638965, -2.08974078895036e-13],
                            [-0.0469506932684968,0.901082265777532,-0.431099040480069,-9.77368597530024e-14],
                            [0.338346650497956, 0.420409216902265, 0.841889324341649, 50.0000000000000],
                            [0,0,0,1]])



    for i in range(wHc['wHc'].shape[2]):

        camera_pose = np.linalg.inv(wHc['wHc'][:,:,i])
        gripper_pose = bHg['bHg'][:,:,i]
        
        he_calib.add_measurement(gripper_pose, camera_pose)

    transform = he_calib.calibrate()

    print "Found transform: \n", transform
    print "Ground truth: \n", transform_gd

    dt, daxis_angle, drot_angle = posediff(transform_gd,transform)

    print "Error translation: \n", np.linalg.norm(dt)
    print "Error axis angle: \n", daxis_angle*180/np.pi
    print "Error rotation angle: \n", drot_angle*180/np.pi
    print "Error: ", (transform_gd - transform)




