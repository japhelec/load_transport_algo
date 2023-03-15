#!/usr/bin/env python3

import os
import numpy as np
import yaml
import pathlib

path = os.path.dirname(__file__)

filepath = str(path) + '/camera_calib/%s.yml' % "tello_601"
with open(filepath, 'r') as f:
    drone_601_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = str(path) + '/camera_calib/%s.yml' % "tello_C"
with open(filepath, 'r') as f:
    drone_C_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = str(path) + '/camera_calib/%s.yml' % "tello_E"
with open(filepath, 'r') as f:
    drone_E_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = str(path) + '/camera_calib/%s.yml' % "tello_A"
with open(filepath, 'r') as f:
    drone_A_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = str(path) + '/camera_calib/%s.yml' % "tello_D"
with open(filepath, 'r') as f:
    drone_D_hd = yaml.load(f, Loader=yaml.FullLoader)

#############################################
#coordinate frame
# c: camera frame
# l: payload frame
# m: captured marker frame
# b: Q frame
# P: ap on payload
#############################################



class Marker:
    length = 0.095 #m

class Drone:
    @staticmethod
    def bRc(drone_id):
        if drone_id == "tello_601":
            return np.array(drone_601_hd['R'])
        elif drone_id == "tello_C":
            return np.array(drone_C_hd['R'])
        elif drone_id == "tello_E":
            return np.array(drone_E_hd['R'])
        elif drone_id == "tello_A":
            return np.array(drone_A_hd['R'])
        elif drone_id == "tello_D":
            return np.array(drone_D_hd['R'])  

    @staticmethod
    def ns2id(tello_ns):
        if tello_ns == "tello_A":
            return 1
        elif tello_ns == "tello_C":
            return 2
        elif tello_ns == "tello_D":
            return 0
        elif tello_ns == "tello_601":
            return 0
            

class Payload:
    @staticmethod
    def Ml(marker_id):
        if marker_id == 0:
            return np.array([0, -0.155, 0]) #m
        elif marker_id == 1:
            return np.array([-0.271, 0.145, 0]) #m
        elif marker_id == 2:
            return np.array([0.271, 0.155, 0]) #m

    @staticmethod
    def mRl(marker_id):
        angle = -np.pi/2*(marker_id%4)
        mRl = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        return mRl

    @staticmethod
    def Pl(ap_id):
        if ap_id == 0:
            Ml = Payload.Ml(0)
            return Ml + np.array([0, -Marker.length/2, 0]) #m
        elif ap_id == 1:
            Ml = Payload.Ml(1)
            return Ml + np.array([-Marker.length/2, Marker.length/2, 0]) #m
        elif ap_id == 2:
            Ml = Payload.Ml(2)
            return Ml + np.array([Marker.length/2, Marker.length/2, 0]) #m
