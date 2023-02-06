#!/usr/bin/env python3

import numpy as np
import yaml


filepath = './camera_calib/%s.yml' % "tello_601"
with open(filepath, 'r') as f:
    drone_601_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = './camera_calib/%s.yml' % "tello_C"
with open(filepath, 'r') as f:
    drone_C_hd = yaml.load(f, Loader=yaml.FullLoader)

filepath = './camera_calib/%s.yml' % "tello_E"
with open(filepath, 'r') as f:
    drone_E_hd = yaml.load(f, Loader=yaml.FullLoader)


#############################################
#coordinate frame
# c: camera frame
# l: payload frame
# m: captured marker frame
# b: Q frame
# P: ap on payload
#############################################



class Marker:
    length = 0.05 #m

class Drone:
    @staticmethod
    def bRc(drone_id):
        if drone_id == "tello_601":
            return np.array(drone_601_hd['R'])
        elif drone_id == "tello_C":
            return np.array(drone_C_hd['R'])
        elif drone_id == "tello_E":
            return np.array(drone_E_hd['R'])
            

class Payload:
    @staticmethod
    def m_L(marker_id):
        if marker_id == 0:
            return np.array([0.3312, 0.1713, 0]) #m
        elif marker_id == 1:
            return np.array([-0.2977, 0.1691, 0]) #m
        elif marker_id == 2:
            return np.array([-0.2984, -0.1795, 0]) #m
        elif marker_id == 3:
            return np.array([0.3217, -0.1761, 0]) #m
        elif marker_id == 4:
            return np.array([0, 0.1784, 0]) #m
        elif marker_id == 5:
            return np.array([-0.3063, -0.0017, 0]) #m
        elif marker_id == 6:
            return np.array([0, -0.1784, 0]) #m
        elif marker_id == 7:
            return np.array([0.3196, -0.0056, 0]) #m

    @staticmethod
    def mRL(marker_id):
        angle = -np.pi/2*(marker_id%4)
        mRl = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        return mRl

    @staticmethod
    def P_L(ap_id):
        if ap_id == 0:
            m_L = Payload.m_L(0)
            return m_L + np.array([Marker.length/2, Marker.length/2, 0]) #m
        elif ap_id == 1:
            m_L = Payload.m_L(4)
            return m_L + np.array([0, Marker.length/2, 0]) #m
        elif ap_id == 2:
            m_L = Payload.m_L(1)
            return m_L + np.array([-Marker.length/2, Marker.length/2, 0]) #m
        elif ap_id == 3:
            m_L = Payload.m_L(5)
            return m_L + np.array([-Marker.length/2, 0, 0]) #m
        elif ap_id == 4:
            m_L = Payload.m_L(2)
            return m_L + np.array([-Marker.length/2, -Marker.length/2, 0]) #m
        elif ap_id == 5:
            m_L = Payload.m_L(6)
            return m_L + np.array([0, -Marker.length/2, 0]) #m
        elif ap_id == 6:
            m_L = Payload.m_L(3)
            return m_L + np.array([Marker.length/2, -Marker.length/2, 0]) #m
        elif ap_id == 7:
            m_L = Payload.m_L(7)
            return m_L + np.array([Marker.length/2, 0, 0]) #m
