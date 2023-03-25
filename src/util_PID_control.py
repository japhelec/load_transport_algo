#!/usr/bin/env python3

import numpy as np

class PID():
    def __init__(self, Kp_x, Kp_y, Kp_z, Ki_x, Ki_y, Ki_z, Kd_x, Kd_y, Kd_z):
        self.Kp_x = Kp_x
        self.Kp_y = Kp_y
        self.Kp_z = Kp_z
        self.Kd_x = Kd_x
        self.Kd_y = Kd_y
        self.Kd_z = Kd_z
        self.Ki_x = Ki_x
        self.Ki_y = Ki_y
        self.Ki_z = Ki_z

        self.preErrx = 0.0
        self.preErry = 0.0
        self.preErrz = 0.0
        self.sumErrx = 0.0
        self.sumErry = 0.0
        self.sumErrz = 0.0
        self.err = 0

    def setTarget(self, target, tolerence):
        self.target = target
        self.tolerence = tolerence

    def check(self, data):
        err = self.target - data

        if err.dot(err) < self.tolerence*self.tolerence:
            return True
        else:
            return False

    def update(self, data):
        err = self.target - data
        self.err = err

        self.sumErrx = self.sumErrx + err[0]
        self.sumErry = self.sumErry + err[1]
        self.sumErrz = self.sumErrz + err[2]
        dErrx = err[0] - self.preErrx
        dErry = err[1] - self.preErry
        dErrz = err[2] - self.preErrz
        ux = self.Kp_x * err[0] + self.Ki_x * self.sumErrx + self.Kd_x * dErrx
        uy = self.Kp_y * err[1] + self.Ki_y * self.sumErry + self.Kd_y * dErry
        uz = self.Kp_z * err[2] + self.Ki_z * self.sumErrz + self.Kd_z * dErrz
        self.preErrx = err[0]
        self.preErry = err[1]
        self.preErrz = err[2]

        return np.array([ux, uy, uz])



class PID_z():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.preErr = 0.0
        self.sumErr = 0.0
        self.err = 0

    def setTarget(self, target, tolerence = 0.0):
        self.target = target
        self.tolerence = tolerence

    def check(self, data):
        err = self.target - data

        if err*err < self.tolerence*self.tolerence:
            return True
        else:
            return False

    def update(self, data):
        err = self.target - data
        self.err = err

        self.sumErr = self.sumErr + err
        dErr = err - self.preErr
        u = self.Kp * err + self.Ki * self.sumErr + self.Kd * dErr
        self.preErr = err

        return u
