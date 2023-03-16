#!/usr/bin/env python3
import numpy as np

class RotM:
    @staticmethod
    def Euler_zxy(R):
        r00 = R[0,0]
        r01 = R[0,1]
        r02 = R[0,2]
        r10 = R[1,0]
        r11 = R[1,1]
        r12 = R[1,2]
        r20 = R[2,0]
        r21 = R[2,1]
        r22 = R[2,2]

        if r21<1:
            if r21 > -1:
                thetaX = np.arcsin(r21)
                thetaZ = np.arctan2(-r01, r11)
                thetaY = np.arctan2(-r20, r22)
            else:
                thetaX = -np.pi/2
                thetaZ = -np.arctan2(r02, r00)
                thetaY = 0
        else:
            thetaX = np.pi/2
            thetaZ = np.arctan2(r02, r00)
            thetaY = 0

        return thetaX, thetaY, thetaZ

    @staticmethod
    def toRotm(X, Y, Z):
        Rx = RotM.Rx(X)
        Ry = RotM.Ry(Y)
        Rz = RotM.Rz(Z)
        return np.matmul(Rz, np.matmul(Rx, Ry))

    @staticmethod
    def Rx(th):
        return np.array([
            [1, 0, 0],
            [0, np.cos(th), -np.sin(th)],
            [0, np.sin(th), np.cos(th)]
        ])

    @staticmethod
    def Ry(th):
        return np.array([
            [np.cos(th), 0, np.sin(th)],
            [0, 1, 0],
            [-np.sin(th), 0, np.cos(th)]
        ])

    @staticmethod
    def Rz(th):
        return np.array([
            [np.cos(th), -np.sin(th), 0],
            [np.sin(th), np.cos(th), 0],
            [0, 0, 1]
        ])

    @staticmethod
    def vee(M):
        return np.array([M[2,1], M[0,2], M[1, 0]])
