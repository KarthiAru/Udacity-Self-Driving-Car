# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        # Process noise dimensionality
        self.dim_state = params.dim_state
        # Discrete time interval (fixed)
        self.dt = params.dt
        # Process noise covariance design parameter
        self.q = params.q

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        dt = self.dt
        return np.matrix([[1., 0., 0., dt, 0., 0.],
                         [0., 1., 0., 0., dt, 0.],
                         [0., 0., 1., 0., 0., dt],
                         [0., 0., 0., 1., 0., 0.],
                         [0., 0., 0., 0., 1., 0.],
                         [0., 0., 0., 0., 0., 1.]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        q = self.q
        dt = self.dt
        q1 = ((dt**3)/3)*  q 
        q2 = ((dt**2)/2)*  q 
        q3 = dt * q 
    
        return np.matrix([
            [q1, 0., 0., q2, 0., 0.],
            [0., q1, 0., 0., q2, 0.],
            [0., 0., q1, 0., 0., q2],
            [q2, 0., 0., q3, 0., 0.],
            [0., q2, 0., 0., q3, 0.],
            [0., 0., q2, 0., 0., q3]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        x = self.F() * track.x # state prediction
        P = self.F() * track.P * self.F().T + self.Q() # covariance prediction
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        gamma = self.gamma(track, meas)
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        I = np.identity(n=self.dim_state)
        # K = track.P * H.transpose() * np.linalg.inv(S) # Kalman gain
        K = track.P * H.T * S.I  # Kalman gain
        x = track.x + K * gamma # state update
        P = (I - K*H) * track.P # covariance update
        track.set_x(x)
        track.set_P(P)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        gamma = meas.z - meas.sensor.get_hx(track.x)
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        S = H * track.P * H.transpose() + meas.R
        return S
        
        ############
        # END student code
        ############ 