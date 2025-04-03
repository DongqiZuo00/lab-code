'''Create the gains for the Mocap state estimator (position). We look at a single axis' position
   and velocity (measurements are assumed isotropic, process noise too, and dynamics are decoupled). 
'''
from __future__ import print_function, division #use newer syntax in Python2

import controlpy  # you'll need this, get it here: https://github.com/markwmuller/controlpy
import numpy as np

measFreq = 200 #[Hz]
accNoiseStd  = 10  #tuning factor, size of "continuous time" acceleration noise, [m/s**2]
measNoiseStd = 0.05 #std dev in [m]

dt = 1/measFreq  # timestep [s]
A = np.matrix([[1,dt],[0,1]])

B = np.matrix([[dt**2/2, dt]])
Q = B.T.dot(B)*accNoiseStd**2  # this is a special case, we have rank(R) == 1, and a known form

#meas is position (x) 
#C = np.matrix([[1,0]])
C = np.matrix([[1,0]])
R = np.identity(1)*measNoiseStd**2

#compute the KF gain: 
L, X, eigs = controlpy.synthesis.estimator_kalman_steady_state_discrete_time(A, C, Q, R)
print('At dt =',dt, 's gain is =',L[0,0], L[1,0])
