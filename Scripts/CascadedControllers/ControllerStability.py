from __future__ import division, print_function

import numpy as np

# Construct the LTI system, for horizontal direction (x - pitch):
#   states are: pos_x, vel_x, pitch_angle, pitch_rate
#   input is: pitch torque 

Ixx = 16e-6 #CF
g  = 9.81
posCtrl_wn   = 1.0
posCtrl_zeta = 0.7
att_timeConst = 0.4
angVel_timeConst = 0.04

#from symbolic script
Aol = np.matrix([[0, 1, 0, 0], [0, 0, g, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
Bol = np.matrix([[0], [0], [0], [1/Ixx]])
K   = np.matrix([[Ixx*posCtrl_wn**2/(angVel_timeConst*att_timeConst*g), 2*Ixx*posCtrl_wn*posCtrl_zeta/(angVel_timeConst*att_timeConst*g), Ixx/(angVel_timeConst*att_timeConst), Ixx/angVel_timeConst]])

evals = np.linalg.eigvals(Aol-Bol.dot(K))
if np.max(np.real(evals))>0:
    print("Unstable!")
else:
    print("Stable.")

print('Eigenvalues are:')
for e in evals:
    print(e)


