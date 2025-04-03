from __future__ import print_function, division

import sys
import matplotlib.pyplot as plt
from convertToPickle import get_data

from py3dmath import Rotation
import numpy as np

dat,fname = get_data(sys.argv)
if len(sys.argv) < 3:
    raise AssertionError('Failed to specify vehicle ID')
vehID = str(sys.argv[2])
print('Plotting')

#shorthands:
mocap = dat['/mocap_output'+vehID]
est = dat['/estimator'+vehID]
tel = dat['/telemetry'+vehID]
radioCmd = dat['/radio_command'+vehID]

figAtt = plt.figure()
figAtt.suptitle(fname)
figAtt.add_subplot(3, 1, 1)
for j in range(3):
    if j==0:
        figAtt.add_subplot(3, 1, j+1)    
    else:
        figAtt.add_subplot(3, 1, j+1, sharex=figAtt.axes[0])    

tatts = mocap['t']
atts_q0 = mocap['attq0']
atts_q1 = mocap['attq1']
atts_q2 = mocap['attq2']
atts_q3 = mocap['attq3']

nPoints = tatts.shape[0]
angVelMocap = np.zeros([nPoints,3])
print('Computing rates from mocap:')
for i in range(2,nPoints):
    a0 = Rotation(atts_q0[i-1], atts_q1[i-1], atts_q2[i-1], atts_q3[i-1]) 
    a1 = Rotation(atts_q0[i], atts_q1[i], atts_q2[i], atts_q3[i]) 
    t1 = tatts[i]
    t0 = tatts[i-1]

    angVelMocap[i,:] = ((a0.inverse()*a1).to_rotation_vector()/(t1-t0)).to_list()
    

dt = tatts[1:]-tatts[:-1]
print(np.median(dt), np.mean(dt), np.min(dt), np.max(dt))

print('Plotting')
#plot ang velocity
for j in range(3):
    axNo = j
    figAtt.axes[axNo].plot(est['t'], est['angvel'+'xyz'[j]], color='b', label='est')
    figAtt.axes[axNo].plot(tel['t'], tel['rateGyro'][:,j], color='g', label='gyro')
    figAtt.axes[axNo].plot(radioCmd['t'], radioCmd['debugvals'][:,j+1], color='r', label='cmd')
    figAtt.axes[axNo].plot(tatts, angVelMocap[:,j], color='c', label='mocapDerivative')
    figAtt.axes[axNo].set_ylabel('AngVel_'+'xyz'[j])

figAtt.axes[0].legend()

plt.show()

