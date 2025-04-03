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

figPos, axPos = plt.subplots(3,1,sharex=True)
figVel, axVel = plt.subplots(3,1,sharex=True)
figAtt, axAtt = plt.subplots(3,1,sharex=True)

for i in range(3):
    #position
    axPos[i].plot(est['t'], est['pos'+'xyz'[i]], 'k', label='mocap')
    axPos[i].plot(tel['t'], tel['position'][:,i], 'b', label='uwb')
    axPos[i].plot(radioCmd['t'][radioCmd['debugtype']==3], radioCmd['debugvals'][radioCmd['debugtype']==3,i],'r--', label='cmd')
    axPos[i].set_ylabel('Pos '+'xyz'[i])

    #velocity
    axVel[i].plot(est['t'], est['vel'+'xyz'[i]], 'k', label='mocap')
    axVel[i].plot(tel['t'], tel['velocity'][:,i], 'b', label='uwb')
    axVel[i].set_ylabel('Vel '+'xyz'[i])

    #velocity
    axAtt[i].plot(est['t'], est['att'+['yaw','pitch','roll'][i]], 'k', label='mocap')
    axAtt[i].plot(tel['t'], tel['attitudeYPR'][:,i], 'b', label='uwb')
    axAtt[i].set_ylabel('Att '+'YPR'[i])


for ax in [axPos, axVel, axAtt]:
    ax[-1].set_xlabel('Time [s]')
    ax[-1].legend()

plt.show()

