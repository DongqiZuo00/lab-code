from __future__ import print_function, division

import sys
import matplotlib.pyplot as plt
import numpy as np
from convertToPickle import get_data

dat, fname = get_data(sys.argv)
if len(sys.argv) < 3:
    raise AssertionError('Failed to specify vehicle ID')
vehID = str(sys.argv[2])
print('Plotting')

mocap = dat['/mocap_output'+vehID]
est = dat['/estimator'+vehID]
tel = dat['/telemetry'+vehID]
radioCmd = dat['/radio_command'+vehID]

#Thrust
sumMotorForces = np.sum(tel['motorForces'],1)

fig = plt.figure()
fig.suptitle(fname)
n = 5  # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
        fig.add_subplot(n, 1, i, sharex=fig.axes[0])    



#Thrust

mass = 38e-3  #kg

fig.axes[0].plot(tel['t'], tel['accelerometer'][:,2], color='b', label='meas z')
fig.axes[0].plot(tel['t'], tel['debugVals'][:,0], color='c',label='tel cmd')
fig.axes[0].plot(radioCmd['t'], radioCmd['debugvals'][:,0], color='k', label='radio cmd')
fig.axes[0].plot(tel['t'], sumMotorForces/mass, color='r', label='motor forces')
fig.axes[0].set_ylabel('Acceleration')
fig.axes[0].legend()

#gyro
for j in range(3):
    fig.axes[1+j].plot(tel['t'], tel['rateGyro'][:,j], color='b',label='gyro')
    fig.axes[1+j].plot(radioCmd['t'], radioCmd['debugvals'][:,j+1], color='k',label='radio cmd')
    if j<2:
        fig.axes[1+j].plot(tel['t'], tel['debugVals'][:,j+1], color='c',label='tel cmd')

    fig.axes[1+j].set_ylabel('Rates '+'xyz'[j])


#motor forces
for j in range(4):
    fig.axes[4].plot(tel['t'], tel['motorForces'][:,j], color='rgbc'[j], label='mot '+str(j))
fig.axes[4].set_ylabel('Motor forces [N]')

fig.axes[1].legend()
fig.axes[4].legend()
fig.axes[-1].set_xlabel('Time [s]')

plt.show()

