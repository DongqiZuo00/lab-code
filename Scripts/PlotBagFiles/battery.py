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

fig = plt.figure()
fig.suptitle(fname)
n = 3  # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
        fig.add_subplot(n, 1, i, sharex=fig.axes[0])    


tel = dat['/telemetry'+vehID]
#Thrust
sumMotorForces = np.sum(tel['motorForces'],1)


mass = 38e-3  #kg

fig.axes[0].plot(tel['t'], tel['accelerometer'][:,2], color='b', label='meas z')
fig.axes[0].plot(tel['t'], tel['debugVals'][:,0], color='c',label='tel cmd')
fig.axes[0].plot(dat['/radio_command'+vehID]['t'], dat['/radio_command'+vehID]['debugvals'][:,0], color='k',label='radio cmd'+vehID)
fig.axes[0].plot(tel['t'], sumMotorForces/mass, color='r', label='motor forces')
fig.axes[0].set_ylabel('Accel')
fig.axes[0].legend()

fig.axes[1].plot(tel['t'], tel['batteryVoltage'], color='b', label='meas z')
fig.axes[1].set_ylabel('Battery voltage')
fig.axes[1].legend()

#motor forces
for j in range(4):
    fig.axes[2].plot(tel['t'], tel['motorForces'][:,j], color='rgbc'[j], label='mot '+str(j))
fig.axes[2].set_ylabel('Motor forces [N]')

fig.axes[0].legend()
fig.axes[2].legend()
fig.axes[-1].set_xlabel('Time [s]')

plt.show()

