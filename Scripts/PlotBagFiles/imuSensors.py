from __future__ import print_function, division

import sys
import matplotlib.pyplot as plt
from convertToPickle import get_data

dat,fname = get_data(sys.argv)
if len(sys.argv) < 3:
        raise AssertionError('Failed to specify vehicle ID')
vehID = str(sys.argv[2])
print('Plotting')

fig = plt.figure() 
fig.suptitle(fname)
n = 2  # num sub-plots
fig.add_subplot(2, 3, 1)
for i in range(2):
    for j in range(3):
        if i==j and i==0:
            fig.add_subplot(2, 3, 1+i*3+j)    
        else:
            fig.add_subplot(2, 3, 1+i*3+j, sharex=fig.axes[0])    


tel = dat['/telemetry'+vehID]
for j in range(3):
    axNo = 0*3+j
    fig.axes[axNo].plot(tel['t'], tel['accelerometer'][:,j], 'g.-', label='imu')
    fig.axes[axNo].set_ylabel('Accelerometer'+'xyz'[j])
for j in range(3):
    axNo = 1*3+j
    fig.axes[axNo].plot(tel['t'], tel['rateGyro'][:,j],'g.-', label='imu')
    fig.axes[axNo].set_ylabel('Gyro '+'xyz'[j])


fig.axes[0].legend()

plt.show()
