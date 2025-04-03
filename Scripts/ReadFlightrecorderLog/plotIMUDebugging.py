from __future__ import print_function, division
import struct
import numpy as np

vehId = 1

import pickle
inFile = 'flightrecorder_log_veh' + str(vehId).zfill(3) + '.pickle'
print('Reading file: <' + inFile + '>')
pickleFile = open(inFile, 'rb')
[vehIdRead, imu, uwbRequest, uwbResponse] = pickle.load(pickleFile)
pickleFile.close()

import matplotlib.pyplot as plt

fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(imu['t'], imu['accelerometer'])
ax[1].plot(imu['t'], imu['rateGyro'])
ax[2].plot(imu['t'][:-1], np.diff(imu['t']))
# ax[2].plot(imu['t'])

print(imu['t'])

plt.show()
