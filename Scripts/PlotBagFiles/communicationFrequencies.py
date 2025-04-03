from __future__ import print_function, division

import sys
import matplotlib.pyplot as plt
import numpy as np
from convertToPickle import get_data

# Size of the moving average window [s]
window_size = 0.1

def calcRates(data):
    # Record where data starts wrt mocap time
    offset = data['t'][0] - mocap['t'][0]
    
    # Set all initial times to zero
    data['t'] = data['t'] - data['t'][0]
    
    # Calculate rate using moving average
    ratesList = list()
    startIndex = (np.abs(data['t'] - window_size)).argmin()
    count = startIndex
    for i in range(startIndex, len(data['t'])-1):
        dt = data['t'][i] - data['t'][i - startIndex]
        ratesList.append(count/dt)
    
    return ratesList, startIndex, offset

dat, fname = get_data(sys.argv)
if len(sys.argv) < 3:
    raise AssertionError('Failed to specify vehicle ID')
vehID = str(sys.argv[2])
print('Plotting')

mocap = dat['/mocap_output'+vehID]
tel = dat['/telemetry'+vehID]
joy = dat['/joystick_values']
radioCmd = dat['/radio_command'+vehID]

(mocapRates, mocapStartIndex, _) = calcRates(mocap)
(telRates, telStartIndex, telOffset) = calcRates(tel)
(joyRates, joyStartIndex, joyOffset) = calcRates(joy)
(radioRates, radioStartIndex, radioOffset) = calcRates(radioCmd)

plt.figure()
plt.plot(mocap['t'][mocapStartIndex+1:], mocapRates, label='mocap')
plt.plot(tel['t'][telStartIndex+1:] + telOffset, telRates, label='telemetry')
plt.plot(joy['t'][joyStartIndex+1:] + joyOffset, joyRates, label='joystick')
plt.plot(radioCmd['t'][radioStartIndex+1:] + radioOffset, radioRates, label='radio_cmd')

plt.legend()

plt.ylim((0, 300))

plt.show()