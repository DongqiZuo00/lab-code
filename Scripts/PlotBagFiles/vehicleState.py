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
joy = dat['/joystick_values']
radioCmd = dat['/radio_command'+vehID]
       
# Create a figure and a 2x1 grid of subplots
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

# Plot data on the first subplot
axs[0].plot(tel['t'], 'o-', label='tel[t]')
axs[0].set_ylabel('tel[t]')
axs[0].legend()
axs[0].grid(True)

# Plot data on the second subplot
axs[1].plot(radioCmd['t'], 's-', label='radioCmd[t]')
axs[1].set_xlabel('Index')
axs[1].set_ylabel('radioCmd[t]')
axs[1].legend()
axs[1].grid(True)

# Adjust layout for better appearance
plt.tight_layout()
plt.show()
         
fig = plt.figure()
fig.suptitle(fname)
n = 5  # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i, sharex=fig.axes[0])    

# Plot position
for j in range(3):
    fig.axes[0].plot(est['t'], est['pos'+'xyz'[j]], linestyle=':', color='rgb'[j], label='est_'+'xyz'[j])
    fig.axes[0].plot(mocap['t'], mocap['pos'+'xyz'[j]], color='rgb'[j], label='mocap_'+'xyz'[j])
fig.axes[0].set_ylabel('Position')
fig.axes[0].legend()

# Plot Velocity
for j in range(3):
    fig.axes[1].plot(est['t'], est['vel'+'xyz'[j]], color='rgb'[j], label='est_'+'xyz'[j])
fig.axes[1].set_ylabel('Velocity')
fig.axes[1].legend()

# Plot Attitude
att = ["roll", "pitch", "yaw"]
for j in range(3):
    fig.axes[2].plot(est['t'], est['att'+att[j]] * 180.0/np.pi, linestyle=':', color='rgb'[j], label='est_'+att[j])
    fig.axes[2].plot(mocap['t'], mocap['att'+att[j]] * 180.0/np.pi, color='rgb'[j], label='mocap_'+att[j])
fig.axes[2].set_ylabel('Attitude [deg]')
fig.axes[2].legend()

# Plot Angular Velocity
angVel = ["p","q","r"]
for j in range(3):
    fig.axes[3].plot(tel['t'], tel['rateGyro'][:,j], color='rgb'[j], label='gyro_'+angVel[j])
    fig.axes[3].plot(radioCmd['t'], radioCmd['debugvals'][:,1+j], linestyle='--', color='rgb'[j], label='des_'+'rpy'[j])
fig.axes[3].set_ylabel('Angular Velocity')
fig.axes[3].legend()

# Plot Motor Forces
for j in range(4):
    fig.axes[4].plot(tel['t'], tel['motorForces'][:,j], color='rgbc'[j], label='mot '+str(j+1))
fig.axes[4].set_ylabel('Motor forces [N]')
fig.axes[4].legend()

fig.axes[-1].set_xlabel('Time [s]')

plt.show()
