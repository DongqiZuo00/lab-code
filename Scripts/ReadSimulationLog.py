# read the simulation log, and make a couple of plots

import numpy as np
import matplotlib.pyplot as plt

# read the data:
fname = '../Logs/Simulation.csv'
data = np.genfromtxt(fname, delimiter=',')

t = data[:, 0]
pos = data[:, 1:4]
vel = data[:, 4:7]
attYPR = data[:, 7:10]
angVel = data[:, 10:13]
motForces = data[:, 13:17]

estPos = data[:, 17:20]
estVel = data[:, 20:23]
estAttYPR = data[:, 23:26]
estAngVel = data[:, 26:29]

cmdPos = data[:, 29:32]

cmdThrust = data[:,33]
cmdAngVel = data[:,34:38]

# plot the data:
fig = plt.figure()
n = 5  # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i, sharex=fig.axes[0])

colours = 'rgb'
dim = 'xyz'
angle = 'YPR'
for i in range(3):  # over each spatial axis:
    fig.axes[0].plot(t, pos[:, i], colours[i], label=dim[i])
    fig.axes[0].plot(t, estPos[:, i], ':' + colours[i], label='est_' + dim[i])
    fig.axes[0].plot(t, cmdPos[:, i], '--' + colours[i], label='cmd_' + dim[i])
    fig.axes[1].plot(t, vel[:, i], colours[i], label=dim[i])
    fig.axes[1].plot(t, estVel[:, i], ':' + colours[i])
    fig.axes[2].plot(t, attYPR[:, i] / np.pi * 180, colours[i], label=angle[i])  # convert to degrees
    fig.axes[2].plot(t, estAttYPR[:, i] / np.pi * 180, ':' + colours[i])
    fig.axes[3].plot(t, angVel[:, i], colours[i], label=dim[i])
    fig.axes[3].plot(t, estAngVel[:, i], ':'+colours[i])
    fig.axes[3].plot(t, cmdAngVel[:, i], '--'+colours[i])
    
fig.axes[4].plot(t, motForces)

for a in fig.axes[0:4]:
    a.legend()
    
fig.axes[-1].set_xlabel('Time [s]')
fig.axes[0].set_ylabel('Position [m]')
fig.axes[1].set_ylabel('Velocity [m/s]')
fig.axes[2].set_ylabel('Attitude [deg]')
fig.axes[3].set_ylabel('AngVel [rad/s]')
fig.axes[4].set_ylabel('Motor forces [N]')

plt.show()
