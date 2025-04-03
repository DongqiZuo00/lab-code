# read the simulation log, and make a couple of plots
from __future__ import print_function, division

import numpy as np
import matplotlib.pyplot as plt


class Filter:
    def __init__(self, dt, wc, order):
        sqrt = np.sqrt
        if order == 1:
#             self._coeffs_A = np.array([-1/(dt*wc + 1)])
#             self._coeffs_B = np.array([dt*wc/(dt*wc + 1)])
            self._coeffs_A = np.array([(dt*wc - 2)/(dt*wc + 2)])
            self._coeffs_B = np.array([dt*wc/(dt*wc + 2) ,dt*wc/(dt*wc + 2) ])
        elif order == 2:
            self._coeffs_A = np.array([(dt**2*wc**2 - 2*sqrt(2)*dt*wc + 4)/(dt**2*wc**2 + 2*sqrt(2)*dt*wc + 4) ,2*(dt**2*wc**2 - 4)/(dt**2*wc**2 + 2*sqrt(2)*dt*wc + 4) ])
            self._coeffs_B = np.array([dt**2*wc**2/(dt**2*wc**2 + 2*sqrt(2)*dt*wc + 4) ,dt**2*wc**2/(dt**2*wc**2 + 2*sqrt(2)*dt*wc + 4) ,2*dt**2*wc**2/(dt**2*wc**2 + 2*sqrt(2)*dt*wc + 4)])

        self._n_A = self._coeffs_A.shape[0]
        self._n_B = self._coeffs_B.shape[0]-1
            
        self._values_A = np.zeros([self._n_A,])
        self._values_B = np.zeros([self._n_B,])
        print('A', self._coeffs_A)
        print('B', self._coeffs_B)
        return
    
    def run(self, value):
        out = self._coeffs_B[-1]*value
        for i in range(self._n_B):
            out += self._values_B[i]*self._coeffs_B[i] 

        for i in range(self._n_A):
            out += -self._values_A[i]*self._coeffs_A[i] 
            
        for i in range(self._values_B.shape[0]-1):
            self._values_B[i] = self._values_B[i+1]
        if self._n_B:
            self._values_B[-1] = value
            
        for i in range(self._n_A-1):
            self._values_A[i] = self._values_A[i+1]
        self._values_A[self._n_A-1] = out
            
#         print(self._values_A)
#         print(out)
        return out
    
    
    
    
dt = 0.002  #s
tEnd = 1 #s
ctsTimeCutoff = 100 # rad/s

# winput = 0.1*ctsTimeCutoff
winput = 1*ctsTimeCutoff

order = 2

filt = Filter(dt, ctsTimeCutoff, order)


nSteps = np.int(tEnd/dt+0.5)

t = np.zeros([nSteps,])
x = np.zeros([nSteps,])
y = np.zeros([nSteps,])

for i in range(nSteps):
    t[i] = i*dt
    u = np.sin(t[i]*winput)
        
        
    x[i] = u
    y[i] = filt.run(u)
    
    
fig = plt.figure()
fig.add_subplot(1, 1, 1)
fig.axes[0].plot(t,x,'b',label='x')
fig.axes[0].plot(t,y,'r',label='y')
fig.axes[0].set_xlabel('order = '+str(order))

plt.show()
