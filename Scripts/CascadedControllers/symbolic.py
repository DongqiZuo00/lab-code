from __future__ import division, print_function

import sympy as sympy
import numpy as np
from numpy import rate

#system state
pos, vel, pitch, rate = sympy.symbols('pos, vel, pitch, rate', real=True)

#ref position
refPos = sympy.symbols('refPos')
moment = sympy.symbols('moment')

#constants:
g = sympy.symbols('g')
Ixx = sympy.symbols('Ixx')

posCtrl_wn, posCtrl_zeta = sympy.symbols('posCtrl_wn, posCtrl_zeta')
att_timeConst, angVel_timeConst = sympy.symbols('att_timeConst, angVel_timeConst')

#controllers:
desAcc = -2*posCtrl_zeta*posCtrl_wn*vel - posCtrl_wn**2*(pos-refPos)
desPitch = desAcc/g
desRate = -(pitch-desPitch)/att_timeConst
desAngAcc = -(rate-desRate)/angVel_timeConst
momentCmd = Ixx*desAngAcc

#dynamics

dpos = vel
dvel = g*pitch
dpitch = rate
drate = moment/Ixx

x = sympy.Matrix([[pos, vel, pitch, rate]]).T
dx = sympy.Matrix([[dpos, dvel, dpitch, drate]]).T

Aol = dx.jacobian(x)
Bol = dx.jacobian(sympy.Matrix([[moment]]))

print("Open loop")
print(Aol)
print(Bol)

K = -sympy.Matrix([[momentCmd]]).jacobian(x)
print(K)

#Values