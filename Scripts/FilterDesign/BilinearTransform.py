# read the simulation log, and make a couple of plots
from __future__ import print_function, division

import sympy

order = 2
# The nominal, cts time butterworth filter:

coeffs_A = [1]
coeffs_B = [1]

s = sympy.symbols('s')
wc, dt = sympy.symbols('wc, dt') #cut-off and sampling period

if (order//2)*2 == order:
    #even order
    b_s = 1
    for kk in range(order//2):
        k = sympy.Integer(kk+1)
        term = s**2 - 2*s*sympy.cos((2*k + order-1)/(2*order)*sympy.pi) + 1
        b_s = b_s*term

else:
    #odd order
    b_s = s+1
    for kk in range((order-1)//2):
        k = sympy.Integer(kk+1)
        term = s**2 - 2*s*sympy.cos((2*k + order-1)/(2*order)*sympy.pi) + 1
        b_s = b_s*term

#Create continuous time filter with desired cutoff:
ss = sympy.symbols('ss')
aa = b_s.subs({s:ss/wc})

print('Order is',order)

ctsTimeTF = 1/aa
print('Continuous time TF is:')
print(ctsTimeTF)

#apply the bilinear transform
zinv = sympy.symbols('zinv')

discreteTimeTF = sympy.simplify(ctsTimeTF.subs({ss:((2/dt)*(1-zinv)/(1+zinv))}))
# discreteTimeTF = sympy.simplify(ctsTimeTF.subs({ss:(1-zinv)/dt}))

print('Discrete time TF is:')
print(discreteTimeTF)

B,A = sympy.fraction(discreteTimeTF)
A = sympy.collect(sympy.expand(A), zinv, evaluate = False)
B = sympy.collect(sympy.expand(B), zinv, evaluate = False)
print('A:',)
for i in A.keys():
    print(sympy.simplify(A[i]/A[1]),',', end='')
print(';')
    
print('B:',)
for i in B.keys():
    print(sympy.simplify(B[i]/A[1]),',', end='')
print(';')
    



