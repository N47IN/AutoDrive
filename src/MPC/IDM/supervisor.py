""" Supervisor which runs the optimization """

"""
reciver listens to channel 5
emitter is on channel 4

Data is recived in the same order as it is sent
so the timings should be same
"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')
from scipy.signal import cont2discrete
import time
import sys
import matplotlib.pyplot as plt

plt.ion()


from controller import Supervisor

## initialization
Ts = 0.05
TIME_STEP = int(1000*Ts)
supervisor = Supervisor()
receiver = supervisor.getDevice("receiver")
emitter = supervisor.getDevice("emitter")
receiver.enable(TIME_STEP)

import numpy as np

'''
v0:max speed of the car
delta: 4
a:max acceleration m/s^2
b:normal deceleration m/s^2
s0: 2 m least safe distance between two cars
    --> exceed this means need to brake immediately
:param T: 1.5 human reaction time T for s*
'''
v0 = 28
a = 2.5
b = 2
s0 = 2
T = 1.5
delta = 4

def initialize(v_table):
    dt = 0.5 #relaxation timestep 0.5s
    kmax = 20 #number of iteration in relaxation
    for s in range(1, ismax+1):
        Ve = v_table[s-1]
        for k in range(0, kmax):
            s_star = s0 + Ve*T
            acc = a * (1.-np.power(Ve/v0, delta) - (s_star**2) / (s**2))
            Ve += acc * dt/kmax
            Ve = max(Ve, 0) # can't be lower
        v_table[s] = Ve
    return v_table

def idm_acc(dx ,v_host, v_ref, v_table ):
    s = int(np.floor(dx))
    acc = 0
    V = 0
    if s < 0:
        pass # V=0
    elif s < ismax:
        # rest = dx - s
        # V = (1-rest) * v_table[s] + rest * self.v_table[s+1]
        delta_v = v_host - v_ref
        s = dx - 2
        vel = v_host
        s_star_raw = s0 + vel*T + (vel * delta_v) / (2 * np.sqrt(a*b))
        s_star = max(s_star_raw, s0)
        acc = a * (1 - np.power(vel / v0, delta) - (s_star **2) / (s**2))
        acc = max(acc, -2.5)

    else:
        V = v_table[ismax]
    return acc , V

message = ''
ismax = 100 # ve(s) = ve(ismax) if s > ismax
point_index = 0
v_table = np.zeros(ismax + 1)
v_table = initialize(v_table)

while supervisor.step(TIME_STEP) != -1:
    point_index +=1
    # recive the data
    # print("this is sup Q length",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        x0 = np.frombuffer(message, dtype=np.float64)
        x0 = x0.reshape(3,1)
        receiver.nextPacket()

        v_host, v_ref = x0[1] , x0[2]
        dx = x0[0]

        acc2 , V = idm_acc(dx ,v_host, v_ref , v_table )
        acc = v_ref - v_host
        if (dx) > s0:
            acc = V - v_host
        acc = max(acc2, acc+.2) #can't exceed the maximum value

        #plt.figure(" dx")
        #plt.scatter(point_index,x0[0])

        #plt.figure("v_host")
        #plt.scatter(point_index,x0[1])

        #plt.figure("accel")
        #plt.scatter(point_index,acc)

        #plt.show()
        #plt.pause(0.01)

        if acc:
            message = acc.tobytes()

    if message != '' : #and message != previous_message:
        # previous_message = message
        emitter.send(message)
