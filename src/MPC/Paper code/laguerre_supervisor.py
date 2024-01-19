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

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

def kalman(A,B,x,P,u,z_odometry):

    H_odo = np.array([[1 , 0],[0,1]])  # Measurement matrix for odometry

    R_odometry = np.array([[0.1, 0],
                           [0, 0.2]])  # Odometry measurement noise covariance

    Q = np.array([[0.1, 0],
                  [0, 0.01]]) # Define process noise covariance

    # Prediction
    x_hat = np.dot(A, x) + np.dot(B, u)
    P_hat = np.dot(np.dot(A, P), A.T) + Q

    # Kalman Gain for odometry
    K_odometry = np.dot(np.dot(P_hat, H_odo.T), np.linalg.inv(np.dot(np.dot(H_odo, P_hat), H_odo.T) + R_odometry))
    # Update using both measurements
    x = x_hat +  np.dot(K_odometry, (z_odometry - np.dot(H_odo, x_hat)))
    P = P_hat - np.dot(np.dot(K_odometry , H_odo), P_hat)

    return x , P

def lagd(a, N):
    v = np.zeros(N)
    Lo = np.zeros(N)
    v[0] = a
    Lo[0] = 1

    for k in range(1, N):
        v[k] = ((-a) ** (k - 1)) * (1 - a * a)
        Lo[k] = (-a) ** (k)

    Lo = np.sqrt((1 - a * a)) * Lo
    # Lo = Lo.reshape(-1,1)
    A = np.zeros((N, N))
    A[:, 0] = v

    for i in range(1, N):
        A[:, i] = np.concatenate([np.zeros(i), v[0:N - i]])
    # print(Lo)
    return A, Lo

def dmpc(Ae, Be, a, N, Np, Q, R):
    #Ae and Be are the matrices of the augmented system when integrator is used
    # They can also be other forms of state space model
    # a is the parameter of the laguerre network
    # N is the number of terms for each input
    # Np is the prediction horizon
    # Q and R are the cost matrices

    #cost function is J = eta^T E eta + 2 eta^T H x(k_i)
    #Δui(k) = Li(k)T ηi,

    n, _ = Be.shape
    n_in = 1
    Npa = N[0]  # dimention of eta = sum of all the values of Ns
    E = np.zeros((Npa,Npa))
    H = np.zeros((Npa,n))
    R_para = np.zeros((Npa,Npa))
    n0 = 0
    ne = N[0]
    for i in range(0,n_in):
        R_para[n0:ne,n0:ne] = R[i,i]*np.eye(N[i],N[i])

    # Now initial condition for the convolution sum
    Sin = np.zeros((n,Npa))

    [A1,Lo] = lagd(a[0],N[0])
    Lo = Lo.reshape(-1, 1)
    # print(Be.shape, Lo.shape)
    Sin[:,0:N[0]] = Be@np.transpose(Lo)

    Sc = Sin
    E = np.transpose(Sc)@Q@Sc
    H = np.transpose(Sc)@Q@Ae
## The iteration i is with respect to the prediction horizon
    for i in range(0,Np):
        Eae = np.linalg.matrix_power(Ae, i+1)
        [A1,Lo] = lagd(a[0],N[0])
        Lo = Lo.reshape(-1,1)
        Sc[:,0:N[0]] = np.linalg.matrix_power(Ae, i-1)@Sc[:,0:N[0]] + Be@np.transpose(np.linalg.matrix_power(A1, i-1)@Lo)
        E = E + np.transpose(Sc)@Q@Sc
        H = H + np.transpose(Sc)@Q@Eae
    E = E + R_para

    return E,H

def Mdu(a, N, Nc, n_in=1):
    N_pa = np.sum(N)
    M = np.zeros((n_in, N_pa))
    M_du1 = np.zeros((n_in, N_pa))
    k0 = 0
    Al, L0 = lagd(a[k0], N[k0])
    M_du1[k0, :N[k0]] = np.transpose(L0)
    cc = N[k0]

    for k0 in range(1, n_in):
        Al, L0 = lagd(a[k0], N[k0])
        M_du1[k0, cc:cc+N[k0]] = np.transpose(L0)
        cc += N[k0]

    Lzerot = np.copy(M_du1)
    M = np.copy(M_du1)

    for kk in range(2, Nc+1):
        k0 = 0
        Al, L0 = lagd(a[k0], N[k0])
        L = np.linalg.matrix_power(Al, kk-1) @ L0
        M_du1[k0, :N[k0]] = np.transpose(L)
        cc = N[k0]

        M = np.vstack((M, M_du1))

    return M, Lzerot

def QPHild(E, F, M, gamma):
    # inputs are H f A_cons b  in order
    # Determine which constraints are active and which are inactive
    n1 = M.shape[0]
    m1 = M.shape[1]
    eta = -np.linalg.solve(E, F)
    kk = 0

    for i in range(n1):
        if np.dot(M[i], eta) > gamma[i]:
            kk += 1

    if kk == 0:
        return eta

    P = np.dot(M,np.dot(np.linalg.inv(E),M.T))
    d = np.dot(M,np.dot(np.linalg.inv(E),F)) + gamma

    n, m = d.shape
    x_ini = np.zeros((n, m))
    lam = x_ini
    al = 10

    # 40 is the number of iterations to
    for km in range(40):
        lambda_p = np.copy(lam)

        for i in range(n):
            w = np.dot(P[i,:],lam) - np.dot(P[i,i],lam[i])
            w = w + d[i]
            la = -w/P[i,i]
            lam[i,0] = max(0,la)
            #print(lam[i,0])
        al = np.dot((lam-lambda_p).T,lam-lambda_p)

        if al < 10e-8:
            break

    eta = eta  - np.dot(np.linalg.solve(E, M.T), lam)
    return eta

def Mu(a, N, Nc, n_in=1):
    N_pa = np.sum(N)
    M = np.zeros((n_in, N_pa))
    M_du1 = np.zeros((n_in, N_pa))
    k0 = 0
    Al, L0 = lagd(a[k0], N[k0])
    M_du1[k0, :N[k0]] = np.transpose(L0)
    cc = N[k0]

    for k0 in range(1, n_in):
        Al, L0 = lagd(a[k0], N[k0])
        M_du1[k0, cc:cc+N[k0]] = np.transpose(L0)
        cc += N[k0]

    M1 = np.copy(M_du1)
    M = np.copy(M_du1)

    for kk in range(1, Nc):
        k0 = 0
        Al, L0 = lagd(a[k0], N[k0])
        L = np.linalg.matrix_power(Al, kk-1) @ L0
        M_du1[k0, :N[k0]] = np.transpose(L)
        cc = N[k0]

        M = M + M_du1
        M1 = np.vstack((M1, M))

    return M1

## Constrints are clamping type
def simCon3(xm,y, A1, A2, B1 , B2, C, N_sim, Omega1, Omega2, Psi1, Psi2, Lzerot, M0, M1):
    m1, n1 = Ce.shape
    u = np.zeros((1, 1))
    yr = np.array([[0], [0], [0]])
    # n1, _ = Bp.shape
    n_in = 1
    Lzerot = Lzerot.reshape(-1,1)

    u_max = 2.5
    u_min = -5.5
    delu_min = -1
    delu_max = 1
    Nc = 15   # number of steps on which limit has to be imposed
    M = np.vstack((M0,-M0,M1, -M1))


    u1 = np.zeros((N_sim, 1))
    y1 = np.zeros((m1, N_sim))
    deltau1 = np.zeros((N_sim,1))
    xf = np.vstack((xm, (y - yr)))

    for kk in range(N_sim):
        u_prev = u
        gamma = np.vstack(((u_max - u_prev) * np.ones((Nc, 1)),
                    -(u_min - u_prev) * np.ones((Nc, 1))
                    , delu_max * np.ones((Nc,1))
                    , -delu_min * np.ones((Nc,1))))
        # print(f"this is Omega {Omega.shape}, Psi {Psi.shape}, M {M.shape}, gamma {gamma.shape}")

        if u >=0:
            eta = QPHild(Omega1, Psi1@xf, M, gamma)
        elif u < 0:
            eta = QPHild(Omega2, Psi2@xf, M, gamma)
        # Kmpc = np.transpose(Lzerot) @ np.linalg.solve(Omega, Psi)
        # Ke = Kmpc[0,3:6].reshape(1,-1)
        deltau = np.transpose(Lzerot) @ eta   #
        if deltau[0] > delu_max:
            deltau[0] = delu_max
        if deltau[0] < delu_min:
            deltau[0] = delu_min
        u += deltau
        if u[0] > u_max:
            u[0] = u_max
        if u[0] < u_min:
            u[0] = u_min

        xm_old = xm
        deltau1[kk] = deltau
        u1[kk] = u
        if u >= 0:
            xm = A1@xm + B1@u
        elif u <0:
            xm = A2@xm + B2@u
        y = C@xm
        # plt.figure("computed accel")
        # plt.scatter(kk,xm[2])
        # xf = (Ae - Be@Kmpc) @ xf + Be @ Ke @ yr #xf = Ae @ xf + Be @ deltau
        # y = Ce @ xf

        y1[:,kk] = y.reshape(3,)
        xf = np.vstack((xm - xm_old, y - yr)) #Xf = np.vstack((xm - xm_old, y - sp[:, kk + 1]))

    k = np.arange(N_sim)

    # plt.figure("computed U")
    # plt.plot(k,u1)

    return u1, y1, deltau1, k


## initialization
Ts = 0.05
TIME_STEP = int(1000*Ts)
supervisor = Supervisor()
receiver = supervisor.getDevice("receiver")
emitter = supervisor.getDevice("emitter")
receiver.enable(TIME_STEP)

accel = 0
brake = 0

T_eng =  0.460 #0.26
K_eng = 0.732  # can add second order term if error is more
T_brake = 0.193
K_brake = 0.973
A_f = -1/T_eng
B_f = K_eng/T_eng
A_b = -1/T_brake
B_b = K_brake/T_brake

C = np.eye(3)
T_hw = 3
Ts = 0.05
T_total = 20
T = int(T_total/Ts)

#Discretize the system
A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B = np.array([[0], [0], [B_f]])
sys2 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A1, B1, C, D , dt = sys2

A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_b]])
B = np.array([[0], [0], [B_b]])
sys3 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A2, B2, C, D , dt = sys3

Ak1 = A1[1:,1:]
Bk1 = B1[1:]

Ak2 = A2[1:,1:]
Bk2 = B2[1:]

m1,n1 = C.shape  # m1 = 3, n1 = 3
n1,n_in = B.shape  # n_in = 1

a1 = 0.85  #0.6 and weight 10*Ce@Ce and R - 1, a = 5 - 7  ( 7 is slow, 5 is fast response)
N1 = 7
Np = 50

a = [a1]
N = [N1]

# Augment the state equations
Ce = np.zeros((m1,n1+m1))
Ce[:,n1:n1+m1] = np.eye(m1,m1)

Ae1 = np.eye(n1+m1,n1+m1)
Ae1[0:n1,0:n1] = A1
Ae1[n1:n1+m1,0:n1] = C@A1
Be1 = np.zeros((n1+m1,n_in))
Be1[0:n1,:] = B1
Be1[n1:n1+m1,:] = C@B1

Ae2 = np.eye(n1+m1,n1+m1)
Ae2[0:n1,0:n1] = A2
Ae2[n1:n1+m1,0:n1] = C@A2
Be2 = np.zeros((n1+m1,n_in))
Be2[0:n1,:] = B2
Be2[n1:n1+m1,:] = C@B2


Q1 = 1.4*np.transpose(Ce)@Ce
Q2 = 0.7*np.transpose(Ce)@Ce
# print(Q.shape)
R1 = 0.8*np.eye(1,1)
R2 = 0.8*np.eye(1,1)

# Initialize variables
N_sim = 50
Nc = 15

u = np.zeros((n_in, 1))
xm = np.array([[50], [10], [2]])
y = xm.copy()
yr = np.array([[0], [0], [0]])
M1, Lzerot = Mdu(a, N,Nc,n_in)
M0 = Mu(a, N,Nc,n_in)
_, Lz = lagd(a[0], N[0])


Omega1, Psi1 = dmpc(Ae1, Be1, a, N, Np, Q1, R1)
Omega2, Psi2 = dmpc(Ae2, Be2, a, N, Np, Q2, R2)
P = np.array([[1, 0],  # Initial velocity covariance
              [0, 1]]) # Initial accel covariance

message = ''

y1 = np.array([[50], [10], [2]])
path = [-100]
failure_iter  = 0
point_index = 0
x0 = np.array([0,0,0])
x0 = x0.reshape(3,1)
command = np.array([0])

while supervisor.step(TIME_STEP) != -1:
 
    point_index +=1
    # recive the data
    # print("this is sup Q length",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        x_prev = x0
        x0 = rnd(np.frombuffer(message, dtype=np.float64))
        x0 = x0.reshape(3,1)
        receiver.nextPacket()
        # plt.figure("del D")
        # plt.scatter(point_index,x0[0])

        # plt.figure("accel")
        # plt.scatter(point_index,x0[2])

        if x0[2]<0:
            xk , P = kalman(Ak2, Bk2, x_prev[1:],P,command[0],x0[1:])
        else:
            xk , P = kalman(Ak1, Bk1, x_prev[1:],P,command[0],x0[1:])

        x0 = np.array([x0[0], *xk])
        x0 = x0.reshape(3,1)

        path , y , deltau1 , k=simCon3(x0,y1,A1,A2,B1,B2,C,Np,Omega1, Omega2,Psi1, Psi2,Lz,M0, M1)

        plt.show()
        plt.pause(0.01)

        y1 = y[:,0].reshape(3,1)

        cmd = []
        if path[0]!= -100 and len(path)>1:
            i=0
            for point in path:
                cmd.append(point[-1])
                i+=1
                if i>5:
                    break
        else:
            cmd = -100
            failure_iter += 1

        command = np.array(cmd)
        message = command.tobytes()

    if message != '' : #and message != previous_message:
        # previous_message = message
        emitter.send(message)
