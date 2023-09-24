#R Navin sriram
# use the associated world file in the same given directory
# execute both Scripts using the extern controller command one after the other
# finish the simulation (not pause, rewind for the sim to end) to generate a csv file with diagnostic data in the scripts' directory
# a plot will also be plotted with all the results

from vehicle import Driver
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
from controller import DistanceSensor, Keyboard, Supervisor, Receiver
import atexit


#
driver = Driver()
keyboard = Keyboard()
keyboard.enable(32)

#receiver = driver.getDevice("receiver")
#receiver.enable(32)


# 0.5 of torque corresponds to an accel of 2.54 m/s^2
step = 32
speed =  0
Cd = 0.3            # coefficient of drag
A = 2.4             # frontal area
m = 2100            # mass in Kg
Rc = 0.01           # Rolling Resistance coefficient
Vmax = 8.3          # m/s
Amax_T = 5.02        # max Acceleration, Tesla 
Dmax_T = 7.12       # max deceleration ie. BrakingIntensity(1), Tesla
Amax = 1.2           # ozone parameters
Dmax = 1.4
Pmax = 5000         # max power output
Tmax = 110         # max wheel torque
R= 0.24            # radius of the wheel
Rpmmax = Vmax * 60 / (6.28 * R)
rev_speed = 25    # max rev speed in kmph
cmdAcc = 0
tg = 0                # 0 for constant distance, 1 for CTH

sensorsNames = ["front"]
sensors ={}

for name in sensorsNames:
    sensors[name] = driver.getDevice("distance sensor front")
    sensors[name].enable(10)

Dmin = 25    # minimum distance to maintain in a CTH environment
Th = 2       # time gap of 2s in addition to the Dmin 
Kp = 2        # gain of proportional controller
Kd = 2.828       # gain of derivative controller
step = 32  # time gap between two measurements

# air drag and rolling resistance
def drag(speed):
  speed=speed*5/18
  air_drag = 0.5*1.1*Cd*A*(speed**2)
  rolling_res=Rc*9.8*m
  drag_tot= air_drag + rolling_res
  drag_decel=drag_tot/m
  drag_decelf = min(Dmax,drag_decel) # to make sure that drag remains
  bi_drag= drag_decelf / Dmax # mapped to brake intensity
  return bi_drag

#Filter the speed command by averaging, to avoid abrupt speed changes.
def dist_filter(sep):                      
    dist_filter.previousDist.append(speed)
    if len(dist_filter.previousDist) > 5:  # keep only 5 values
        dist_filter.previousDist.pop(0)
    return sum(dist_filter.previousDist) / float(len(dist_filter.previousDist))
    
def speed_filter(speed):                      
    speed_filter.previousSpeed.append(speed)
    if len(speed_filter.previousSpeed) > 5:  # keep only 5 values
        speed_filter.previousSpeed.pop(0)
    return sum(speed_filter.previousSpeed) / float(len(speed_filter.previousSpeed))

dist_filter.previousDist = []
speed_filter.previousSpeed = []
# stores the speed of the car at i-1 th and i th instance
# used for getting the deriavtive
Vel = [0,15.0]
prev_speed = 0  

a = []
b = []
c = []
d = []

driver.setGear(1)

# checking condition for activating ACC
def vehicle_in_front(): 
    if sensors["front"].getValue()< 100 :
            return True
    return False

# finding Reference Separation
def Req_distance(current_speed):
    if tg == 0:
        Req_distance = Dmin
    else :
        Req_distance = Dmin + Th*current_speed
    return Req_distance

# finding Deirvative of filtered var to avoid random spikes
def Derivative(Err):
    derivative = (Err[1] - Err[0]) * step
    return derivative

# PID block to give accel commands
def cmd_acc(current_speed,front_sensor):
    if current_speed < 75:  
     err = -Req_distance(current_speed) + front_sensor
     cmd_acc = Kp * err + Kd * Derivative(Vel)
    else:
     cmd_acc=0.0
    return cmd_acc

def logging(track,speed,cmdAcc,frontDistance,a,b,c,d):
   a.append(track)
   print(a)
   b.append(speed)
   c.append(cmdAcc)
   d.append(frontDistance)

def save_file(aa,bb,cc,dd):
   df = pd.DataFrame({"Sample time" : np.array(aa), "speed" : np.array(bb), "accel": np.array(cc), "separation": np.array(dd)})
   df.to_csv("diagnostics.csv", index=False)
   
plt.style.use('fivethirtyeight')

'''def animate(a,b,c,d):
    
    x = a
    y1 = b
    y2 = c
    y3 = d

    plt.cla()

    plt.plot(x, y1, label='speed time')
    plt.plot(x, y2, label='acc time')
    plt.plot(x, y3, label='distance time')

    plt.legend(loc='upper left')
    plt.tight_layout()

ani = FuncAnimation(plt.gcf(), animate(a,b,c,d), interval=50)
plt.tight_layout()
plt.show()'''

# vehicle plant controller, can be optimised further
def Command(cmdAcc,filteredSpeed):
   flag = -1
   if cmdAcc > 0:
    flag = 1
 
   k=min(Amax,cmdAcc)/Amax_T
   l=-1*max(-1.4,Dmax)/Dmax_T
   
   cmd ={                 
   1:[k,0,0,"Accel"],    
   -1:[0,l,0,"Decel"]
   } 
   driver.setThrottle(cmd[flag][0])
   driver.setBrakeIntensity(-1*cmd[flag][1])
   acelll= ( cmd[flag][0] * Amax_T ) + (-1*cmd[flag][1]* Dmax_T)
   print(cmd[flag][3])
   logging(track,filteredSpeed,acelll,frontDistance,a,b,c,d)
  

def plotter(aa,bb,cc,dd):

 # Initialise the subplot function using number of rows and columns
 figure, axis = plt.subplots(2, 2)
  
 # For Sine Function
 axis[0, 0].plot(aa, bb)
 axis[0, 0].set_title("Speed v/ time")
  
 # For Cosine Function
 axis[0, 1].plot(aa, cc)
 axis[0, 1].set_title("Acceleration v/ time")
  
 # For Tangent Function
 axis[1, 0].plot(aa, dd)
 axis[1, 0].set_title("Separation v/ time")
  
 # For Tanh Function
 axis[1, 1].plot(dd, cc)
 axis[1, 1].set_title("Acceleration v/ distance")
 plt.show()     
            
track=0   
           
while driver.step() != -1:
      
    frontDistance = sensors["front"].getValue()
    speed = driver.getCurrentSpeed()
    filteredSpeed = speed_filter(speed)
    filteredDist = dist_filter(frontDistance)        
    Vel[0] = Vel[1]
    Vel[1] = filteredDist
    bi_drag = drag(speed)
    cmdAcc= cmd_acc(speed,frontDistance)
    track = track + 1/32
    print(track)
    
     
    #print(receiver.getQueueLength())
    #stringg=receiver.getString()   
     
    # checking for ACC condition
    if vehicle_in_front():
         if prev_speed > 30:
           cmdAcc=0
         Command(cmdAcc,filteredSpeed)
         prev_speed = driver.getCurrentSpeed()
         
                 
    else :
    # cruise at 30kmph when ACC inactive
       print("Inactive")
       driver.setGear(1)
       if filteredSpeed < 29.85:
         driver.setThrottle(0.1)
         driver.setBrakeIntensity(0.0)
         acelll= 0.1 * Amax_T
       else:
        driver.setCruisingSpeed(30)
        acelll= 0
       logging(track,filteredSpeed,acelll,frontDistance,a,b,c,d)
      
       print(" Cruising at max speed")
       prev_speed = driver.getCurrentSpeed()
    print("ego speed is", prev_speed)

atexit.register(plotter, aa=a, bb= b, cc = c, dd=d)
atexit.register(save_file, aa=a, bb= b, cc = c, dd=d)
