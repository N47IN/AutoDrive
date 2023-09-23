# R Navin Sriram / 18/8/23
from controller import Robot, Keyboard
from vehicle import Driver
import numpy as np
import math

# based on experimental tests for the Tesla model S,
# Brake Intensity of 0.9 delivered a deceleration of 6.43 m/s^2
# so we take max braking deceleration as 6.43/0.9

step=32
speed=0
Cd=0.3            # coefficient of drag
A=2.4             # frontal area
m=2100            # mass in Kg
Rc=0.01           # Rolling Resistance coefficient
Vmax=8.3          # m/s
Amax = 1.2        # max Acceleration
Dmax = 1.4        # max deceleration ie. BrakingIntensity(1)
Pmax=5000         # max power output
Tmax =110         # max wheel torque
R=0.24            # radius of the wheel
Rpmmax= Vmax * 60 / (6.28 * R)
rev_speed = 25    # max rev speed in kmph

driver = Driver()
keyboard= Keyboard()
keyboard.enable(step)

def checkPower(speed, cmd_trq):
   speed=speed
   power = cmd_trq * Tmax * speed / R       # checking for motor power
   print("Power :",power)
   print("speed",speed)
   print("Torque output", cmd_trq*Tmax)
   if power == Pmax:
      cmd_trq = Pmax * R / (Tmax * speed)   # constant power region for a PMSM motor
      print(" Max nominal Power reached ! ")
      
    
def drag(speed):
  air_drag = 0.5*1.1*Cd*A*(speed**2)
  rolling_res=Rc*9.8*m
  drag_tot= air_drag + rolling_res
  drag_decel=drag_tot/m
  drag_decelf = min(Dmax,drag_decel) # to make sure that drag remains
# within the bounds of brake intensity
  bi_drag= drag_decelf / Dmax # mapped to brake intensity
  return bi_drag

""" def steer_restore(steer):
   if steer!=0:
      steer= steer - np.sign(steer)*0.01
   return(steer) """

def drive(cmd_trq,cmd_steer):
   checkPower(cmd_trq,speed)
   driver.setThrottle(cmd_trq)
   driver.setSteeringAngle(cmd_steer)

#first element is the cmd_trq, second is stee-
#-ring angle, third is gear, fourth is brakes
command ={                 
   ord("W"):[0.5,0,1,0],    
   ord("A"):[0,-0.3,0,0],
   ord("D"):[0,0.3,0,0],
   ord("S"):[0.5,0,-1,0],
   ord(" "):[0,0,0,1]
   }

while driver.step() != -1:
                        
    speed=driver.getCurrentSpeed()*5/18  # kmph -> m/s
    key=keyboard.getKey()
    bi_drag=drag(speed)                   # setting the drag
       
    if key in command.keys():
       driver.setGear(command[key][2])
       drive(command[key][0],command[key][1])
       driver.setBrakeIntensity(bi_drag + 0.5*command[key][3])
       
    else:
       drive(0,0)
