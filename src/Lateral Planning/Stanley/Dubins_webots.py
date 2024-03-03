import OSMnav
from OSMnav import llc
import Stanley
from controller import Robot, Keyboard
from controller import InertialUnit
from vehicle import Driver
import matplotlib.pyplot as plt
import numpy as np


l = []
action = []
coords = []

coordinates_cartesian_transformed = []
driver = Driver()
Yaw = driver.getDevice("inertial unit")
gps = driver.getDevice("gps")
driver.setGear(1)
timestep = 10
Yaw.enable(timestep)
gps.enable(timestep)

AutoDrive = OSMnav.OSMnav()
start_pos = gps.getValues()

""" inp = int(input("Enter the destination, 1 for GC, 2 for ED Building, 3 for Mandakini"))
Destinations = { 1:[12.991510, 80.233733], 2:[12.989981, 80.226453],2 :[12.986709, 80.239436]}
 """
i = 0


while driver.step()!=-1:
   curr_pos = gps.getValues()
   
   
   if i==120:
      #print("hi")
      #print(llc(curr_pos))
      AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.991650, 80.233696)
      coordinates_LL = AutoDrive.ShortestPath()
      #print(coordinates_LL)
      coordinates_cartesian = np.asarray(AutoDrive.getCartesian(coordinates_LL))
      coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
   
      stanley = Stanley.LatControl(coordinates_cartesian)
      AutoDrive.Dubin(coordinates_cartesian,1)
      AutoDrive.plotDubins(coordinates_cartesian)
      #print(AutoDrive.getCartesian([[12.99305, 80.24142]]))
      
      
     

   elif i>120:
      curr_pos = llc(curr_pos)
      driver.setCruisingSpeed(20)
      #print(driver.minSteeringAngle())
      speed = driver.getCurrentSpeed()*5/18
      steer_angle, j = stanley.steer_control(curr_pos,yaw,speed)
      #AutoDrive.plotCar(coordinates_cartesian,curr_pos)
      #print(steer_angle)
      driver.setSteeringAngle(steer_angle)
      
   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   i +=1  
   
   





