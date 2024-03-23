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
k=1
prev_k = 1
while driver.step()!=-1:
   curr_pos = gps.getValues()
   
   

   
   
   if i==120:
 
      AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.988420, 80.228086)
      #AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.992449, 80.227444)
 
      coordinates_LL = AutoDrive.ShortestPath()
      coordinates_LL[:,0], coordinates_LL[:,1] = coordinates_LL[:,0] + 0.000033, coordinates_LL[:,1] + 0.000053
      coordinates_cartesian = np.asarray(AutoDrive.getCartesian(coordinates_LL))
      coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
      coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
      coordinates_cartesian[:,1] = coordinates_cartesian[:,1] - 2
      """ coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
      coordinates_cartesian[:,1] = coordinates_cartesian[:,1] + 0.5 """
      coordinates_bezier = np.asarray(AutoDrive.bezier_curve(coordinates_cartesian))
      coordinates_cartesian = np.delete(coordinates_cartesian, (0),axis =0)   
      print(coordinates_bezier)
      print(coordinates_cartesian)
      stanley = Stanley.LatControl(coordinates_cartesian)  
      AutoDrive.plotDubins(coordinates_cartesian,curr_pos)
      prev_stter = 0

   elif i>120 : 
      #AutoDrive.update(curr_pos)
      #curr_pos = gps.getValues()  
      #AutoDrive.show_graph(curr_pos[0:2])
      curr_pos = llc(curr_pos)
      driver.setCruisingSpeed(30)
      speed = driver.getCurrentSpeed()*5/18
      steering = driver.getSteeringAngle()
      obs = llc([12.991664, 80.231718])
      obs = np.asarray([obs[0],obs[1]])
      curr_coord = np.asarray([curr_pos[0],curr_pos[1]])
      if np.linalg.norm(curr_coord - obs) > 13:
         steer_angle, k = stanley.steer_control(curr_pos,yaw,speed)
         print("Stanley")
      else:
         print("Avoidance")
         print(curr_coord,obs,yaw)
         #[1054453.18935894 9187157.01811257]
          #[1054440.25544082 9187156.53298734]
         #-0.49735188283943693

         dubins_avoid = stanley.get_alternate_dubinsRRT(curr_coord,obs,yaw)
         print(dubins_avoid)
         plt.plot(dubins_avoid[0],dubins_avoid[1])
         plt.pause(2)
         plt.close()
         break
      
      #print(steer_angle)
      if steer_angle is None:
         steer_angle = prev_stter
      
      prev_stter = steer_angle
      driver.setSteeringAngle(steer_angle)
   
   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   i +=1  
   prev_k = k
   





