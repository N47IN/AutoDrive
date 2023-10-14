# usr/bin#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from controller import Driver, DistanceSensor
import numpy as np
import pandas as pd

speed =  0
Vmax = 8.3          # m/s
Amax = 1.2           # ozone parameters
Dmax = 1.4           # ozone parameters
cmdAcc = 0
Ascale = 1/3        # scaling acc from ozone to p3
Dscale = 1/4        # scaling dec from ozone to p3
Amax_T = 5.02        # max Acceleration, Tesla 
Dmax_T = 7.12
Dmin = 2            # minimum distance to maintain in a CTH environment 
Dact = 5
Kp = 2              # gain of proportional controller
Kd = 2.828          # gain of derivative controller
step = 32


class ros_nav():

    def __init__(self):
        rospy.init_node('P3_AT', anonymous=True)    
        self.pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10)
        self.dist_filter.previousDist = []
        self.speed_filter.previousSpeed = []
        self.driver = Driver()
        self.sensors={}
        self.sensors["front"] = self.driver.getDevice("distance sensor front")
        self.sensors["front"].enable(step)
        self.currentSpeed = 0
        self.current_acc = 0
        self.prev_speed = 0
        self.Vel = [0,15.0]
        rospy.spin()
        
    def publisher(self,cmd):
        msg = Twist()
        msg.linear.x = cmd
        msg.angular.z = 0.0
        msg.linear.y = 0.0
        rospy.loginfo(cmd)
        self.pub.publish(msg)

    def ACC(self,data):
     while self.driver.step() != -1:
       self.frontDistance = self.sensors["front"].getValue()
       self.frontDistance = self.dist_filter(self.frontDistance)
       self.speed = self.driver.getCurrentSpeed()*5/18
       self.speed = self.speed_filter(self.speed)
       self.Vel[0] = self.Vel[1]
       self.Vel[1] = self.frontDistance

       if self.frontDistance < Dact:
         self.current_acc = self.Command()
         
       else :
           print("Inactive")
           self.currentSpeed = Vmax
           self.current_acc = 0

       self.currentSpeed = self.currentSpeed + self.current_acc()*1/5
       self.publisher.publisher(self.currentSpeed)

    def dist_filter(self,sep):                      
        self.dist_filter.previousDist.append(speed)
        if len(self.dist_filter.previousDist) > 3:  
            self.dist_filter.previousDist.pop(0)
        return sum(self.dist_filter.previousDist) / float(len(self.dist_filter.previousDist))
    
    def speed_filter(self,speed):                      
        self.speed_filter.previousSpeed.append(speed)
        if len(self.speed_filter.previousSpeed) > 5:  # keep only 5 values
            self.speed_filter.previousSpeed.pop(0)
        return sum(self.speed_filter.previousSpeed) / float(len(self.speed_filter.previousSpeed))
    
    def Command(self):
       if self.currentSpeed < Vmax:  
         err = -Dmin + self.frontDistance
         cmd_acc = Kp * err + Kd * self.Derivative(self.Vel)
       else:
         cmd_acc=0.0

       flag = -1
       if cmd_acc > 0:
        flag = 1
       p3_acc=min(Amax,cmd_acc)/Amax_T
       p3_dec=-1*max(Dmax,cmd_acc)/Dmax_T
       cmd ={ 1:[p3_acc,0,0,"Accelerating"], -1:[0,p3_dec,0,"Decelerating"]}
       self.driver.setThrottle(cmd[flag][0])
       self.driver.setBrakeIntensity(-1*cmd[flag][1])
       current_acc = ( cmd[flag][0]) + (cmd[flag][1])
       return current_acc
       
    def Derivative(self,Err):
        derivative = (Err[1] - Err[0]) * 32
        return derivative
    
pioneer = ros_nav()
pioneer.ACC()

