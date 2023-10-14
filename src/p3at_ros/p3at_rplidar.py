# usr/bin#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd

speed =  0
Vmax = 0.20          # m/s
Amax = 1.2           # ozone parameters
Dmax = 1.4           # ozone parameters
cmdAcc = 0
Ascale = 1/3        # scaling acc from ozone to p3
Dscale = 1/4        # scaling dec from ozone to p3
Dmin = 2            # minimum distance to maintain in a CTH environment 
Dact = 5
Kp = 2              # gain of proportional controller
Kd = 2.828          # gain of derivative controller


class ros_nav():

    def __init__(self):
        rospy.init_node('P3_AT', anonymous=True)    
        self.pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10)
        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.callback)
        self.dist_filter.previousDist = []
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

    def callback(self,data):
       self.frontDistance = (data.ranges[358] + data.ranges[359] + data.ranges[360] + data.ranges[361])/4
       self.frontDistance = self.dist_filter(self.frontDistance)
       
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
    
    def Command(self):
       if self.currentSpeed < Vmax:  
         err = -Dmin + self.frontDistance
         cmd_acc = Kp * err + Kd * self.Derivative(self.Vel)
       else:
         cmd_acc=0.0

       flag = -1
       if cmdAcc > 0:
        flag = 1
       p3_acc=min(Amax*Ascale,cmd_acc)
       p3_dec=-1*max(Dmax*Dscale,cmd_acc)
       cmd ={ 1:[p3_acc,0,0,"Accelerating"], -1:[0,p3_dec,0,"Decelerating"]}
       current_acc = ( cmd[flag][0]) + (cmd[flag][1])
       return current_acc
       
    def Derivative(self,Err):
        derivative = (Err[1] - Err[0]) * 32
        return derivative
    
pioneer = ros_nav()
