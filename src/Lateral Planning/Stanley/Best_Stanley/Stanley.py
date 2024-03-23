import numpy as np
import math
from scipy.optimize import minimize


class LatControl():

    def __init__(self,path):
        self.sign=[0,-1]
        self.path =  path
        self.prev_path  = 0
        self.i=1
        self.m =0
        self.c =0
        self.k =1

    def cross_track_error(self,curr_pos):
        m = (self.path[self.i -1,1] - self.path[self.i,1])/(self.path[self.i -1,0] - self.path[self.i,0]) 
        self.m=m
        c = self.path[self.i-1,1] - m*self.path[self.i-1,0]
        self.c=c
        e = (curr_pos[1] - m*curr_pos[0] - c )/ math.sqrt(1+m**2)
       
        return self.k*e
    
    def path_correct(self,hvecx,hvecy):
        k = hvecx/hvecy
        if k > 0:
             if hvecx > 0:
                  print(1)
                  path_angle = math.atan(k)
             else:
                  print(2)
                  path_angle = math.pi + math.atan(k)
        else:
             if hvecx > 0:
                  print(3)
                  path_angle = 2*math.pi+math.atan(k)
             else:
                  print(4)
                  path_angle = math.pi + math.atan(k)
        return path_angle

    def heading_error(self,curr_pos,yaw):
        h_vecx = self.path[self.i,0] - self.path[self.i-1,0]
        h_vecy = self.path[self.i,1] - self.path[self.i-1,1]
        k = h_vecy/h_vecx
        if k > 0:
             if h_vecx > 0:
                  print(1)
                  self.k=1
                  self.path_angle = math.atan(k)
             else:
                  print(2)
                  self.k =-1
                  self.path_angle = math.pi + math.atan(k)
        else:
             if h_vecx > 0:
                  print(3)
                  self.k=1
                  self.path_angle = 2*math.pi+math.atan(k)
             else:
                  print(4)
                  self.k=-1
                  self.path_angle = math.pi + math.atan(k)
        
        self.path_angle += 0.08
        if self.path_angle > 6.1:
            self.path_angle =  2*math.pi - self.path_angle
        yaw = yaw + math.pi
        self.yaw = yaw
        e = self.path_angle - yaw
        print("path",self.path_angle)
        print("yaw",yaw)
        print("heading correction",e)

        if self.path_angle < 0.3:
            if yaw < 3.14:
                return -e
            else:
                return e/7
        else:
            return -e

    def steer_control(self,curr_pos,yaw,speed):
        K = 2.8
        self.pos_x = curr_pos[0]
        self.pos_y = curr_pos[1]
        self.obs = np.asarray([1,2])
        self.checkPoints(curr_pos)
        print("chasing point",self.i)
        C_err = self.cross_track_error(curr_pos)
        H_err = self.heading_error(curr_pos,yaw)
        print("cross track error",C_err)
        print("cross track correction",math.atan(K*C_err/speed)/3)
        print("Heading correction",H_err/3)
        
        command =   H_err/2.7 + math.atan(K*C_err/speed)/4
        return command, self.i

    def checkPoints(self,curr_pos):
        if np.linalg.norm(self.path[self.i,0:1]-curr_pos[0:1]) < 1:
            self.i+=1
            self.prev_m = self.m
        else:
            pass
        return self.i
    
    def checkObstacle(self,curr_pos, obs):
        if np.linalg.norm(self.curr_pos-obs) < 10:
            return True
        else:
            return False
        

    
   
