import numpy as np
import math


class LatControl():

    def __init__(self,path):
        inflation_rad=10
        #new_x = np.linspace(min(path[:,0]), max(path[:,0]), num=300)
        #new_y = np.interp(new_x, path[:,0], path[:,1])
        self.sign=[0,-1]
        self.path =  path
        self.prev_path  = 0
        self.i=1

    def cross_track_error(self,curr_pos):
        m = (self.path[self.i -1,1] - self.path[self.i,1])/(self.path[self.i -1,0] - self.path[self.i,0]) 
        self.m=m
        c = self.path[0,1] - m*self.path[0,0]
        self.c=c
        e = (curr_pos[1] - m*curr_pos[0] - c )/ math.sqrt(1+m**2)
        e += + -1*e*3/abs(e)
        return e
    
    def getSign(self):
        k = (self.path[self.i +1,1] - self.m*self.path[self.i +1,0] - self.c )/ math.sqrt(1+self.m**2)
        return k/abs(k)
    
    def heading_error(self,curr_pos,yaw):     
        self.m = (self.path[self.i -1,1] - self.path[self.i,1])/(self.path[self.i -1,0] - self.path[self.i,0]) 
        self.path_angle = math.atan(self.m)
        if self.path_angle < 0:
            self.path_angle = math.pi - self.path_angle
        
        if self.sign[self.i] > 0:
            self.path_angle = math.pi - self.path_angle
            #print("path",self.path_angle)
    
        if abs(self.path_angle) < 0.2:
            if yaw < 0:
             print("case 1a")
             yaw = yaw + math.pi
             #print("yaw1",yaw)
             e = -self.path_angle + yaw
            else:
             yaw = yaw - math.pi
             print("case 1b")
             #print("yaw1",yaw)
             e = -self.path_angle + yaw
        elif abs(math.pi - self.path_angle) < 0.2:   
            yaw = yaw + math.pi
            print("case 2")
            e = -self.path_angle +yaw
        else:
            print("case 3")
            if yaw < 0:
                yaw = math.pi + yaw
            else:
                yaw = yaw - math.pi
            e = self.path_angle - yaw

        print("path",self.path_angle)
        print("heading correction",e)      
        return e
    
    def steer_control(self,curr_pos,yaw,speed):
        K = 1.5
        self.checkPoints(curr_pos)
        print("chasing point",self.i)
        C_err = self.cross_track_error(curr_pos)
        H_err = self.heading_error(curr_pos,yaw)
        print("cross track correction",C_err)
        command =   H_err/4 + math.atan(K*C_err/speed)/3
        return command, self.i

    def checkPoints(self,curr_pos):
        if np.linalg.norm(self.path[self.i,0:1]-curr_pos[0:1]) < 1.5:
            self.sign.append(self.getSign())
            
            self.i+=1
            self.prev_m = self.m
        else:
            pass
        return self.i
