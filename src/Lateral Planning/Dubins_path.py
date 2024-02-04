import OSMnav
from circle_fit import taubinSVD
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import interp1d

l = []

def dist(l,i):
    return np.linalg.norm(l[i,0:1]-l[i+1,0:1])

coords = []

def distcheck(l,i):
   if abs(dist(l,i+1)- dist(l,i)) < 30 :  
    return 1

def check_Circle(l,i):   
    temp = []
    count = 0
    global coords
    
    if i < len(l)-2:
     if abs(dist(l,i+1)- dist(l,i)) < 20 :
      
        temp.append(l[i,0:2])
        temp.append(l[i+1,0:2])
        temp.append(l[i+2,0:2])

        i+=1
        xc, yc, r, sigma = taubinSVD(temp)
        coords.append([xc,yc,r])
        check_Circle(l,i)

     else :
       i +=1
       check_Circle(l,i)
      
     """ elif dist(l,i) > 5 and abs(dist(l,i+1)- dist(l,i+2)) > 20:
        temp.append(l[i,0:2])
        temp.append(l[i+1,0:2])
        xc, yc, r = (l[i,0] + l[i+1,0])/2, (l[i,1] + l[i+1,1])/2, dist(l,i)/2 
        coords.append([xc,yc,r])
        i+=1
        check_Circle(l,i) """

     
    
    else:
       return coords



AutoDrive = OSMnav.OSMnav(12.991466, 80.233750)
AutoDrive.ResetCoords(12.992987, 80.229860                  ,12.984205, 80.234767)



k = AutoDrive.ShortestPath()
l = np.asarray(AutoDrive.getCartesian(k))
point_coordinates = [[1, 0], [-1, 0], [0, 1], [0, -1]]
xc, yc, r, sigma = taubinSVD(point_coordinates)
check_Circle(l,1)
print(coords)
figure, axes = plt.subplots()
for i in range(len(coords)):
   Drawing_uncolored_circle = plt.Circle((coords[i][0],coords[i][1]),coords[i][2], fill = False )
   axes.set_aspect( 1 )
   axes.add_artist( Drawing_uncolored_circle )
plt.plot(l[:,0],l[:,1],'--o')
plt.show()




