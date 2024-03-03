
import osmnx as ox
import numpy as np
import matplotlib.pyplot as plt
import math
from circle_fit import taubinSVD
from matplotlib.pyplot import figure

coords = []

def llc(a):
    latitude = a[0]
    longitude = a[1]
    radius = 6378000
    r_lat = math.radians(latitude)
    r_lon = math.radians(longitude)
    x = radius * math.cos(r_lat) * math.cos(r_lon)
    y = radius * math.cos(r_lat) * math.sin(r_lon)
    z = radius * math.sin(r_lat)
    
    return [x, y, z]


class OSMnav():

    def ResetCoords(self,sla,slo,ela,elo):
        self.startLat = sla
        self.startLon = slo
        self.endLat = ela
        self.endLon = elo
        self.start_coords = (self.startLat, self.startLon)
        self.end_coords = (self.endLat, self.endLon)
        self.middle_coords = ((self.start_coords[0]+self.end_coords[0])/2,(self.start_coords[1]+self.end_coords[1])/2)
        self.graph = ox.graph_from_point(self.middle_coords, network_type='all_private', dist=1000)

    def Plot(self):    
        fig, ax = ox.plot_graph(self.graph)

    def ShortestPath(self):
        start_node = ox.distance.nearest_nodes(self.graph,X=self.start_coords[1],Y=self.start_coords[0])
        end_node = ox.distance.nearest_nodes(self.graph,X=self.end_coords[1],Y=self.end_coords[0])
        route = ox.shortest_path(self.graph, start_node, end_node, weight='length')
        #fig, ax = ox.plot_graph_route(self.graph, route, node_size=1)
        waypoints = [(self.graph.nodes[node]['y'], self.graph.nodes[node]['x']) for node in route]
        self.way_points_list = np.array([list(x) for x in waypoints ])
        return self.way_points_list
    
    def getCartesian(self,waypoints):
        Cartesian =  [[llc(a)[0], llc(a)[1], llc(a)[2]] for a in waypoints]
        return Cartesian
    
    
    def PlotPath(self,Waypoints):
        fig, ax = plt.subplots()
        ax.plot(Waypoints[:,0],Waypoints[:,1], label='Line 1', marker='o',
        linestyle='--')
        ax.set_ylabel('Longitude')
        ax.set_xlabel('Latitude')
        plt.show()

    def dist(self,l,i):
        return np.linalg.norm(l[i,0:1]-l[i+1,0:1])

    def distcheck(self,l,i):
        if abs(self.dist(l,i+1)- self.dist(l,i)) < 30 :  
            return 1
   
    def Dubin(self,l,i):   
        temp = []
        count = 0
        global coords
    
        if i < len(l)-2:
         if abs(self.dist(l,i+1)- self.dist(l,i)) < 20 :
      
            temp.append(l[i,0:2])
            temp.append(l[i+1,0:2])
            temp.append(l[i+2,0:2])

            i+=1
            xc, yc, r, sigma = taubinSVD(temp)
            coords.append([xc,yc,r])
            self.Dubin(l,i)

         else :
            i +=1
            self.Dubin(l,i)
      

        else:
            return coords
        
    def plotDubins(self,a):
     figure, axes = plt.subplots()
     plt.figure(figsize=(8, 6))

    
     for i in range(len(coords)):
        Drawing_uncolored_circle = plt.Circle((coords[i][0],coords[i][1]),coords[i][2], fill = False )
        axes.set_aspect( 1 )
        axes.add_artist( Drawing_uncolored_circle )


     
     #img = plt.imread("map.png")
     #plt.imshow(img)
     axes.plot(a[:,0],a[:,1],'--o')
     #axes.plot(1055187.6864792807,6124678.785627263,'o')
     #axes.plot(1053374.6155685824,6124783.73812779,'o')
     plt.show()

    def plotCar(self,a,v):
      figure, axes = plt.subplots()
      
      for i in range(len(coords)):
        Drawing_uncolored_circle = plt.Circle((coords[i][0],coords[i][1]),coords[i][2], fill = False )
        axes.set_aspect( 1 )
        axes.add_artist( Drawing_uncolored_circle )
      
      plt.plot(a[:,0],a[:,1],'--o')
      #plt.plot(v[0],v[1],'o')
      plt.show(block=False)
      plt.pause(0.1)
      plt.close()





