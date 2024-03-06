
import osmnx as ox
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2 as cv2
from circle_fit import taubinSVD
from matplotlib.pyplot import figure
from scipy.interpolate import UnivariateSpline

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
        self.route = route
        middle_coords =((self.start_coords[0]+self.end_coords[0])/2,(self.start_coords[1]+self.end_coords[1])/2)
        self.graph = ox.graph_from_point(middle_coords, network_type='all_private', dist=1000)
        #fig, ax = ox.plot_graph_route(self.graph, route, node_size=1)
        waypoints = [(self.graph.nodes[node]['y'], self.graph.nodes[node]['x']) for node in route]
        self.way_points_list = np.array([list(x) for x in waypoints ])
        return self.way_points_list
    
    def getCartesian(self,waypoints):
        Cartesian =  [[llc(a)[0], llc(a)[1], llc(a)[2]] for a in waypoints]
        return Cartesian
    
    

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

    def save_graph(self):
     fig,ax = ox.plot_graph_route(
     self.graph,self.route,
    ax=None,  # optionally draw on pre-existing axis
    figsize=(8, 8),  # figure size to create if ax is None
    bgcolor="w",  # background color of the plot
    node_color="b",  # color of the nodes
    node_size=15,  # size of the nodes: if 0, skip plotting them
    node_alpha=None,  # opacity of the nodes
    node_edgecolor="none",  # color of the nodes' markers' borders
    node_zorder=1,  # zorder to plot nodes: edges are always 1
    edge_color="#111111",  # color of the edges
    edge_linewidth=1,  # width of the edges: if 0, skip plotting them
    edge_alpha=None,  # opacity of the edges
    show=False,  # if True, call pyplot.show() to show the figure
    close=False,  # if True, call pyplot.close() to close the figure
    save=False,  # if True, save figure to disk at filepath
    filepath=None,  # if save is True, the path to the file
    dpi=300,  # if save is True, the resolution of saved file
    bbox=None,  # bounding box to constrain plot
)
     fig.savefig('plot.png')
       
    def plotDubins(self,a,curr_state):
     figure,axes = plt.subplots(1,2)
     image = plt.imread('/home/navin/webots/resources/osm_importer/OSMmod/plot.png')

     for i in range(len(coords)):
        Drawing_uncolored_circle = plt.Circle((coords[i][0],coords[i][1]),coords[i][2], fill = False )
        axes[0].set_aspect( 1 )
        axes[0].add_artist( Drawing_uncolored_circle )
     #img = cv2.imread("/home/navin/webots/resources/osm_importer/OSMmod/map.jpg")
     axes[1].imshow(image)
     """ print(image)
     height, width = image.shape[:2]
     print(height,width)
     aminx=a[:,0].min()
     aminy=a[:,1].min()
     amaxx=a[:,0].max()
     amaxy=a[:,1].max()
     c1 =[1055250.1493748755, 6124465.093442681]
     c4 = [1055185.821532067, 6124687.127161345]
     c2 = [1053397.9607934437, 6124777.412067398]
     c3 =[1053416.6352519023, 6124989.661678567]
     x_range = -1053397 + 1055250
     y_range =  6124989 - 6124465
     x, y, w, h =  1055250 - aminx,    6124465, amaxx - aminx, amaxy - aminy """
     """ x, y, w, h = x*width/x_range, y*height/y_range, w*width/x_range, h*height/y_range
     print(x,y,w,h)
     region_of_interest = image[int(y):int(y+h), int(x):int(x+w)]
     print(region_of_interest)
     cv2.imshow('hi',region_of_interest)
     cv2.waitKey(0) """    
     #axes.set_yticks([6124683.809464188, 6124693, 6124703, 6124713, 6124723, 61247333, 6124743, 6124753, 6124763, 6124773, 6124783,6124790.711675112])
     #axes.imshow(img, extent=[1053377, 1055250, 6124465, 6124790])
     axes[0].plot(a[:,0],a[:,1],'--o',c='red')
     axes[0].plot(curr_state[0],curr_state[1],'--o',c='blue')
     plt.title("Route")
     plt.show()

    

    def spline(self,path):
       x = path[:,0]
       y = path[:,1]
       minx, miny = x.max(), y.max()
       spl = UnivariateSpline(x, y)
       x_s = np.linspace(minx, miny, 100)
       plt.plot(x_s, spl(x_s), 'g', lw=3)
       

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




