
import osmnx as ox
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2 as cv2
from circle_fit import taubinSVD
from matplotlib.pyplot import figure
from scipy.interpolate import UnivariateSpline
import matplotlib.animation as animation
import numpy as np
from scipy.special import comb

coords = []

pause = False

def bernstein_poly(i, n, t):
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def simData():
    t_max = 10.0
    dt = 0.05
    x = 0.0
    t = 0.0
    while t < t_max:
        if not pause:
            x = np.sin(np.pi*t)
            t = t + dt
        yield x, t

def onClick(event):
    global pause
    pause ^= True

#
def llc(a):
    latitude = a[0]
    longitude = a[1]
    radius = 6378137 
    r_lat = math.radians(latitude)
    r_lon = math.radians(longitude)
    x = radius * math.cos(r_lat) * math.cos(r_lon)
    y = radius * math.cos(r_lat) * math.sin(r_lon)
    z = radius * math.sin(r_lat)
    k = 1
    l=1.5
    return [k*x, l*y, z]


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
      # optionally draw on pre-existing axis
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
     
     
     ax.scatter(self.v[1],self.v[0],  c='red')
     fig.savefig('plot.png')
     plt.close()

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

    def show_graph(self,curr):
     fig,ax = ox.plot_graph_route(
     self.graph,self.route,
      # optionally draw on pre-existing axis
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
     
     
     ax.scatter(curr[1],curr[0],  c='red')
     plt.close()
     plt.draw()
     plt.pause(0.03)
     
     

    def save_path(self):
     fig,ax = plt.subplots()
     ax.plot(self.path[:,0],self.path[:,1],'--o',c='red')
     ax.plot(self.curr_state[0],self.curr_state[1],'--o',c='blue')
     #plt.savefig()
     plt.close()
     fig.savefig('plot1.png')

          
    def plotDubins(self,a,curr_state):
     figure,axes = plt.subplots(1,2)
     self.v=curr_state
     self.save_graph()
     image = plt.imread('/home/navin/webots/resources/osm_importer/OSMmod/plot.png')

     """ for i in range(len(coords)):
        Drawing_uncolored_circle = plt.Circle((coords[i][0],coords[i][1]),coords[i][2], fill = False )
        axes[0].set_aspect( 1 )
        axes[0].add_artist( Drawing_uncolored_circle ) """
     #img = cv2.imread("/home/navin/webots/resources/osm_importer/OSMmod/map.jpg")
     axes[1].imshow(image)
     self.path = a 
     axes[0].plot(a[:,0],a[:,1],'--o',c='red')
     self.curr_state = llc(curr_state)
     axes[0].plot(self.curr_state[0],self.curr_state[1],'--o',c='blue')
     self.save_graph()
     plt.title("Route")
     plt.show()

    

    def bezier_curve(self,points, nTimes=1000):
    
        nPoints = len(points)
        xPoints = points[:,0]
        yPoints = points[:,1]
        t = np.linspace(0.0, 1.0, nTimes)
        polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])
        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)
        for i in range(nTimes):
           coords.append([xvals[i],yvals[i]])
        return coords
    
    def update(self,curr_state):
       #self.figure,axes = plt.subplots(1,2)
       self.v = curr_state
       self.curr_state =  llc(curr_state)
       
       self.save_path()

    
       
        



"""     def animate(self):
        ani = animation.FuncAnimation(self.figure, self.update, interval=1000)
        plt.show() """

       
       

    
      
      




