
import osmnx as ox
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math

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
    def __init__(self,RefLat,RefLon):
        [self.localX, self.LocalY, self.LocalZ] = llc([RefLat, RefLon])

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
        Cartesian =  [[llc(a)[0] - self.localX, llc(a)[1] - self.LocalY, llc(a)[2] - self.LocalZ] for a in waypoints]
        return Cartesian
    
    def PlotPath(Self,Waypoints):
        fig, ax = plt.subplots()
        ax.plot(Waypoints[:,0],Waypoints[:,1], label='Line 1', marker='o',
        linestyle='--')
        ax.set_ylabel('Longitude')
        ax.set_xlabel('Latitude')
        plt.show()


AutoDrive = OSMnav



