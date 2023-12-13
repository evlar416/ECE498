import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from jetbot_ros.motors import MotorController
from aruco_opencv_msgs.msg import ArucoDetection
import numpy as np
#from scipy.spatial.transform import Rotation as R
import scipy
import time

import random
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from collections import deque

####################################################################################################

# THE RRT ALGORITHM GIVEN BELOW IS CREDITED TO https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

# line class for RRT algorithm
class Line():
    
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

# Function definitions for RRT algorithm

def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex


# Graph class for RRT algorithm
class Graph:
    
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        return posx, posy

# RRT algorithm for finding path to goal
def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):

    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            #print('success')
            # break
    return G

# once graph G is constructed with RRT, find shortest path with Dijkstra
def dijkstra(G):

    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)

def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=2)
        ax.add_collection(lc2)

    # ax.autoscale()
    ax.margins(0.1)
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.show()

################################################################################
# need to find initial theta, then update based on next target

# function for coordinate transforms
# want to fix origin on goal
# want to find translation vector for jetbot with respect to bottle

def coord_transform(pos):
    #xa = [posA.x,posA.z]
    tb = [pos.x,pos.z]
    rotation_matrix = [[np.cos(pos.theta),-(np.sin(pos.theta))],
                       [np.sin(pos.theta),np.cos(pos.theta)]]
    #xb = np.matmul(np.transpose(rotation_matrix),xa) - np.matmul(np.transpose(rotation_matrix),tb)
    ta = -(np.matmul(np.transpose(rotation_matrix),tb))

    return ta
    

# define 3D state space
class state_space:
    def __init__(self,x,z,theta):
        self.x = x
        self.z = z
        self.theta = theta # use theta for jetbot movement as these verticies do not 
                            # need an accurate self.theta to move in correct direction

# define action space
class action_space:
    # initialize class variables to be calculated / set by transition function
    def __init__(self):
        self.phi = 0 # will be the incremental angle change (relative)
        self.dist = 0
        self.lin_velocity = 0.1 # meter per second
        self.ang_velocity = 1 # degree per second
        self.ang_time = 0
        self.lin_time = 0
    # determine timestep needed to orient jetbot given linear and angular velocity
    def movement(self):
        #self.lin_time = np.absolute(self.dist / self.lin_velocity)
        #self.ang_time = np.absolute(self.phi / self.ang_velocity)
        return ((np.cos(self.phi)*self.dist),(np.sin(self.phi)*self.dist))
        
# movement needed to transition from current state to next state
# sets action_space to be called to move robot
def state_transition(state_space,action_space,next_vertex):
    # find angle to next_vertex
    # print(next_vertex[0],state_space.x)
    # use arctan or arctan2?
    ang = np.arctan2((next_vertex[1]-state_space.z),(next_vertex[0] - state_space.x))
    # adjust theta of jetbot assuming it matches this rotation --> maybe use an ideal/next angle var?
    state_space.theta += ang #valid with atan2 always returning quandrant?
    # update action_space 
    action_space.phi = ang
    # find distance between points using pythagorean thm
    dist = np.sqrt((next_vertex[0] - state_space.x)**2 + (next_vertex[1]-state_space.z)**2)
    action_space.dist = dist
    mv = action_space.movement()
    state_space.x = state_space.x + mv[0]
    state_space.z = state_space.z + mv[1]
    
################################################################################    

# Read positions of aruco markers- in this implementation these values will only be 
# viewed once (before movement) to determine path to goal
class ArucoSubscriber(Node):
    
    def __init__(self):
        super().__init__('aruco_subscriber') # this name will show in node list
        
        # create subscriber to aruco_detection messages
        self.sub = self.create_subscription(
                ArucoDetection, # type of message
                'aruco_detections', # topic to subscribe to
                self.listener_callback, # called every time node receives message
                10)
        
        # create dictionary of markers to be populated by messages in "x,z,theta" format
        self.markerOrientation = {}
        self.markerPosition = {}
        
    # Build/update dictionary of coordinates for each marker every time new message arrives
    def listener_callback(self,aruco_msg):
        for marker in aruco_msg.markers:
            id = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            self.markerPosition.update({id:[x,y,z]})
        
################################################################################    

def see_aruco_tag():
    # create node
    aruco_subscriber = ArucoSubscriber()
    # define markers
    markers = aruco_subscriber.markers
    # assume starting position of jetbot to be 0,0
    startpos = (0,0)
    # choose arbitrary goal 0
    goal_id = 0
    endpos=(markers.markerPosition[goal_id][0],markers[goal_id][2]) # is this or [goal_id][1] correct?

    obstacles = []
    for iter in markers:
        if iter.id == goal_id:
            continue
        else:
            obstacles.append((iter.markerPosition[iter.id][0],iter.markerPosition[iter.id][2])) # which one is correct? ([1] or [2])
    return endpos,obstacles

def path_plan(startpos, endpos, obstacles, n_iter, radius, stepSize, plot):
    G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        if(plot):
            plot(G, obstacles, radius, path)
    else:
        path = -1
        if(plot):
            plot(G, obstacles, radius)
    return path

def jetbot_sim(path):
    jetbot_state = state_space(0,0,0)
    jetbot_action = action_space()
    jetbot_path = [[0,0]]
    for i in range(len(path)-1):
        state_transition(jetbot_state,jetbot_action,path[i+1])
        jetbot_path.append((jetbot_state.x,jetbot_state.z))
    return jetbot_path

def main(args=None):


if __name__ == '__main__':
    main()    