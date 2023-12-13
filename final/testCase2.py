import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import math
import numpy as np

####################################################################################################

# THE RRT ALGORITHM GIVEN BELOW IS CREDITED TO https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

# line class for RRT algorithm
class Line():
    
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        #self.dirn /= self.dist # normalize
        self.dirn = self.dirn / self.dist # normalize


    def path(self, t):
        return self.p + t * self.dirn
    
def Intersection(line, center, radius):
    ''' Check line-sphere (circle) intersection '''
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a);
    t2 = (-b - np.sqrt(discriminant)) / (2 * a);

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

# Function definitions for RRT algorithm

def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False

def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False

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

def window(startpos, endpos):
    ''' Define seach window - 2 times of start to end rectangle'''
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height


def isInWindow(pos, winx, winy, width, height):
    ''' Restrict new vertex insides search window'''
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height:
        return True
    else:
        return False



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
        rx = random.random()
        ry = random.random()

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

def smooth(path,obstacles,radius):
    newPath = []
    parent = path[0]
    prev = path[0]
    #newPath.append(parent)
    for i in path:
        line = Line(i,parent)
        if isThruObstacle(line,obstacles,radius):
            newPath.append(prev)
            parent = prev
            prev = i
        elif np.allclose(i,path[-1]):
            newPath.append(i)
            prev = i
        else:
            prev = i

    return newPath

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

    ax.autoscale()
    ax.margins(0.1)
    #plt.xlim(-10,10)
    #plt.ylim(-10,10)
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
        self.theta = theta

# define action space
class action_space:
    # initialize class variables to be calculated / set by transition function
    def __init__(self):
        self.theta = 0 # will be the incremental angle change (relative)
        self.dist = 0
        self.lin_velocity = 0.1 # meter per second
        self.ang_velocity = 1 # degree per second
        self.ang_time = 0
        self.lin_time = 0
    # determine timestep needed to orient jetbot given linear and angular velocity
    def movement(self):
        self.lin_time = np.absolute(self.dist / self.lin_velocity)
        self.ang_time = np.absolute(self.theta / self.ang_velocity)
        # print(((np.sin(self.theta)*self.dist),(np.sin(self.theta)*self.dist)))
        # return value below intended for simulation purposes
        str1 = str(float(np.cos(self.theta)*self.dist))
        str2 = str(float(np.sin(self.theta)*self.dist))
        print(str1 + "," + str2)
        return ((np.cos(self.theta)*self.dist),(np.sin(self.theta)*self.dist))
        
# movement needed to transition from current state to next state
# sets action_space to be called to move robot
def state_transition(state_space,action_space,next_vertex):
    # find angle to next_vertex
    # print(next_vertex[0],state_space.x)
    # use arctan or arctan2?
    ang = np.arctan2((next_vertex[1]-state_space.z),(next_vertex[0] - state_space.x))
    # update action_space
    action_space.theta = ang - state_space.theta
    print("ACTION ANGLE: " + str(float(action_space.theta)))
    #action_space.theta = ang - state_space.theta # % (2*math.pi) ?
    # adjust theta of jetbot
    state_space.theta = ang
    #angStr = str(float(action_space.theta))
    #print(angStr)
    # find distance between points using pythagorean thm
    #dist = np.sqrt((next_vertex[0] - state_space.x)**2 + (next_vertex[1]-state_space.z)**2)
    dist = np.linalg.norm((next_vertex[0] - state_space.x) - (next_vertex[1]-state_space.z))
    action_space.dist = dist
    print("DISTANCE: " + str(float(dist)))
    # update state_space assuming position becomes new position (will definitely introduce error)
    # should really update state space according to action_space.movement()
    state_space.x = next_vertex[0]
    state_space.z = next_vertex[1] 
    mv = action_space.movement()
    #state_space.x = state_space.x + mv[0]
    #state_space.z = state_space.z + mv[1]
    '''
    print ("STATE X: " + str(state_space.x) + "\n")
    print ("STATE Z: " + str(state_space.z) + "\n")
    print ("STATE T: " + str(state_space.theta) + "s\n")
    '''
    
################################################################################    
        
def main(args=None):
    
    # assume starting position of jetbot to be 0,0
    startpos = (0,0)

    ArucoMarkers = {}
    # define 5 total markers
    # NEED TO CONSIDER NEGATIVE COORDINATES!!!
    #for i in range(5):
    #    ArucoMarkers.update({i:(random.randint(-8,8),random.randint(-8,8))})

    ArucoMarkers.update({0:(1,1)})
    ArucoMarkers.update({1:(2,2)})
    ArucoMarkers.update({2:(3,3)})
    ArucoMarkers.update({3:(4,4)})
    ArucoMarkers.update({4:(5,5)})


    # choose arbitrary goal for now
    #goal = ArucoMarkers[random.randint(0,4)]
    goal_marker = random.randint(0,4)
    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][1])
    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][1]) # is this or [1] correct?
    #endpos=(-0.1166,0.5012) # is this or above correct?
    #endpos = (3,4)
    #print("END POS: ")
    #print(endpos)

    obstacles = []
    # these are exact comparisons, will need to be margin of error when comparing real coordinates (floats)
    for marker in ArucoMarkers.keys():
        if marker == goal_marker:
            continue
        else:
            # obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][1]))
            obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][1])) # which one is correct?
              
    print(obstacles)

    # define max iteration for RRT
    n_iter = 500
              
    # define radius for both reaching goal and avoiding obstacles??
    radius = 0.2 # 10 cm radius
              
    # define stepsize for forming new verticies
    stepSize = 0.3 # need to figure out stepsize for jetbot
              
    # call RRT and Dijkstra to find shortest path to object  
    G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        print(path)
        path = smooth(path,obstacles,radius)
        print(path)
        # simulate jetbot movement
        jetbot_state = state_space(0,0,math.pi/2)
        jetbot_action = action_space()
        #jetbot_path = [[0,0]]
        jetbot_path = [(0,0)]
        hist = (0,0)
        error = [] # use this eventually

        print("PATH LENGTH: ")
        print(len(path))
        for i in range(len(path)-1):
            state_transition(jetbot_state,jetbot_action,path[i+1]) # jetbot state is updated by this function
            #mv = jetbot_action.movement()
            #jetbot_path.append(((mv[0]+hist[0]),(mv[1]+hist[1])))
            #hist = mv
            jetbot_path.append((jetbot_state.x,jetbot_state.z))

        plot(G, obstacles, radius, jetbot_path)
        print("JETBOT PATH LENGTH: ")
        print(len(jetbot_path))
        print(jetbot_path)
    '''
        plot(G, obstacles, radius, path)
    else:
        plot(G, obstacles, radius)
    '''

if __name__ == '__main__':
        main()