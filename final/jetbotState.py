import numpy as np
import rrtDijkstra as rrt

# define 3D state space
class state_space:
    def __init__(self,x,z,theta):
        self.x = x
        self.z = z
        self.theta = theta # use theta for jetbot movement as these verticies do not 
                            # need an accurate self.theta to move in correct direction?
        self.journeylen = 0
        self.halt = 0 # for collision avoidance -- TREAT FIRST JETBOT TO GET TO GOAL ALSO AS GOAL

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
    if not state_space.halt:
        # find angle to next_vertex
        ang = np.arctan2((next_vertex[1]-state_space.z),(next_vertex[0] - state_space.x))
        #print(" ANGLE : " + str(float(ang)))
        # adjust theta of jetbot assuming it matches this rotation --> maybe use an ideal/next angle var?
        state_space.theta = ang
        # update action_space 
        action_space.phi = ang - state_space.theta
        # find distance between points using pythagorean thm
        dist = np.sqrt((next_vertex[0] - state_space.x)**2 + (next_vertex[1]-state_space.z)**2)
        action_space.dist = dist
        # check variable updated by collision handler, move if necessary
        mv = action_space.movement()
        #state_space.x = state_space.x + mv[0]
        #state_space.z = state_space.z + mv[1]
        state_space.x = next_vertex[0]
        state_space.z = next_vertex[1]
        state_space.journeylen += 1
        # return angle and distance for motor controller
        return tuple((action_space.dist,action_space.phi))
    # case for no movement
    else:
        return(0,action_space.phi)

# call after each movement?
# check proximity and interference with other jetbot path - set flags?
def collision_check(jbs1, jbs2,radius):
    if rrt.distance((jbs1.x,jbs1.z),(jbs2.x,jbs2.z)) < radius:
        return True

# return vals
# 0 - clear
# 1 - jetbot1 in way of jetbot2
# 2 - jetbot2 in way of jetbot1

# see if jetbot in way of other jetbot path
def path_check(jbs1, jbs2, jbp1, jbp2, radius):
    # check if jetbot 2 is in way of jetbot 1
    for i in range(len(jbp1)-1):
        if i < jbs1.journeylen:
            continue
        line = rrt.Line(jbp1[i], jbp1[i+1])
        if rrt.isThruObstacle(line,(jbs2.x,jbs2.z), radius):
            if np.allclose(jbp1[i+1],jbp1[-1]):
                col = 0
            else:
                col = 2
    for i in range(len(jbp2)-1):
        if i < jbs2.journeylen:
            continue
        line = rrt.Line(jbp2[i], jbp2[i+1])
        if rrt.isThruObstacle(line,(jbs1.x,jbs1.z), radius):
            if np.allclose(jbp2[i+1],jbp2[-1]):
                col = 0
            else:
                col = 1
    else:
        col = 0
    return col
    
def dist_left(jbs1, jbs2, jbp1, jbp2):
    dist1 = 0
    dist2 = 0
    for i in range(len(jbp1)-1):
        if i < jbs1.journeylen:
            continue
        dist1 += rrt.distance(jbp1[i],jbp1[i+1])
    for i in range(len(jbp2)-1):
        if i < jbs1.journeylen:
            continue
        dist2 += rrt.distance(jbp2[i],jbp2[i+1]) 
    if dist1 < dist2: which = 1
    else: which = 2
    return which

def collision_handler(jbs1, jbs2, jbp1, jbp2,radius):
    if(collision_check(jbs1, jbs2, radius)):
        which = path_check(jbs1, jbs2, jbp1, jbp2,radius)
        if which != 0:
            #what to do - closer jetbot keeps moving
            if which == 2 :
                jbs2.halt = 1
                print("JETBOT2 HALTING: jetbot1 in path of jetbot2")
            else:
                jbs1.halt = 1
                print("JETBOT1 HALTING: jetbot2 in path of jetbot1")
        else:
            which = dist_left(jbs1, jbs2, jbp1, jbp2)
            if which == 1:
                # jetbot 1 closer
                jbs1.halt = 1
                print("JETBOT1 HALTING: jetbot 1 closer to goal")
            else:
                jbs2.halt = 1 
                print("JETBOT2 HALTING: jetbot 2 closer to goal")
    else:
        jbs1.halt = 0
        jbs2.halt = 0
