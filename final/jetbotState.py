import numpy as np

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
    # return angle and distance for motor controller
    return tuple((action_space.dist,action_space.phi))
    