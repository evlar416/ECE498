import numpy as np
import math

# define 3D state space
class state_space:
    def __init__(self,x,z,theta):
        self.x = x
        self.z = z
        self.theta = theta # use theta for jetbot movement as these verticies do not 
                            # need an accurate self.theta to move in correct direction
        
    # movement needed to transition from current state to next state
    # sets action_space to be called to move robot
    def state_transition(self, next_vertex):

        # find angle to next_vertex
        
        ang = math.atan2((next_vertex[1]-self.z),(next_vertex[0] - self.x))

        # update next rotation: rot and state space
        rot = math.degrees(ang - self.theta)
        print("current angle, point global ang, and next rot", math.degrees(self.theta), rot, math.degrees(ang))
        self.theta = ang

        # find distance between points using pythagorean thm
        dist = np.sqrt((next_vertex[0] - self.x)**2 + (next_vertex[1]-self.z)**2)
        
        self.x = next_vertex[0]
        self.z = next_vertex[1]
        
        # return angle and distance for motor controller
        return tuple((dist, rot))
