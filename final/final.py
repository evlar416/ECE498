import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from jetbot_ros.motors import MotorController
from aruco_opencv_msgs.msg import ArucoDetection
import numpy as np
#from scipy.spatial.transform import Rotation as R
import scipy
from random import random
from collections import deque
import time    
import lqr_rrt_star




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
        self.theta = 0
        self.dist = 0
        self.lin_velocity = 0.1 # meter per second
        self.ang_velocity = 1 # degree per second
        self.ang_time = 0
        self.lin_time = 0
    # determine timestep needed to orient jetbot given linear and angular velocity
    def movement(self):
        self.lin_time = np.absolute(self.distance / self.lin_velocity)
        self.ang_time = np.absolute(self.theta / self.ang_velocity)



# movement needed to transition from current state to next state
# sets action_space to be called to move robot
def state_transition(state_space,action_space,next_vertex):
    # find angle to next_vertex
    ang = np.arctan((next_vertex[1]-next_vertex[0])/(state_space.z - state_space.x))
    # adjust theta of jetbot
    state_space.theta = state_space.theta + ang
    # update action_space
    action_space.theta = ang
    # find distance between points using pythagorean thm
    dist = np.sqrt((state_space.z - state_space.x)**2 - (next_vertex[1]-next_vertex[0])**2)
    action_space.dist = dist
    # update state_space assuming position becomes new position
    state_space.x = next_vertex[0]
    state_space.z = next_vertex[1]


class ArucoSubscriber(Node):
    
    def __init__(self):
        super().__init__('aruco_subscriber') # this name will show in node list
        
        # create subscriber to aruco_detection messages
        self.sub = self.create_subscription(
                ArucoDetection, # type of message
                'aruco_detections', # topic to subscribe to
                self.listener_callback, # called every time node receives message
                10)
        
        # create publisher to jetbot motor control
        # based off teleop_keyboard.py
        self.pub = self.create_publisher(
                Twist, 
                'cmd_vel', 
                10) 
        
        # create dictionary of markers to be populated by messages in "x,z,theta" format
        self.markers = {}
        
    # Build/update dictionary of coordinates for each marker every time new message arrives
    def listener_callback(self,aruco_msg):
        # This only works for using sequential markers (ID = 0,1,2,3, etc) // will fix, done this
        # way for testing
        for i in range(len(aruco_msg.markers)):
            x = aruco_msg.markers[i].pose.orientation.x
            y = aruco_msg.markers[i].pose.orientation.y
            z = aruco_msg.markers[i].pose.orientation.z
            w = aruco_msg.markers[i].pose.orientation.w
            r = scipy.spatial.transform.Rotation.from_quat([x,y,z,w])
            # convert to rotation vector "x,z,theta"
            r = r.as_rotvec()
            # update position of marker in dictionary
            self.markers.update({i:[r[0],r[1],r[2]]})
            #self.markers[i] = [r[0],r[1],r[2]]
        
def main(args=None):
    rclpy.init(args=args)
    
    aruco_subscriber = ArucoSubscriber() # create node
    
    # choose goal to be marker ID 0, find coordinates for endpos
    goal = 0

    # assume starting position of jetbot to be 0,0
    startpos = [0.0, 0.0]
    
    endpos=[aruco_subscriber.markers[goal][0],aruco_subscriber.markers[goal][1]]
    
    # establish all obstacles (other identified markers)
    obstacles = [(0,0,0) for i in range(len(aruco_subscriber.markers))]

    for marker,coordinate in aruco_subscriber.markers.items():
        if marker == goal:
            continue
        else:
            obstacles[marker] = (coordinate[0],coordinate[1],1)

    
    lqr_rrt_star = LQRRRTStar(startpos, endpos,
                              obstacleList,
                              [-2.0, 15.0])

    path = lqr_rrt_star.planning(animation=show_animation)

    # Draw final path
    if show_animation:  # pragma: no cover
        lqr_rrt_star.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()

    print("Done")

    # find position of jetbot relative to goal (x,z,theta form)
    goal_state = state_space(aruco_subscriber.markers[goal][0],
                             aruco_subscriber.markers[goal][1],
                             aruco_subscriber.markers[goal][2])
    
    # translation vector is given as jetbot initial position (x,z)                                                 
    t_jetbot = coord_transform(goal_state[0],goal_state[1],goal_state[2])
    # same angle between jetbot and goal as angle between goal and jetbot
    jetbot_state = state_space(t_jetbot[0],t_jetbot[1],goal_state[2])
    
    #create twist object
    twist = Twist()
    
    # initialize action space
    jetbot_action = action_space()

    # use path (list of verticies) to inform next state of jetbot          
    # iterate through each vertex on path as jetbot travels to it
    for i in range(len(path)):
        state_transition(jetbot_state,jetbot_action,path[i]) # jetbot state is updated by this function
        # calculate timesteps for linear and angular movement
        jetbot_action.movement()
              
        # jetbot linear movement
              
        twist.linear.x = float(jetbot_action.lin_velocity)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # move until timestep is elapsed
        end_time = time.clock_gettime_ns(time.CLOCK_PROF) + jetbot_action.lin_time*1000000000
        while(end_time > time.clock_gettime_ns(time.CLOCK_PROF)):
            pub.publish(twist)
        
        # jetbot angular movement
        
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(jetbot_action.ang_velocity)
        
        end_time = time.clock_gettime_ns(time.CLOCK_PROF) + jetbot_action.lin_time*1000000000
        while(end_time > time.clock_gettime_ns(time.CLOCK_PROF)):
            pub.publish(twist)
              
    # cease jetbot movement once goal is reached
    
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
        
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    
    pub.publish(twist)
              
if __name__ == '__main__':
        main()