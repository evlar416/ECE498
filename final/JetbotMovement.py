import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

JETBOT_LIN_VEL = -0.2    # parameter /jetbot/teleop_keyboard/max_linear_vel   (.63172 for real JetBot)
JETBOT_ANG_VEL = 3.0   # parameter /jetbot/teleop_keyboard/max_angular_vel  (12.5 for real JetBot)

#Need to use different coefficients for different surfaces
LIN_VEL_COEF = 1.739 # found by calibrating jetbot for library table tops
ANG_VEL_COEF = 0.0545 # found by calibrating jetbot


class JetbotMovement(Node):

    def __init__(self):
        self.twist = Twist()
        self.twist.linear.x = 0.0 # only this value will change
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0 # only this value will change
        
        # create publisher to jetbot motor control
        # based off teleop_keyboard.py
        rclpy.init()
        self.node = rclpy.create_node('move_jetbot', namespace='jetbot')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        print("Initialized jetbot movement node")
    
    
    def move(self, cmd:tuple):
        """
        Input: (dist, rot) rotates the jetbot, rot degrees, then moves the jetbot forward dist meters
        """
        
        dist = float(cmd[0])
        rot = float(cmd[1])

        drive_time = np.absolute(dist/JETBOT_LIN_VEL) * LIN_VEL_COEF
        rot_time = np.absolute(rot/JETBOT_ANG_VEL) * ANG_VEL_COEF

        if(rot > 0 ):
            self.twist.angular.z = JETBOT_ANG_VEL
        elif(rot < 0 ):
            self.twist.angular.z = -1*(JETBOT_ANG_VEL)
        
        self.pub.publish(self.twist)
        print("Rotating for ", rot_time, " seconds")
        time.sleep(rot_time)
        
        self.clear_twist()

        if(dist > 0 ):
            self.twist.linear.x = JETBOT_LIN_VEL
        
        self.pub.publish(self.twist)
        print("Driving for ", drive_time, " seconds")
        time.sleep(drive_time)
        
        self.clear_twist()

        return


    def clear_twist(self):
        """
        Resets all twist values to zero and publishes values to node
        """

        self.twist.linear.x = 0.0 # only this value will change
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0 # only this value will change
        
        self.pub.publish(self.twist)
        
        time.sleep(0.1)

        return
    
    
    def print_twist_stats(self):
        print("Lin stats: x* = %d, y = %d, z = %d",self.twist.linear.x, self.twist.linear.y, self.twist.linear.z)
        print("Ang stats: x = %d, y = %d, z* = %d",self.twist.angular.x, self.twist.angular.y, self.twist.angular.z)

    
    def test_movement(self):
        """
        Moves to 3 predetermined points to test jetbot movement
        """

        moves = [(0.2, 45),(0.2, -180),(0.5,360)]

        print("Testing movement...")

        for i in range(len(moves)):
            print("Moving to point #",str(i))
            self.move(moves[i])

        return

    