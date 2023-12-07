
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import String


#from teleop_keyboard.py
JETBOT_LIN_VEL = -0.2    # parameter /jetbot/teleop_keyboard/max_linear_vel   (.63172 for real JetBot)
JETBOT_ANG_VEL = 1.0   # parameter /jetbot/teleop_keyboard/max_angular_vel  (12.5 for real JetBot)

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


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
        

    def move_to_point(self, next_point:tuple):
        """
        Converts the next cartesian coordinates to rotational, then moves the jetbot to that location
        """

        dist = math.sqrt((next_point[0]**2)+(next_point[1]**2))
        rot = math.atan(next_point[0]/next_point[1])

        drive_time = np.absolute(dist/JETBOT_LIN_VEL)
        rot_time = np.absolute(rot/JETBOT_ANG_VEL)

        if(rot > 0 ):
            self.twist.angular.z = JETBOT_ANG_VEL
        elif(rot < 0 ):
            self.twist.angular.z = -1*(JETBOT_ANG_VEL)
        
        self.print_twist_stats()
        self.pub.publish(self.twist)
        
        print("Sleeping for rotation, %d Seconds", rot_time)
        time.sleep(rot_time)
        print("Slept for forward movement")


        self.clear_twist()

        if(dist > 0 ):
            self.twist.linear.x = JETBOT_LIN_VEL
        
        self.print_twist_stats()
        self.pub.publish(self.twist)
        
        print("Sleeping for forward movement, %d Seconds", drive_time)
        time.sleep(drive_time)
        print("Slept for forward movement")

        self.clear_twist()
        

        return


    def clear_twist(self):
        """
        Resets all twist values to zero
        """

        self.twist.linear.x = 0.0 # only this value will change
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0 # only this value will change
        
        self.pub.publish(self.twist)

        return
    
    def print_twist_stats(self):
        print("Lin stats: x* = %d, y = %d, z = %d",self.twist.linear.x, self.twist.linear.y, self.twist.linear.z)
        print("Ang stats: x = %d, y = %d, z* = %d",self.twist.angular.x, self.twist.angular.y, self.twist.angular.z)

    
    def test_movement(self):
        """

        """

        moves = [(-0.05, 0.5),(1.0, 0.2),(-.5,1.0)]

        print("Testing movement...")

        for i in range(len(moves)):
            print("Moving to point #",str(i))
            self.move_to_point(moves[i])



        return



def main(args=None):
    

    node = JetbotMovement()

    node.test_movement()
    
    node.clear_twist()
    
    rclpy.spin(node.node)

    # Destroy the node explicitly

    # (optional - otherwise it will be done automatically

    # when the garbage collector destroys the node object)

    node.node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

        main()
