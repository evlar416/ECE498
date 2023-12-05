
import math
import time
import numpy as np

from geometry_msgs.msg import Twist


#from teleop_keyboard.py
JETBOT_LIN_VEL = 0.2    # parameter /jetbot/teleop_keyboard/max_linear_vel   (.63172 for real JetBot)
JETBOT_ANG_VEL = 2.0    # parameter /jetbot/teleop_keyboard/max_angular_vel  (12.5 for real JetBot)

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class JetbotMovement:


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
        self.pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)
        

    def move_to_point(next_point:tuple):
        """
        Converts the next cartesian coordinates to rotational, then moves the jetbot to that location
        """

        dist = math.sqrt((next_point[0]**2)+(next_point[1]**2))
        rot = math.arctan(next_point[0],next_point[1])

        drive_time = np.absolute(dist/JETBOT_LIN_VEL)
        rot_time = np.absolute(rot/JETBOT_ANG_VEL)

        if(rot > 0 ):
            self.twist.angular.z = JETBOT_ANG_VEL
        elif(rot < 0 ):
            self.twist.angular.z = -1*(JETBOT_ANG_VEL)

        self.pub.publish(self.twist)

        time.sleep(rot_time)

        self.clear_twist

        if(dist > 0 ):
            self.twist.linear.x = JETBOT_LIN_VEL

        self.pub.publish(self.twist)

        time.sleep(drive_time)

        self.clear_twist
        self.pub.publish(self.twist)

        return


    def clear_twist():
        """
        Resets all twist values to zero
        """

        self.twist.linear.x = 0.0 # only this value will change
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0 # only this value will change

        return




