import rclpy

from rclpy.node import Node

from std_msgs.msg import String

from aruco_opencv_msgs.msg import ArucoDetection

from geometry_msgs.msg import Twist

import math

#import matplotlib.pyplot as plt

GOAL_INDEX = 1


def truncate_float(float_number, decimal_places):
    multiplier = 10 ** decimal_places
    return int(float_number * multiplier) / multiplier

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('minimal_publisher')

        #self.publisher_ = self.create_publisher(String, 'topic', 10)

        #timer_period = 0.5  # seconds

        #self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

        self.sub = self.create_subscription(
            ArucoDetection,
            'aruco_detections',
            self.listener_callback,
            10)


        #self.goal = self.sub.ArucoDetection.markers[1]
        #self.goalPos = (truncate_float(self.goal.pose.position.x,3), truncate_float(self.goal.pose.position.z,3))
        #self.origin = (0,0)
        self.obstacles = {}
        self.obstaclesXPos = []
        self.obstaclesZPos = []

#       self.getObstacles(self)

#       print("Goal pos = ", goalPos)
#       for i in self.obstaclesXPos[i]:
#             print("Obstacle ",i," pos = ", self.obstaclesXPos[i],",",self.obstaclesZPos[i])


        #self.pub = self.create_publisher(Twist, 'jetbot/cmd_vel', 10)

            
    def listener_callback(self, aruco_msg):
        
        x0 = aruco_msg.markers[0].pose.position.x
        z0 = aruco_msg.markers[0].pose.position.z
        theta0 = aruco_msg.markers[0].pose.orientation.z
        print("Marker 0: x = ", x0, " | z = ", z0, " | theta = ", theta0)
        
        x1 = aruco_msg.markers[1].pose.position.x
        z1 = aruco_msg.markers[1].pose.position.z
        theta1 = aruco_msg.markers[1].pose.orientation.z
        print("Marker 1: x = ", x1, " | z = ", z1, " | theta = ", theta1)
        
        
    def getObstacles(self, aruco_msg):
        
        for i in aruco_msg.markers[i]:
            if(i == 1):
                continue
            else:
                self.obstacles[i] = aruco_msg.marker[i]
                self.obstaclesXPos.append(truncate_float(aruco_msg.markers[i].pose.position.x,3))
                self.obstaclesZPos.append(truncate_float(aruco_msg.markers[i].pose.position.z,3))
    
    

"""  
    def timer_callback(self):

            msg = String()

            #msg.data = 'Hello World: %d' % self.i

            self.publisher_.publish(msg)

            #self.get_logger().info('Publishing: "%s"' % msg.data)

            self.i += 1
"""
    
    
    

def main(args=None):

        rclpy.init(args=args)


        minimal_publisher = MinimalPublisher()


        rclpy.spin(minimal_publisher)


        # Destroy the node explicitly

        # (optional - otherwise it will be done automatically

        # when the garbage collector destroys the node object)

        minimal_publisher.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

        main()