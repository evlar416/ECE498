from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String
from rclpy.node import Node

import arucoSubscriber as aruco
import rrtDijkstra as rrt
import jetbotState as jbs

def main(args=None):
    startpos = (0,0)
    endpos,obstacles = aruco.seeTags()
    G = rrt.RRT(startpos, endpos, obstacles, 400, 10, 0.03) # (nStep/radius/stepSize) figure out best parameters
    if G.success:
        path = rrt.dijkstra(G)

if __name__ == '__main__':

        main()