import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import final.JetbotMovement as jbm
import final.arucoSubscriber as aruco
import final.rrtDijkstra as rrt
import final.jetbotState as jbs

import time

GOAL_ID = 0

def main(args=None):
    
    rclpy.init()
    
    aruco_sub = aruco.ArucoSubscriber(GOAL_ID)
    node = jbm.JetbotMovement()
    node.clear_twist()
    
    length = -1
    
    # while aruco tags are still being added and the goal tag has not been found, keep spinning the aruco subscriber node
    while((len(aruco_sub.markerPosition) > length) and (not aruco_sub.goal_found)):
        rclpy.spin_once(aruco_sub)
        length = len(aruco_sub.markerPosition)

    
    jetState = jbs.state_space(0,0,0)
    jetAction = jbs.action_space()

    startpos = (0,0)
    endpos,obstacles = aruco_sub.seeTags()
    print("End pos: ", endpos, " obstables: ",obstacles)
    
    G = rrt.RRT(startpos, endpos, obstacles, 400, 0.1, 0.03) # (nStep/radius/stepSize) figure out best parameters
    if G.success:
        print("Path planning succeeded")
        path = rrt.dijkstra(G)
    else:
        print("Path planning failed")
    
    aruco_sub.destroy_node()

    for p in path[1:]:
        node.move(jbs.state_transition(jetState,jetAction,p))
        node.clear_twist()
    
    rclpy.spin(node.node)
    

    # Destroy the node explicitly
    node.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

        main()
