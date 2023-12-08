
from final.JetbotMovement import JetbotMovement

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import final.arucoSubscriber as aruco
import final.rrtDijkstra as rrt
import final.jetbotState as jbs


def main(args=None):
    
    rclpy.init()
    
    aruco_sub = aruco.ArucoSubscriber()
    
    jetState = jbs.state_space(0,0,0)
    jetAction = jbs.action_space()

    startpos = (0,0)
    endpos,obstacles = aruco.seeTags()
    G = rrt.RRT(startpos, endpos, obstacles, 400, 10, 0.03) # (nStep/radius/stepSize) figure out best parameters
    if G.success:
        path = rrt.dijkstra(G)

    node = JetbotMovement.JetbotMovement()

    for p in path[1:]:
        jbm.move(jbs.state_transition(jetState,jetAction,p))
        node.clear_twist()
    
    rclpy.spin(node.node)

    # Destroy the node explicitly
    node.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

        main()
