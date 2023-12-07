
import JetbotMovement as jbm

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import arucoSubscriber as aruco
import rrtDijkstra as rrt
import jetbotState as jbs


def main(args=None):
    jetState = jbs.state_space(0,0,0)
    jetAction = jbs.action_space()

    startpos = (0,0)
    endpos,obstacles = aruco.seeTags()
    G = rrt.RRT(startpos, endpos, obstacles, 400, 10, 0.03) # (nStep/radius/stepSize) figure out best parameters
    if G.success:
        path = rrt.dijkstra(G)

    node = jbm.JetbotMovement()

    node.test_movement()
    for p in path[1:]:
        jbm.move(jbs.state_transition(jetState,jetAction,p))
        node.clear_twist()
    
    rclpy.spin(node.node)

    # Destroy the node explicitly
    node.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

        main()
