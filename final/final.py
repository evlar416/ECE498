
import JetbotMovement as jbm

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist




def main(args=None):

    node = jbm.JetbotMovement()

    node.test_movement()
    
    node.clear_twist()
    
    rclpy.spin(node.node)

    # Destroy the node explicitly
    node.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

        main()
