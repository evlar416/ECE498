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
            
            
            self.goal = self.sub.ArucoDetection.markers[1]
            self.goalPos = (truncate_float(self.goal.pose.position.x,3), truncate_float(self.goal.pose.position.z,3))
            self.origin = (0,0)
            self.obstacles = {}
            self.obstaclesXPos = []
            self.obstaclesZPos = []
            
            self.getObstacles(self)
            
            print("Goal pos = ", goalPos)
            for i in self.obstaclesXPos[i]:
                print("Obstacle ",i," pos = ", self.obstaclesXPos[i],",",self.obstaclesZPos[i])
            
            
            self.pub = self.create_publisher(Twist, 'jetbot/cmd_vel', 10)
            
            
    def listener_callback(self, aruco_msg):
        """
        x0 = aruco_msg.markers[0].pose.position.x
        z0 = aruco_msg.markers[0].pose.position.z
        theta0 = aruco_msg.markers[0].pose.orientation.z
        print("Marker 0: x = ", x0, " | z = ", z0, " | theta = ", theta0)
        
        x1 = aruco_msg.markers[1].pose.position.x
        z1 = aruco_msg.markers[1].pose.position.z
        theta1 = aruco_msg.markers[1].pose.orientation.z
        print("Marker 1: x = ", x1, " | z = ", z1, " | theta = ", theta1)
        """
        
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

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    
    
    

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