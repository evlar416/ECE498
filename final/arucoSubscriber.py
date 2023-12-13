from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String

import rclpy
from rclpy.node import Node

class ArucoSubscriber(Node):
    
    def __init__(self, goal_id:int):
        
        super().__init__('aruco_subscriber') # this name will show in node list\
        
        self.goal_id = goal_id
        self.goal_found = False
        
        # create subscriber to aruco_detection messages
        self.sub = self.create_subscription(
                ArucoDetection, # type of message
                'aruco_detections', # topic to subscribe to
                self.listener_callback, # called every time node receives message
                10)
        
        # create dictionary of markers to be populated with id:(x,z) format
        self.markerPosition = {}
        
    # Build/update dictionary of coordinates for each marker every time new message arrives
    def listener_callback(self,aruco_msg):
        for marker in aruco_msg.markers:
            name = marker.marker_id
            x = round(marker.pose.position.x,4)
            z = round(marker.pose.position.z,4)            
            self.markerPosition.update({name:(x,z)})
            if(name == self.goal_id):
                self.goal_found = True
            print("Marker position dict: ", self.markerPosition)
    

    def seeTags(self):

        markers = self.markerPosition
        
        endpos = tuple((markers[self.goal_id][0], markers[self.goal_id][1]))
        
        obstacles = []
        
        for iter in markers:
            if iter == self.goal_id:
                continue
            else:
                obstacles.append((markers[iter][0],markers[iter][1]))
        return endpos,obstacles