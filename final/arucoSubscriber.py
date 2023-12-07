from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String

class ArucoSubscriber(Node):
    
    def __init__(self):
        super().__init__('aruco_subscriber') # this name will show in node list
        
        # create subscriber to aruco_detection messages
        self.sub = self.create_subscription(
                ArucoDetection, # type of message
                'aruco_detections', # topic to subscribe to
                self.listener_callback, # called every time node receives message
                10)
        
        # create dictionary of markers to be populated by messages in "x,z,theta" format
        self.markerOrientation = {}
        self.markerPosition = {}
        
    # Build/update dictionary of coordinates for each marker every time new message arrives
    def listener_callback(self,aruco_msg):
        for marker in aruco_msg.markers:
            id = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            self.markerPosition.update({id:[x,y,z]}) 

def seeTags():
    # create node
    aruco_subscriber = ArucoSubscriber()
    # define markers
    markers = aruco_subscriber.markers
    # assume starting position of jetbot to be 0,0
    startpos = (0,0)
    # choose arbitrary goal 0
    goal_id = 0
    endpos=(markers.markerPosition[goal_id][0],markers[goal_id][2]) # is this or [goal_id][1] correct?

    obstacles = []
    for iter in markers:
        if iter.id == goal_id:
            continue
        else:
            obstacles.append((iter.markerPosition[iter.id][0],iter.markerPosition[iter.id][2])) # which one is correct? ([1] or [2])
    return endpos,obstacles