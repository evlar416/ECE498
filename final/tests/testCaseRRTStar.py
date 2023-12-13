import random
import jetbotState as jbs
import rrtDijkstra as rrt


def main(args=None):
    
    # assume starting position of jetbot to be 0,0
    startpos = (0,0)

    ArucoMarkers = {}
    # define 5 total markers
    # NEED TO CONSIDER NEGATIVE COORDINATES!!!
    for i in range(5):
        ArucoMarkers.update({i:(random.randint(-8,8),random.randint(-8,8),random.randint(-8,8))})

    # choose arbitrary goal for now
    # goal = ArucoMarkers[random.randint(0,4)]
    goal_marker = 0
    # endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][2])
    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][1]) # is this or above correct?
    print("END POS: ")
    print(endpos)

    obstacles = []
    # these are exact comparisons, will need to be margin of error when comparing real coordinates (floats)
    for marker in ArucoMarkers.keys():
        if marker == goal_marker:
            continue
        else:
            # obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][1]))
            obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][1])) # which one is correct?
              
    print(obstacles)

    # define max iteration for RRT
    n_iter = 400
              
    # define radius for both reaching goal and avoiding obstacles??
    radius = 0.5 # 10 cm radius
              
    # define stepsize for forming new verticies
    stepSize = 0.3 # need to figure out stepsize for jetbot
              
    # call RRT and Dijkstra to find shortest path to object  
    G = rrt.RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = rrt.dijkstra(G)
        print(path)
        # simulate jetbot movement
        jetbot_state = jbs.state_space(0,0,0)
        jetbot_action = jbs.action_space()
        jetbot_path = [[0,0]]
        hist = (0,0)
        error = [] # use this eventually

        print("PATH LENGTH: ")
        print(len(path))
        for i in range(len(path)-1):
            jbs.state_transition(jetbot_state,jetbot_action,path[i+1]) # jetbot state is updated by this function
            #mv = jetbot_action.movement()
            #jetbot_path.append(((mv[0]+hist[0]),(mv[1]+hist[1])))
            #hist = mv
            jetbot_path.append((jetbot_state.x,jetbot_state.z))

        rrt.plot(G, obstacles, radius, jetbot_path)
        print("JETBOT PATH LENGTH: ")
        print(len(jetbot_path))
        print(jetbot_path)
    '''
        plot(G, obstacles, radius, path)
    else:
        plot(G, obstacles, radius)
    '''

if __name__ == '__main__':
        main()