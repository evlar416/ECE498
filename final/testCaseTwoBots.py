import random
import jetbotState as jbs
import rrtDijkstra as rrt
import numpy as np
import matplotlib.animation as ani
import math


def main(args=None):
    
    # assume starting position of jetbot to be 0,0
    startpos1 = (0,0)
    startpos2 = (1,0)

    ArucoMarkers = {}
    # define 5 total markers

    ArucoMarkers.update({0:(1,1)})
    ArucoMarkers.update({1:(2,2)})
    ArucoMarkers.update({2:(3,3)})
    ArucoMarkers.update({3:(4,4)})
    ArucoMarkers.update({4:(5,5)})

    '''
    # With two jetbots goal and obstacles will have different positons
    for i in range(5):
        ArucoMarkers.update({i:(random.randint(-8,8),random.randint(-8,8),random.randint(-8,8))})

    # choose arbitrary goal for now
    # goal = ArucoMarkers[random.randint(0,4)]
    goal_marker = 0

    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][2])
    print("END POS: ")
    print(endpos)
    '''

    goal_marker = random.randint(0,4)
    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][1])
    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][1])

    obstacles = []

    for marker in ArucoMarkers.keys():
        if marker == goal_marker:
            continue
        else:
            obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][1]))
              
    print(obstacles)

    # define max iteration for RRT
    n_iter = 200
              
    # define radius for both reaching goal and avoiding obstacles??
    radius = 0.4 # adjust for 10 cm radius for jetbot 
              
    # define stepsize for forming new verticies
    stepSize = 0.4 # adjust for jetbot
              
    # call RRT and Dijkstra to find shortest path to object  
    G1 = rrt.RRT_star(startpos1, endpos, obstacles, n_iter, radius, stepSize)
    G2 = rrt.RRT_star(startpos2, endpos, obstacles, n_iter, radius, stepSize)
    if G1.success and G2.success:
        path1 = rrt.dijkstra(G1)
        print("path1 success")
        path1 = rrt.smooth(path1,obstacles,radius)
        # simulate jetbot movement
        jetbot_state1 = jbs.state_space(startpos1[0],startpos1[1],math.pi/2)
        jetbot_action1 = jbs.action_space()
        jetbot_path1 = [startpos1]

        print("path2 success")
        path2 = rrt.dijkstra(G2)
        path2 = rrt.smooth(path2,obstacles,radius)
        jetbot_state2 = jbs.state_space(startpos2[0],startpos2[1],math.pi/2)
        jetbot_action2 = jbs.action_space()
        jetbot_path2 = [startpos2]

        fin = 0
        fin1 = 0
        fin2 = 0
        while not fin:
            jbs.collision_handler(jetbot_state1,jetbot_state2,path1,path2,radius)

            if jetbot_state1.halt != 1:
                if not np.allclose(jetbot_path1[-1],path1[-1]):
                    jbs.state_transition(jetbot_state1,jetbot_action1,path1[jetbot_state1.journeylen + 1]) # jetbot state is updated by this function
                    jetbot_path1.append((jetbot_state1.x,jetbot_state1.z))
                else:
                    fin1 = 1

            if jetbot_state2.halt != 1:
                if not np.allclose(jetbot_path2[-1],path2[-1]):
                    jbs.state_transition(jetbot_state2,jetbot_action2,path2[jetbot_state2.journeylen + 1]) # jetbot state is updated by this function
                    jetbot_path2.append((jetbot_state2.x,jetbot_state2.z))
                else:
                    fin2 = 1

            if fin1 and fin2:
                fin = 1
            else:
                continue

        print(jetbot_path1)
        print(jetbot_path2)
        rrt.plot(G1, obstacles, radius, jetbot_path1,jetbot_path2)
        #rrt.plot(G2, obstacles, radius, jetbot_path2)
    else:
        print("FAILURE")
        rrt.plot(G1, obstacles, radius)


if __name__ == '__main__':
        main()