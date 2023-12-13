import random
import jetbotState as jbs
import rrtDijkstra as rrt


def main(args=None):
    
    # assume starting position of jetbot to be 0,0
    startpos1 = (0,0)
    startpos2 = (2,0)

    ArucoMarkers = {}
    # define 5 total markers
    # With two jetbots goal and obstacles will have different positons
    for i in range(5):
        ArucoMarkers.update({i:(random.randint(-8,8),random.randint(-8,8),random.randint(-8,8))})

    # choose arbitrary goal for now
    # goal = ArucoMarkers[random.randint(0,4)]
    goal_marker = 0

    endpos=(ArucoMarkers[goal_marker][0],ArucoMarkers[goal_marker][2])
    print("END POS: ")
    print(endpos)

    obstacles = []

    for marker in ArucoMarkers.keys():
        if marker == goal_marker:
            continue
        else:
            obstacles.append((ArucoMarkers[marker][0],ArucoMarkers[marker][2]))
              
    print(obstacles)

    # define max iteration for RRT
    n_iter = 200
              
    # define radius for both reaching goal and avoiding obstacles??
    radius = 0.5 # adjust for 10 cm radius for jetbot 
              
    # define stepsize for forming new verticies
    stepSize = 0.3 # adjust for jetbot
              
    # call RRT and Dijkstra to find shortest path to object  
    G1 = rrt.RRT_star(startpos1, endpos, obstacles, n_iter, radius, stepSize)
    G2 = rrt.RRT_star(startpos2, endpos, obstacles, n_iter, radius, stepSize)
    if G1.success:
        path1 = rrt.dijkstra(G1)
        # print(path)
        # simulate jetbot movement
        jetbot_state1 = jbs.state_space(startpos1[0],startpos1[1],0)
        jetbot_action1 = jbs.action_space()
        jetbot_path1 = [[0,0]]
    if G2.success:
        path2 = rrt.dijkstra(G2)
        jetbot_state2 = jbs.state_space(startpos2[0],startpos2[1],0)
        jetbot_action2 = jbs.action_space()
        jetbot_path2 = [[0,0]]


        while not  fin:
            jbs.collision_handler(jetbot_state1,jetbot_state2,path1,path2,radius)

            if jetbot_state1.halt != 1:
                if jetbot_path1[-1] != path1[-1]:
                    jbs.state_transition(jetbot_state1,jetbot_action1,path1[jetbot_state1.journeylen + 1]) # jetbot state is updated by this function
                    jetbot_path1.append((jetbot_state1.x,jetbot_state1.z))

            if jetbot_state2.halt != 1:
                if jetbot_path1[-1] != path1[-1]:
                    jbs.state_transition(jetbot_state2,jetbot_action2,jetbot_path2[jetbot_state2.journeylen + 1]) # jetbot state is updated by this function
                    jetbot_path2.append((jetbot_state2.x,jetbot_state2.z))

        rrt.plot(G1, obstacles, radius, jetbot_path1,jetbot_path2)
        
    '''
        plot(G, obstacles, radius, path)
    else:
        plot(G, obstacles, radius)
    '''

if __name__ == '__main__':
        main()