#!/usr/bin/env python3

'''

작성자 : 최지웅
코드목적 : Navigation2 simple command api 사용하기
작성일 : 
    Setting initial pose : 23.04.19
    Setting goal pose    : 23.04.28
    Add Costmap Genertor : 23.05.01
    Add Pathplanner      : 23.05.02
    Add ThrawPath        : 23.05.
    Config PF Framework  : 23.05.12

'''

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
import tf_transformations
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from Search_2D.Searched_based import Rtaastar, Astar, D_star
from Search_2D.Sampling_based import RrtConn, Rrtori, rrt_star_smart, rrtstar, dynamic_rrt

from time import sleep

SCALE_FACTOR = 3 # 이미지 맵을 스케일링 할 비율
NAV2_MAP_ORIGIN_X = 6
NAV2_MAP_ORIGIN_Y = 5
NAV2_MAP_SIZE_X = 16
NAV2_MAP_SIZE_Y = 11
THETA = 0

def create_pose_stamped(navigator : BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    
    return pose

def setInit():
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

def goToGoal(x, y):
    rclpy.init()
    nav = BasicNavigator()

    nav.waitUntilNav2Active()

    # --- Go to one pose
    # PI == 3.14 == 180 , PO/2 == 1.57 == 90
    goal_pose = create_pose_stamped(nav, float(x), float(y), 1.57)
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    # --- Shut down
    rclpy.shutdown()

def followPoints():
    rclpy.init()
    nav = BasicNavigator()

    nav.waitUntilNav2Active()

    # Go to one pose : PI == 3.14 == 180 , PO/2 == 1.57 == 90
    goal_pose1 = create_pose_stamped(nav, 3.0, 5.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 7.0, 1.0, 1.57)
    goal_pose3 = create_pose_stamped(nav, -4.0, 0.0, 1.57)

    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    print(nav.getResult())

    rclpy.shutdown()

def make2Dcostmap(map):
    sizeX, sizeY = map.metadata.size_x, map.metadata.size_y
    map = map.data.tolist()
    map = np.array(map)
    map = np.reshape(map, (sizeY, sizeX)).astype('uint8')
    map = cv2.resize(map, (int(sizeY*SCALE_FACTOR), int(sizeX*SCALE_FACTOR)))
    map = np.transpose(map)
    map = cv2.flip(map, 1)
    map = cv2.rotate(map, cv2.ROTATE_90_COUNTERCLOCKWISE)
    print("MAP_SIZE : ", sizeX, sizeY)
    print("The number of fixel : ", sizeX*sizeY)
    return map
    
def plotCostmap(map, title):
    plt.figure()
    plt.imshow(map)
    plt.title(title)
    plt.colorbar()

def mapLoader(nav : BasicNavigator):
    globalMap = nav.getGlobalCostmap()
    globalMap = make2Dcostmap(globalMap)
    
    localMap = nav.getLocalCostmap()
    localMap = make2Dcostmap(localMap)

    ret, binGlobalMap = cv2.threshold(globalMap, 250, 255, cv2.THRESH_BINARY)

    plotCostmap(localMap, 'LocalMap')
    plotCostmap(globalMap, 'GlobalMap')
    plotCostmap(binGlobalMap, 'binaryGlobalMap')

    plt.show()

    return binGlobalMap

## -- nav2 map coord -> Img map coord
def nav2img(coord, cost_map_shape):
    return transformer([(coord.x, coord.y)], cost_map_shape, 0)

## -- Img map path <-> nav2 map path 
#       if type == 0  : nav2_map -> img_map
#       else type == 1 : img_map -> nav2_map
def transformer(path, cost_map_shape, type):
    cost_map_w, cost_map_h = cost_map_shape

    print("Init Path: ", path)
    ## -- Convert path for affine transform
    path = np.transpose(path)
    _, n = path.shape # the number of point in path

    ones = np.ones((1, n))
    path = np.vstack([path, ones])
    '''
    plt.plot(path[0,:],path[1,:], color='darkmagenta', marker='o', linestyle='solid')
    print("Image Map: \n", path, path.shape)
    '''

    ## -- Set parameter for affine trasnform
    dx = NAV2_MAP_ORIGIN_X/NAV2_MAP_SIZE_X*cost_map_w
    dy = NAV2_MAP_ORIGIN_Y/NAV2_MAP_SIZE_Y*cost_map_h
    sx = NAV2_MAP_SIZE_X/cost_map_w #90도 회전하기 전이라 y축에 대한 스케일링을 해야함
    sy = NAV2_MAP_SIZE_Y/cost_map_h #90도 회전하기 전이라 x축에 대한 스케일링을 해야함
    

    ## -- Make Matrix for affine trans
    R = np.array([[ np.cos(THETA), -np.sin(THETA),  0], # Rotation
                  [ np.sin(THETA),  np.cos(THETA),  0],
                  [      0       ,       0       ,  1] ])
    
    T = np.array([[ 1, 0,  -dx], # Translation
                  [ 0, 1,  -dy],
                  [ 0, 0,  1] ])

    S = np.array([[sx,0 ,  0],  # Scaling
                  [0 ,sy,  0],
                  [0 ,0 ,  1] ])
    
    
    ## -- Circlur Matrix
    if type :
        path = S@T@R@path

        '''
        path = R@path
        print("R: ", R, R.shape)
        print("Rotate: \n", path, path.shape)
        plt.plot(path[0,:],path[1,:], color='b', marker='o', linestyle='solid')
        
        path = T@path
        print("T: ", T, T.shape)
        print("Translation: \n", path, path.shape)
        plt.plot(path[0,:],path[1,:], color='g', marker='o', linestyle='solid')


        path = S@path
        print("S: ", S, S.shape)
        print("Scaling: \n", path, path.shape)
        plt.plot(path[0,:],path[1,:], color='r', marker='o', linestyle='solid')
        
        
        plt.axis([-60, 60, -60, 60])
        plt.grid(True)
        plt.show()
        '''

        print("Converted Path (img map path -> nav2): ", path)
        return path
    
    else:
        path = np.linalg.inv(S@T@R)@path
        path = (int(path[0]), int(path[1]))
        print("Converted coord(nav2 -> img map): ", path)
        return path
    
    

def rtPathplanner():
    rclpy.init()
    nav = BasicNavigator()
    
    while(True):
        goal_point = input("목표 위치를 입력해 주세요. (종료: -1) : ")
        if goal_point == "-1":
            break
        else :
            goal_point.replace(" ", "")
            goal_point = goal_point.split(",")
            goal_point = list(map(int, goal_point))
            
            # Set our demo's initial pose
            initial_pose = create_pose_stamped(nav, nav.now_position.pose.position.x, nav.now_position.pose.position.y, 0.0)
            nav.setInitialPose(initial_pose)

            # Wait for navigation to fully activate, since autostarting nav2
            nav.waitUntilNav2Active()

            # Go to our demos first goal pose
            goal_pose = create_pose_stamped(nav, float(goal_point[0]), float(goal_point[1]), 0.0)

            binGlobalMap = mapLoader(nav)
            dstar = D_star.DStar(initial_pose.pose.position, goal_pose.pose.position, binGlobalMap)
            custom_path = dstar.run()

            # Convert nav2 map path from img map path            
            transformer(custom_path, (binGlobalMap.shape[1], binGlobalMap.shape[0]), 1)


            '''
            # Get the path, smooth it
            path = nav.getPath(initial_pose, goal_pose)
            print(path)

            print("hehe", type(path))
            
            #path = nav.smoothPath(path)
            '''

            # Follow path
            nav.followPath(custom_path)
            
            i = 0
            while not nav.isTaskComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                i += 1
                feedback = nav.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated distance remaining to goal position: ' +
                        '{0:.3f}'.format(feedback.distance_to_goal) +
                        '\nCurrent speed of the robot: ' +
                        '{0:.3f}'.format(feedback.speed))
            

            # Do something depending on the return code
            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                nav.now_position = goal_pose
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')



    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

## Searched based path planner
def runAstar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    astar = Astar.AStar(s_start, s_goal, "euclidean", binGlobalMap)
    path = astar.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

    return path

def runDstar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    dstar = D_star.DStar(s_start, s_goal, binGlobalMap)
    path = dstar.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()
    return path

def rtaAstar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    rtaa = Rtaastar.RTAAStar(s_start, s_goal, 240, "euclidean", binGlobalMap)
    rtaa.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()


## Sampling based path planner
def rrt(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    rrt = Rrtori.Rrt(s_start, s_goal, 0.5, 0.05, 500000, binGlobalMap)
    rrt.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()
    
def rrtConn(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    rrt_conn = RrtConn.RrtConnect(s_start, s_goal, 0.8, 0.05, 1000, binGlobalMap)
    rrt_conn.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

def rrtSmart(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)

    rrt_smart = rrt_star_smart.RrtStarSmart(s_start, s_goal, 1.5, 0.10, 2, 1000, binGlobalMap)
    rrt_smart.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

def rrtStar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)

    rrt_star = rrtstar.RrtStar(s_start, s_goal, 10, 0.10, 20, 10000, binGlobalMap)
    rrt_star.run()
    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

def drrt(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)

    drrt = dynamic_rrt.DynamicRrt(s_start, s_goal, 0.5, 0.1, 0.6, 50000, binGlobalMap)
    drrt.run()
    
    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()



if __name__ == '__main__':
    

    if len(sys.argv) == 1:
        print("하나 이상의 옵션을 입력해주세요.:\n\n init: 초기화 , go: x y 값으로 이동 , waypoint: waypoint 사용해서 이동")
        exit()

    if sys.argv[1] == 'init':
        setInit()
    
    elif sys.argv[1] == 'go':
        goToGoal(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'waypoint':
        followPoints()
    
    elif sys.argv[1] == 'rtpath':
        rtPathplanner()
    
    elif sys.argv[1] == 'run_astar':
        runAstar(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_rtaastar':
        rtaAstar(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_dstar':
        runDstar(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_rrt':
        rrt(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_rrtconn':
        rrtConn(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_rrtsmart':
        rrtSmart(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_rrtstar':
        rrtStar(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'run_drrt':
        drrt(sys.argv[2], sys.argv[3])
    
    