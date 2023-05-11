#!/usr/bin/env python3

'''

작성자 : 최지웅
코드목적 : Navigation2 simple command api 사용하기
작성일 : 
    Setting initial pose : 23.04.19
    Setting goal pose    : 23.04.28
    Add Costmap Genertor : 23.05.01
    Add Pathplanner      : 23.05.02

'''

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from Search_2D.Searched_based import Rtaastar, Astar, D_star
from Search_2D.Sampling_based import RrtConn, Rrtori, rrt_star_smart, rrtstar, dynamic_rrt

from time import sleep

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
    map = cv2.rotate(map, cv2.ROTATE_180)
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

def rtPathplanner():
    rclpy.init()
    nav = BasicNavigator()

    '''
    binGlobalMap = mapLoader(nav)
    custom_path = runAstar(x_start, x_goal)
    '''
    print(nav.get_parameters)

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

## Searched based path planner
def runAstar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    astar = Astar.AStar(s_start, s_goal, "euclidean", binGlobalMap)
    astar.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

def runDstar(s_start, s_goal):
    rclpy.init()
    nav = BasicNavigator()

    binGlobalMap = mapLoader(nav)
    
    dstar = D_star.DStar(s_start, s_goal, binGlobalMap)
    dstar.run()

    # Wait for Nav2 && Shut down
    nav.waitUntilNav2Active()
    rclpy.shutdown()

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
    SCALE_FACTOR = 0.3 # 0.3

    if len(sys.argv) == 1:
        print("하나 이상의 옵션을 입력해주세요.:\n\n init: 초기화 , go: x y 값으로 이동 , waypoint: waypoint 사용해서 이동")
        rtPathplanner()
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
    