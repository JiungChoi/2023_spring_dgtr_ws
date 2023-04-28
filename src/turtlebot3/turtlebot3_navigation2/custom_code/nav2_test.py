#!/usr/bin/env python3

'''

작성자 : 최지웅
코드목적 : Navigation2 simple command api 사용하기
작성일 : 
    Setting initial pose  : 23.04.19
    Setting goal pose     : 23.04.28

'''

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import sys

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

def setInitialPose():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # -- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Shut down
    rclpy.shutdown()

def goToGoal(x, y):
    # --- Init
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
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    nav.waitUntilNav2Active()

    # --- Go to one pose
    # PI == 3.14 == 180 , PO/2 == 1.57 == 90
    goal_pose1 = create_pose_stamped(nav, 3.0, 5.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 7.0, 1.0, 1.57)
    goal_pose3 = create_pose_stamped(nav, -4.0, 0.0, 1.57)

    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)

    print(nav.getResult())

    # --- Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("하나 이상의 옵션을 입력해주세요.")
        exit()

    if sys.argv[1] == 'init':
        setInitialPose()
    
    elif sys.argv[1] == 'go':
        goToGoal(sys.argv[2], sys.argv[3])

    elif sys.argv[1] == 'waypoint':
        followPoints()
    