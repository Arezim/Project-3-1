#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

class robot_planner:
    def __init__(self):
        rospy.sleep(0.4)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(0.4)
        
        rospy.init_node('move_group_fanuc_crx10ial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.1)
        
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group.set_planning_time(10)
    
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_scene_diff_publisher = rospy.Publisher("planning_scene", moveit_msgs.msg.PlanningScene, queue_size=1)

        rospy.sleep(0.1)

        group_names = self.robot.get_group_names()
        print ("============ Available Planning Groups:", self.robot.get_group_names())

        print ("============ Current Position:", self.move_group.get_current_pose())

    def print_pos(self):
        print ("============ Available Planning Groups:", self.robot.get_group_names())

        print ("============ Current Position:", self.move_group.get_current_pose())


    def move(self,x,y,z,w):
        group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        group.set_pose_target(pose_goal)
        success = group.go(wait=True)
        rospy.sleep(0.5)
        group.stop()
        group.clear_pose_targets()



def main():
    program = robot_planner()
    
    while True:
        time.sleep(3)
        print("starting move 1")
        program.move(0.0,-0.4,1.1,1)
        program.print_pos()
        time.sleep(1)
        # print("starting move 2")
        # program.move(0.3,-0.6,1.2,1)
        # program.print_pos()
        time.sleep(10)
    # while True:
    #    program.print_pos()
    #    time.sleep(1)

main()
