#!/usr/bin/env python3
import sys, rospy, moveit_commander
rospy.init_node("dump_q", anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander(rospy.get_param("~group","manipulator"))
names = group.get_active_joints()
q = group.get_current_joint_values()
print("names:", names)
print("q:", [round(v,6) for v in q])
