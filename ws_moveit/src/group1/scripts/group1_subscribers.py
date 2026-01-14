#!/usr/bin/env python3

import rospy
import sys
sys.path.append("..")
from group1_publishers import Group1Publisher
from std_msgs.msg import Float64MultiArray, Bool, Float64

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_grasp_completed(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_graspOffset(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_isSliderGrabbed(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('group1_listener')
    rospy.Subscriber("/group2/slider/pos", Float64MultiArray, callback)
    rospy.Subscriber("/group4/grasp/isCompleted", Bool, callback_grasp_completed)
    rospy.Subscriber("/group4/graspOffset", Float64, callback_graspOffset)
    rospy.Subscriber("/group4/isSliderGrabbed", Float64, callback_isSliderGrabbed)
    group1_publisher = Group1Publisher()
    group1_publisher.published_target_reached(True)



    rospy.spin()

if __name__ == '__main__':
    listener()
