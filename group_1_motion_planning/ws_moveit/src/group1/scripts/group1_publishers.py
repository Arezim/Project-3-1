#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class Group1Publisher:
    def __init__(self):
        rospy.init_node('group1_publisher')
        self.target_reached = rospy.Publisher("/group1/motion/isTargetReached", Bool, queue_size=10)
        self.motion_failed = rospy.Subscriber("/group1/motion/isFailed", Bool, queue_size=10)
        self.allow_fault_detection = rospy.Publisher("/group1/allowFaultDetection", Bool, queue_size=10)
        self.new_goal_request =  rospy.Subscriber("/group1/getNewGoal", Bool, queue_size=10)

def published_target_reached(self, msg: Bool):
    self.target_reached.publish(msg)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass