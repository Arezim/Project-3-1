#!/usr/bin/env python3

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

class arm_controller_action_server(object):
    _feedback = FollowJointTrajectoryFeedback()
    _feedback.header.seq = -1

    _result = FollowJointTrajectoryResult()
    current_position = [2.2, 0.66, 2.2, 0.66]
    joint_names = ['Soft_RG6_joint_1_1', 'Soft_RG6_joint_1_2', 'Soft_RG6_joint_2_1', 'Soft_RG6_joint_2_2']

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)

        # TODO REMOVE THIS PUBLISHER. THIS IS TO PUBLISH THE JOINT STATES OF THE END EFFECTOR TEMPORARILY
        self.ee_intermediary_joint_state_publisher = rospy.Publisher('/custom/ee/intermediary/joint_states', JointState, queue_size=1)

        self._as.start()
        rospy.loginfo(f"started server {name}")

    def execute_cb(self, goal):
        points = goal.trajectory.points
        self.current_position = points[len(points)-1].positions
        JointStateMsg = JointState()
        JointStateMsg.name = self.joint_names
        JointStateMsg.position = self.current_position
        rospy.loginfo(JointStateMsg)
        self.ee_intermediary_joint_state_publisher.publish(JointStateMsg)

        self._feedback.header.stamp = rospy.Time.now()
        self._feedback.header.seq += 1
        self._feedback.desired = goal.trajectory.points[-1]

        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('gripper_action_server')

    server = arm_controller_action_server('soft_rg6_controller/follow_joint_trajectory')
    rospy.spin()