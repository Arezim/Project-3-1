#!/usr/bin/env python3

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from iiwa_msgs.msg import MoveAlongJointSplineAction, MoveAlongJointSplineGoal, JointPositionVelocity, JointQuantity
# from tres_motores_msgs.msg import position_commandAction
from tres_motores_msgs import msg
import time

class arm_controller_action_server(object):
    _feedback = FollowJointTrajectoryFeedback()
    _feedback.header.seq = -1

    _result = FollowJointTrajectoryResult()

    expected_joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 
                            'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 
                            'iiwa_joint_7',]
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self.iiwa_joint_action_client = actionlib.SimpleActionClient('/iiwa/action/move_along_joint_spline', MoveAlongJointSplineAction)

        # self.le_teknic_joint_spline_client = actionlib.SimpleActionClient("/linear_extension/teknic/joint_spline_action_server",FollowJointTrajectoryAction)
        # self.le_dynamixel_joint_spline_client = actionlib.SimpleActionClient("/linear_extension/dynamixel/joint_spline_action_server",FollowJointTrajectoryAction)

        # rospy.logwarn('arm_controller_action_server >> Waiting for linear_extension/{teknic,dynamixel}/joint_spline_action_server')
        # self.le_teknic_joint_spline_client.wait_for_server()
        # rospy.logwarn('arm_controller_action_server >> linear_extension/teknic/joint_spline_action_server is available now')
        # self.le_dynamixel_joint_spline_client.wait_for_server()
        # rospy.logwarn('arm_controller_action_server >> linear_extension/dynamixel/joint_spline_action_server is available now')

        self.joint_trajectory_publisher = rospy.Publisher("/custom/temp_publish",FollowJointTrajectoryGoal, queue_size=1)

        self._as.start()
        rospy.logwarn(f"arm_controller_action_server >> started server {name}")

    def execute_cb(self, goal):
        rospy.logwarn('arm_controller_action_server >> received goal')
        iiwa_joint_spline_goal = MoveAlongJointSplineGoal()
        iiwa_joint_spline_goal.spline.segments = []
        self.joint_trajectory_publisher.publish(goal)
    
        points_points = goal.trajectory.points
        for i in range(len(points_points)):
            p = points_points[i].positions
            segment = JointPositionVelocity() 
            segment.position = JointQuantity()
            segment.position.a1 = p[0]
            segment.position.a2 = p[1]
            segment.position.a3 = p[2]
            segment.position.a4 = p[3]
            segment.position.a5 = p[4]
            segment.position.a6 = p[5]
            segment.position.a7 = p[6]
            
            v = None
            if i == len(points_points)-1:
                v = points_points[i-1].velocities
            elif i == 0:
                v = points_points[1].velocities
            else:
                v = points_points[i].velocities
            
            segment.velocity = JointQuantity()
            segment.velocity.a1 = v[0]
            segment.velocity.a2 = v[1]
            segment.velocity.a3 = v[2]
            segment.velocity.a4 = v[3]
            segment.velocity.a5 = v[4]
            segment.velocity.a6 = v[5]
            segment.velocity.a7 = v[6]
            iiwa_joint_spline_goal.spline.segments.append(segment)

        rospy.logwarn('sending goal')

        # self.le_teknic_joint_spline_client.send_goal(goal)
        # self.le_dynamixel_joint_spline_client.send_goal(goal)
        self.iiwa_joint_action_client.send_goal_and_wait(iiwa_joint_spline_goal)

        self._feedback.header.seq += 1
        self._feedback.header.stamp = rospy.Time.now()
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('arm_action_server')
    server = arm_controller_action_server('arm_controller/follow_joint_trajectory')
    rospy.spin()
