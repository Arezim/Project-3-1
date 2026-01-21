#!/usr/bin/env python3

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult

from iiwa_msgs.msg import JointPosition, JointPositionVelocity, Spline, SplineSegment, CartesianPose, JointQuantity
from iiwa_msgs.msg import MoveAlongSplineActionGoal, MoveAlongSplineGoal, MoveAlongJointSplineAction, MoveAlongJointSplineGoal
from trajectory_msgs.msg import JointTrajectoryPoint 
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import rospy
import actionlib

from tres_motores_msgs import msg

import numpy as np
from threading import Event
import math

class followJointTrajectoryActionServer(object):
    _feedback = FollowJointTrajectoryFeedback()
    _feedback.header.seq = -1
    _result = FollowJointTrajectoryResult()

    VELOCITY_MULTIPLIER = 5

    goal_event = Event()
    start_time = None
    current_position_index = 0
    goal = None
    abort = False

    expected_joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 
                            'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 
                            'iiwa_joint_7', 'm_robot_linear_drive_dof1_dof2_joint', 
                            'm_robot_linear_drive_guiderail_sledge_joint', 'm_robot_linear_drive_sledge_dof1_joint']
    

    def __init__(self, name):
        self._action_name = name

        self.goal_event.set()
        self.joint_position_velocity_subscriber = rospy.Subscriber('/iiwa/state/JointPositionVelocity', JointPositionVelocity,self.joint_states_callback)
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self.follow_joint_traj_publisher = rospy.Publisher("/custom/follow_joint_traj",FollowJointTrajectoryGoal, queue_size=1)
        # ---
        self.iiwa_joint_position_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
        self.iiwa_joint_action_client = actionlib.SimpleActionClient('/iiwa/action/move_along_joint_spline', MoveAlongJointSplineAction)
        self.iiwa_joint_position_msg = JointPosition()
        self.iiwa_joint_position_msg.header.seq = -1

        # --- service proxy for forward kinematics
        self.forward_kinematics_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        # ---
        rospy.loginfo('start waiting for cavid_client')
        self.cavid_client = actionlib.SimpleActionClient("pos_tres_motores_controller",msg.position_commandAction)
        self.cavid_client.wait_for_server()
        rospy.loginfo('cavid_client is there :)')

        self._as.start()
        rospy.loginfo(f"started server {name}")

    def joint_states_callback(self, joint_position_velocity_msg):
        
        if not self.goal_event.is_set():
            # TODO add linear drive positions
            linear_drive_pos =self.goal.trajectory.points[self.current_position_index].positions
            actual_position = np.array([ joint_position_velocity_msg.position.a1, joint_position_velocity_msg.position.a2, joint_position_velocity_msg.position.a3,
                               joint_position_velocity_msg.position.a4, joint_position_velocity_msg.position.a5, joint_position_velocity_msg.position.a6,
                                joint_position_velocity_msg.position.a7, linear_drive_pos[7],linear_drive_pos[8],linear_drive_pos[9] ])
            desired_position = np.array(self.goal.trajectory.points[self.current_position_index].positions)

            # TODO come up with good tolerances
            if np.allclose(actual_position, desired_position, atol=0.003):
                # --- check last point
                if self.current_position_index == len(self.goal.trajectory.points) - 1:
                    self.current_pos = JointTrajectoryPoint()
                    self.current_pos.positions = []
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a1)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a2)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a3)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a4)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a5)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a6)
                    self.current_pos.positions.append(joint_position_velocity_msg.position.a7)

                    self.current_pos.velocities = []
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a1)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a2)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a3)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a4)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a5)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a6)
                    self.current_pos.velocities.append(joint_position_velocity_msg.velocity.a7)
                
                self.cavid_client.wait_for_result()
                self.goal_event.set()
                
            else:
                time_needed = (rospy.Time.now() - self.start_time).to_sec()
                max_time = self.start_time.to_sec() + self.goal.trajectory.points[-1].time_from_start.to_sec() + self.goal.goal_time_tolerance.to_sec()
                if time_needed > max_time:
                    self.abort = True
                    self._result.error_code = -5 # goal tolerance violation
                    self._result.error_string = f"current time {time_needed} is more than time tolerance {max_time} to reach the final goal"
                    
                    self.cavid_client.wait_for_result()
                    self.goal_event.set()
                if (rospy.Time.now() - self.start_time).to_sec() > self.goal.trajectory.points[self.current_position_index].time_from_start.to_sec():
                    rospy.logdebug_throttle(60,"something wrong with the acceleration of the robot (line 92 followJointTrajectoryActionServer.py)")
                    # self.abort = True
                    # self._result.error_code = -4 # path tolerance violation
                    # self._result.error_string = f"The joints {actual_position} did not reach the expected position {desired_position} within path time limit"
                    # self.goal_event.set()
                
    do_spline = True
    def execute_cb(self, goal):
        if not self.do_spline:
        # self.follow_joint_traj_publisher.publish(goal)
            positions = goal.trajectory.points
            self.goal = goal
            self.start_time = rospy.Time.now()

            print(goal)

            joint_names = self.goal.trajectory.joint_names
            for i in range(len(self.expected_joint_names)):
                if joint_names[i]!=self.expected_joint_names[i]:
                    self.abort=True
                    self._result.error_code = -2
                    self._result.error_string = f"the provided joint names {joint_names} where not equal to the expected joint names {self.expected_joint_names}"
            for i in range(1,len(positions)):
                self.goal_event.clear()
                if self.abort:
                    break # TODO

                self.iiwa_joint_position_msg.header.seq += 1
                self.iiwa_joint_position_msg.header.stamp = rospy.Time.now()
                pos = positions[i].positions
                vel = positions[i].velocities
                self.iiwa_joint_position_msg.position.a1 = pos[0]
                self.iiwa_joint_position_msg.position.a2 = pos[1]
                self.iiwa_joint_position_msg.position.a3 = pos[2]
                self.iiwa_joint_position_msg.position.a4 = pos[3]
                self.iiwa_joint_position_msg.position.a5 = pos[4]
                self.iiwa_joint_position_msg.position.a6 = pos[5]
                self.iiwa_joint_position_msg.position.a7 = pos[6]

                # self.iiwa_joint_position_msg.velocity.a1 = vel[0]
                # self.iiwa_joint_position_msg.velocity.a2 = vel[1]
                # self.iiwa_joint_position_msg.velocity.a3 = vel[2]
                # self.iiwa_joint_position_msg.velocity.a4 = vel[3]
                # self.iiwa_joint_position_msg.velocity.a5 = vel[4]
                # self.iiwa_joint_position_msg.velocity.a6 = vel[5]
                # self.iiwa_joint_position_msg.velocity.a7 = vel[6]
                self.iiwa_joint_position_publisher.publish(self.iiwa_joint_position_msg)

                # --- linear slide linear joint
                # Motor 1
                # goal = msg.position_commandGoal()
                single_pos_command = msg.single_pos_command()
                single_pos_command.pos = 90. - (pos[9] / (math.pi/180.)) # TODO hmmmm 
                single_pos_command.vel = 50
                single_pos_command.acc = 50
                single_pos_command.motor_type = 1
                single_pos_command.id = 0
                # # Motor 2
                single_pos_command_2 = msg.single_pos_command()
                single_pos_command_2.pos = -pos[7] / (math.pi/180.)
                single_pos_command_2.vel = 30
                single_pos_command_2.acc = 70
                single_pos_command_2.motor_type = 1
                single_pos_command_2.id = 1

                # # Motor Teknic

                goal_2 = msg.position_commandGoal()
                single_pos_command_3 = msg.single_pos_command()
                single_pos_command_3.pos = pos[8] - 0.09 # TODO hmmmm -8cm because of the offset in the model
                single_pos_command_3.vel = 600 # 4000
                single_pos_command_3.acc = 600 # 4000
                single_pos_command_3.motor_type = 0 # Teknic == 0
                single_pos_command_3.id = 0

                goal_2.pos_command.append(single_pos_command)
                goal_2.pos_command.append(single_pos_command_2)
                goal_2.pos_command.append(single_pos_command_3)
                self.cavid_client.send_goal(goal_2)

                rospy.loginfo("target position")
                rospy.loginfo(self.iiwa_joint_position_msg)
                rospy.loginfo("published point")
                self.current_position_index=i
                self.goal_event.wait()

                
            self._feedback.header.stamp = rospy.Time.now()
            self._feedback.header.seq += 1
            self._feedback.desired = goal.trajectory.points[-1]
            if not self.abort:
                self._feedback.actual = self.current_pos

                # TODO, REPLACE WITH THE LINEAR DRIVE WHEN CAVID FINALLY GETS BACK TO HELP US
                self._feedback.actual.positions.append(self._feedback.desired.positions[7])
                self._feedback.actual.positions.append(self._feedback.desired.positions[8])
                self._feedback.actual.positions.append(self._feedback.desired.positions[9])
                self._feedback.actual.velocities.append(self._feedback.desired.velocities[7])
                self._feedback.actual.velocities.append(self._feedback.desired.velocities[8])
                self._feedback.actual.velocities.append(self._feedback.desired.velocities[9])

                self._feedback.error = JointTrajectoryPoint()
                self._feedback.error.positions=[]
                self._feedback.error.velocities=[]
                # for j in range(10):
                #     self._feedback.error.positions.append(self._feedback.actual.positions[i]-self._feedback.desired.positions[i])
                #     self._feedback.error.velocities.append(self._feedback.actual.velocities[i]-self._feedback.desired.velocities[i])

            self.current_pos = None
            self._as.publish_feedback(self._feedback)
            if self.abort:
                self._as.set_aborted(self._result)
            else:
                self._as.set_succeeded(self._result)
        else:
            iiwa_joint_spline_goal = MoveAlongJointSplineGoal()
            iiwa_joint_spline_goal.spline.segments = []

        
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
                
                if i == len(points_points)-1:
                    v = points_points[i-1].velocities
                elif i == 0:
                    v = points_points[1].velocities
                else:
                    v = points_points[i].velocities
                
                segment.velocity = JointQuantity()
                segment.velocity.a1 = v[0]*self.VELOCITY_MULTIPLIER
                segment.velocity.a2 = v[1]*self.VELOCITY_MULTIPLIER
                segment.velocity.a3 = v[2]*self.VELOCITY_MULTIPLIER
                segment.velocity.a4 = v[3]*self.VELOCITY_MULTIPLIER
                segment.velocity.a5 = v[4]*self.VELOCITY_MULTIPLIER
                segment.velocity.a6 = v[5]*self.VELOCITY_MULTIPLIER
                segment.velocity.a7 = v[6]*self.VELOCITY_MULTIPLIER
                iiwa_joint_spline_goal.spline.segments.append(segment)

            print('sending goal')
            self.iiwa_joint_action_client.send_goal_and_wait(iiwa_joint_spline_goal)


            

            
        


if __name__ == '__main__':
    rospy.init_node('action_server')
    server = followJointTrajectoryActionServer('arm_controller/follow_joint_trajectory')
    rospy.spin()
    