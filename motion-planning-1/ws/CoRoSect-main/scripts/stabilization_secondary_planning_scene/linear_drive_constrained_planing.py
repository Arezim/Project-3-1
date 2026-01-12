#!/usr/bin/env python3

import rospy

#from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
import moveit_commander

from moveit_msgs.msg import OrientationConstraint, Constraints

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from corosect_msgs.srv import GetConstrainedMotionPoint, GetConstrainedMotionPointRequest, GetConstrainedMotionPointResponse

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations

import numpy as np
from threading import Lock

import time


CONSTRAINT = np.radians(20)  # Won't allow the drive to be angled in pitch or roll more than this value


class DriveConstraintPlanner():


    def __init__(self) -> None:
        moveit_commander.roscpp_initialize("constraint_simulator")
        rospy.init_node("constraint_simulator", anonymous=False)
        rospy.Service('get_constrained_motion_point', GetConstrainedMotionPoint, self.get_constrained_motion_point)

        arm_group = "arm"
        drive_rail_group = 'arm_guide_rail'

        # scene = moveit_commander.PlanningSceneInterface() 
        self.robot = moveit_commander.RobotCommander() 

        self.move_group_arm = moveit_commander.MoveGroupCommander(arm_group)
        self.move_group_rail = moveit_commander.MoveGroupCommander(drive_rail_group)

        self.move_group_arm.set_max_acceleration_scaling_factor(1.0)
        self.move_group_arm.set_max_velocity_scaling_factor(1.0)
        self.move_group_arm.limit_max_cartesian_link_speed(1000)
        self.move_group_rail.set_max_acceleration_scaling_factor(1.0)
        self.move_group_rail.set_max_velocity_scaling_factor(1.0)
        self.move_group_rail.limit_max_cartesian_link_speed(1000)
        self.service_roscall_lock = Lock()

        rospy.spin()


    def get_check_current_pose(self):
        success = False
        current_pose = self.move_group_arm.get_current_pose(end_effector_link='m_robot_linear_drive_guiderail')
        roll, pitch, yaw = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,
                                current_pose.pose.orientation.w])
        if np.pi - np.abs(roll) < CONSTRAINT and np.abs(pitch) < CONSTRAINT:
            success = True
        else:
            success = False
        
        return success, current_pose, [roll, pitch, yaw]


    def get_constrained_motion_point(self, req: GetConstrainedMotionPointRequest):
        self.service_roscall_lock.acquire()
        now = time.time()
        response = GetConstrainedMotionPointResponse()
        try:
            self.move_group_arm.set_named_target("workspace")
            self.move_group_arm.go(wait=True)

            self.move_group_arm.set_pose_target(req.target)
            plan = self.move_group_arm.go(wait=True)

            is_valid_pose, current_pose, euler_orientation = self.get_check_current_pose()

            if not is_valid_pose:
                i = 0
                while not is_valid_pose and i < 10:
                    plan = True
                    print('Iteration', i)
                    target_rail_rotation = quaternion_from_euler(-np.pi, 0.0, euler_orientation[2])
                    rail_position = PoseStamped()
                    rail_position.header.frame_id = current_pose.header.frame_id
                    rail_position.pose.position.x = current_pose.pose.position.x
                    rail_position.pose.position.y = current_pose.pose.position.y
                    rail_position.pose.position.z = current_pose.pose.position.z
                    rail_position.pose.orientation.x = target_rail_rotation[0]
                    rail_position.pose.orientation.y = target_rail_rotation[1]
                    rail_position.pose.orientation.z = target_rail_rotation[2]
                    rail_position.pose.orientation.w = target_rail_rotation[3]

                    print('adjusting rail')
                    self.move_group_rail.set_pose_target(rail_position)
                    self.move_group_rail.go(wait=True)

                    print('completing goal')
                    self.move_group_arm.set_pose_target(req.target)
                    plan = self.move_group_arm.go(wait=True)

                    is_valid_pose, current_pose, euler_orientation = self.get_check_current_pose()
                    i += 1
        
            if is_valid_pose and plan:
                target_joint_position = self.robot.get_current_state().joint_state.position[:10]

                response.success = True
                response.joint_states.position = np.array(target_joint_position)
            else:
                response.success = False
        
        except Exception as e:
            print(e)  # TODO corosect exception

        finally: 
            self.service_roscall_lock.release()
            rospy.logerr('PLANNING TOOK %.2f', (time.time() - now))
            rospy.logerr('PLANNING TOOK %.2f', (time.time() - now))
            rospy.logerr('PLANNING TOOK %.2f', (time.time() - now))
            rospy.logerr('PLANNING TOOK %.2f', (time.time() - now))
            return response


if __name__=='__main__':
    DriveConstraintPlanner()
