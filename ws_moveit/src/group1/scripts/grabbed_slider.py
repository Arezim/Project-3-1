#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SO
import math
from copy import deepcopy

import sys
import time
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from iiwa_msgs.msg import MoveAlongJointSplineActionResult
import moveit_msgs.msg
import geometry_msgs.msg


def make_pose(x, y, z, q=None, frame="base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame

    # default: gripper DOWN (180° rotation around X)
    if q is None:
        q = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.x = q["x"]
    pose.pose.orientation.y = q["y"]
    pose.pose.orientation.z = q["z"]
    pose.pose.orientation.w = q["w"]

    return pose



def distance(pose_a, pose_b):
    dx = pose_a.position.x - pose_b.position.x
    dy = pose_a.position.y - pose_b.position.y
    dz = pose_a.position.z - pose_b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def extract_plan(plan_result):
    """Handle both MoveIt return styles and give (success, RobotTrajectory)."""
    if isinstance(plan_result, tuple):
        success_flag = bool(plan_result[0])
        traj = None
        for item in plan_result:
            if isinstance(item, RobotTrajectory):
                traj = item
                break
        return success_flag and traj is not None, traj

    traj = plan_result
    print("plan_result not a tuple:")
    success_flag = bool(
        traj
        and getattr(traj, "joint_trajectory", None)
        and traj.joint_trajectory.points
    )
    return success_flag, traj


def slice_trajectory(traj, max_time):
    """Return a new RobotTrajectory truncated to max_time seconds."""
    jt = traj.joint_trajectory
    if not jt.points:
        return traj

    # calculate the total duration of the trajectory
    total_duration = jt.points[-1].time_from_start.to_sec()
    if max_time >= total_duration:
        # nothing to slice, just return the original
        return traj

    # build new trajectory with points up to max_time
    sliced_points = [
        pt for pt in jt.points if pt.time_from_start.to_sec() <= max_time + 1e-6
    ]
    if not sliced_points:
        sliced_points = [jt.points[0]]

    new_traj = RobotTrajectory()
    new_traj.joint_trajectory = JointTrajectory()
    new_traj.joint_trajectory.joint_names = list(jt.joint_names)
    new_traj.joint_trajectory.points = sliced_points
    return new_traj

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        rospy.sleep(0.4)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(0.4)
        rospy.init_node("grabber", anonymous=False)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_scene_diff_publisher = rospy.Publisher("planning_scene", moveit_msgs.msg.PlanningScene, queue_size=1)
            # time-slicing parameters
        slice_sec = float(rospy.get_param("~slice_sec", 0.5))
        goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.03))

        vel_scale = float(rospy.get_param("~vel_scale", 0.1))
        acc_scale = float(rospy.get_param("~acc_scale", 0.1))

        # targets (default: same A/B as marker_points.py & experiment_runner.py)
        A = rospy.get_param("~target_A", {"x": -0.5, "y": -0.80, "z": 1.4})
        B = rospy.get_param("~target_B", {"x": 0.0, "y": -0.30, "z": 0.9})

        DOWN_Q = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}  # gripper down

        frame_id = rospy.get_param("~frame_id", "base_link")

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm")
        eef_link = group.get_end_effector_link()
        group.set_pose_reference_frame(frame_id)
        group.set_max_velocity_scaling_factor(vel_scale)
        group.set_max_acceleration_scaling_factor(acc_scale)
        group.set_planning_time(10)
        group.set_num_planning_attempts(5)
        rospy.sleep(0.1)
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = group
        self.eef_link = eef_link

    #Function source: https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py # Author: Acorn Pooley, Mike Lautman
    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "rg6_link_0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.2 # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.4, 0.05, 0.01))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


        #Function source: https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py # Author: Acorn Pooley, Mike Lautman
    def attach_box(self, timeout=4):
            # Copy class variables to local variables to make the web tutorials more clear.
            # In practice, you should use the class variables directly unless you have a good
            # reason not to.
            box_name = self.box_name
            robot = self.robot
            scene = self.scene
            eef_link = self.eef_link

            ## BEGIN_SUB_TUTORIAL attach_object
            ##
            ## Attaching Objects to the Robot
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
            ## robot be able to touch them without the planning scene reporting the contact as a
            ## collision. By adding link names to the ``touch_links`` array, we are telling the
            ## planning scene to ignore collisions between those links and the box. For the Panda
            ## robot, we set ``grasping_group = 'panda_hand'``. If you are using a different robot,
            ## you should change this value to the name of your end effector group name.
            grasping_group = "soft_rg6"
            touch_links = robot.get_link_names(group=grasping_group)
            scene.attach_box(eef_link, box_name, touch_links=touch_links)
            ## END_SUB_TUTORIAL

            # We wait for the planning scene to update.
            return self.wait_for_state_update(
                box_is_attached=True, box_is_known=False, timeout=timeout
            )
    
    #Function source: https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py # Author: Acorn Pooley, Mike Lautman
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

def main():
    try:
        interface = MoveGroupPythonInterfaceTutorial()

        interface.add_box()
        interface.attach_box()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
        main()

