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
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import moveit_msgs.msg
import geometry_msgs.msg
import threading


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

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_scene_diff_publisher = rospy.Publisher(
            "planning_scene", moveit_msgs.msg.PlanningScene, queue_size=1
        )

        # -----------------------------
        # HARD-CODED CONFIGURATION
        # -----------------------------
        # Visual-servoing node publishes grasp offset [m] on this topic:
        self.grasp_offset_topic = "/group4/graspOffset"  # std_msgs/Float64

        # Visual-servoing (or gripper logic) publishes whether the slider is currently grabbed:
        self.slider_grabbed_topic = "/group4/isSliderGrabbed"  # std_msgs/Bool

        # Axis of allowed offset in the gripper frame: 'x'/'y'/'z'
        self.grasp_axis = "x"

        # Frame in which the box pose is specified when adding to the scene
        self.gripper_frame = "rg6_link_0"

        # Name of the collision object
        self.box_name = "slider"

        # Box size in meters: [x, y, z]
        self.profile_size = [0.782, 0.056, 0.025]

        # Constant offset of the box w.r.t. gripper frame
        self.box_static_offset = {"x": 0.0, "y": 0.0, "z": 0.2}

        # MoveIt group scaling
        vel_scale = 0.1
        acc_scale = 0.1

        # targets (kept for compatibility; not used here)
        A = {"x": -0.5, "y": -0.80, "z": 1.4}
        B = {"x": 0.0, "y": -0.30, "z": 0.9}

        DOWN_Q = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}  # gripper down

        frame_id = "base_link"

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
        self.robot = robot
        self.scene = scene
        self.move_group = group
        self.eef_link = eef_link

        # Latest incoming state from topics
        self._lock = threading.Lock()
        self._latest_grabbed = False
        self._latest_offset = 0.0
        self._is_attached = False

        rospy.Subscriber(self.slider_grabbed_topic, Bool, self._on_slider_grabbed, queue_size=1)
        rospy.Subscriber(self.grasp_offset_topic, Float64, self._on_grasp_offset, queue_size=1)

    def _on_slider_grabbed(self, msg):
        grabbed = bool(msg.data)
        with self._lock:
            prev = self._latest_grabbed
            self._latest_grabbed = grabbed

        # Only act on edges
        if grabbed == prev:
            return

        if grabbed:
            self._attach_with_latest_offset()
        else:
            self.detach_and_remove_box()

    def _on_grasp_offset(self, msg):
        with self._lock:
            self._latest_offset = float(msg.data)

    def _attach_with_latest_offset(self):
        with self._lock:
            if self._is_attached:
                return
            offset = self._latest_offset

        self.add_box(grasp_offset=offset)
        self.attach_box()

        with self._lock:
            self._is_attached = True

    def _clamp_grasp_offset(self, grasp_offset):
        axis = self.grasp_axis
        if axis not in ("x", "y", "z"):
            rospy.logwarn("Invalid grasp_axis '%s' (expected x/y/z); using 'x'", axis)
            axis = "x"

        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        half_len = 0.5 * float(self.profile_size[axis_index])
        return max(-half_len, min(half_len, float(grasp_offset)))

    #Function source: https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py # Author: Acorn Pooley, Mike Lautman
    def add_box(self, grasp_offset=None):
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
        box_pose.header.frame_id = self.gripper_frame
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = self.box_static_offset["x"]
        box_pose.pose.position.y = self.box_static_offset["y"]
        box_pose.pose.position.z = self.box_static_offset["z"]

        if grasp_offset is not None:
            clamped = self._clamp_grasp_offset(grasp_offset)
            if self.grasp_axis == "x":
                box_pose.pose.position.x += clamped
            elif self.grasp_axis == "y":
                box_pose.pose.position.y += clamped
            else:
                box_pose.pose.position.z += clamped

        scene.add_box(box_name, box_pose, size=tuple(self.profile_size))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return True


        #Function source: https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py # Author: Acorn Pooley, Mike Lautman
    def attach_box(self):
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

        return True

    def detach_and_remove_box(self):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        # Detach (if attached)
        try:
            scene.remove_attached_object(eef_link, name=box_name)
        except TypeError:
            # Some MoveIt versions use positional args
            scene.remove_attached_object(eef_link, box_name)

        # Remove from world (if present)
        scene.remove_world_object(box_name)
        with self._lock:
            self._is_attached = False

        return True

def main():
    try:
        MoveGroupPythonInterfaceTutorial()
        rospy.loginfo("Listening on %s (Bool) and %s (Float64)", "/group4/isSliderGrabbed", "/group4/graspOffset")
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()

