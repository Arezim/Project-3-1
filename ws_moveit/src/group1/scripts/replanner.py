#!/usr/bin/env python3
import math
from copy import deepcopy

import sys
import time
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from iiwa_msgs.msg import MoveAlongJointSplineActionResult
import moveit_msgs.msg


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

def wait_for_param(name, timeout=30.0):
    t0 = time.time()
    while not rospy.is_shutdown():
        if rospy.has_param(name):
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.1)

# class KukaExecutionWatcher:
#     def __init__(self):
#         self.last_success = True
#         self.received = True

#         rospy.Subscriber("/iiwa/action/move_along_joint_spline/result",MoveAlongJointSplineActionResult,self.callback)

#     def callback(self, msg):
#         # SUCCESS = status 3
#         self.last_success = (msg.status.status == 3 and msg.result.success)
#         self.received = True

#     def wait_for_success(self, timeout=30):
#         t0 = time.time()
#         self.received = False
#         while not rospy.is_shutdown():
#             if self.received:
#                 print("returning success")
#                 return self.last_success
#             if time.time() - t0 > timeout:
#                 print("returning false")
#                 return False
#             rospy.sleep(0.05)


def main():
    rospy.sleep(0.4)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.sleep(0.4)
    rospy.init_node("replanner", anonymous=False)
    ratunek = 0

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
    rospy.sleep(0.1)

    group = moveit_commander.MoveGroupCommander("arm")
    group.set_pose_reference_frame(frame_id)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)

    targets = [
        ("A", make_pose(A["x"], A["y"], A["z"], frame=frame_id)),
        ("B", make_pose(B["x"], B["y"], B["z"], frame=frame_id)),
    ]

    rospy.loginfo("replanner: using group '%s' in frame '%s'", "arm", frame_id)
    rospy.loginfo("replanner: slice_sec=%.3f, vel_scale=%.2f, acc_scale=%.2f",
                   slice_sec, vel_scale, acc_scale)
    rospy.loginfo("replanner: targets: A=(%.2f, %.2f, %.2f), B=(%.2f, %.2f, %.2f)",
                  A["x"], A["y"], A["z"], B["x"], B["y"], B["z"])

    rate = rospy.Rate(2.0)  # prevents bus y-loop when planning fails
    target_index = 0
   # watcher = KukaExecutionWatcher()
    first_movement = True


    while not rospy.is_shutdown():
        if ratunek = 0:
            targets = [
            ("A", make_pose(A["x"], A["y"], A["z"], frame=frame_id)),
            ("B", make_pose(B["x"], B["y"], B["z"], frame=frame_id))]
            ratunek = 1
        else:
            ratunek = 0
            targets = [
            ("B", make_pose(B["x"], B["y"], B["z"], frame=frame_id)),
            ("A", make_pose(A["x"], A["y"], A["z"], frame=frame_id))]
        label, goal_pose = targets[target_index]

        robot_state = robot.get_current_state()
        group.set_start_state(robot_state)

        # check if we're already basically at this goal
        current_pose = group.get_current_pose()
        dist = distance(current_pose.pose, goal_pose.pose)

        rospy.loginfo("replanner: current pose: (%.3f, %.3f, %.3f), goal [%s]: (%.3f, %.3f, %.3f), dist=%.3f",
                      current_pose.pose.position.x,
                      current_pose.pose.position.y,
                      current_pose.pose.position.z,
                      label,
                      goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      goal_pose.pose.position.z,
                      dist)

        if dist <= goal_tolerance:
            rospy.loginfo("replanner: reached goal [%s] within tolerance %.3f, switching targets",
                          label, goal_tolerance)
            target_index = (target_index + 1) % len(targets)
            print()
            rate.sleep()
            rospy.signal_shutdown()

        rospy.loginfo("replanner: planning slice towards [%s] (dist=%.3f)", label, dist)

        # if first_movement == True or watcher.wait_for_success(timeout=10):
        rospy.logwarn("succes planning")
        group.set_pose_target(goal_pose)
        group.set_start_state_to_current_state()
        plan_success, traj = extract_plan(group.plan())
        group.clear_pose_targets()
            # first_movement = False

        if not plan_success or traj is None:
            rospy.logwarn("replanner: planning to [%s] failed, will retry", label)
            rate.sleep()
            continue

        jt_full = traj.joint_trajectory
        if not jt_full.points:
            rospy.logwarn("replanner: trajectory has no points, skipping execution")
            rate.sleep()
            continue

        full_duration = jt_full.points[-1].time_from_start.to_sec()
        # choose how much of the trajectory we want to execute this cycle
        slice_time = min(slice_sec, full_duration)

        # build a sliced trajectory up to slice_time
        sliced_traj = slice_trajectory(traj, slice_time)
        jt = sliced_traj.joint_trajectory
        if not jt.points:
            rospy.logwarn("replanner: sliced trajectory has no points, skipping execution")     
            rate.sleep()
            continue

        slice_duration = jt.points[-1].time_from_start.to_sec()
        rospy.loginfo(
            "replanner: executing SLICE towards [%s] (slice_duration=%.3f s, full_traj_end=%.3f s)",
            label, slice_duration, full_duration
        )

        exec_ok = group.execute(sliced_traj, wait=True)
        # if (watcher.wait_for_success(timeout=10) == True):
        #     print("succes")
        # # rospy.sleep(0.5)
        if not exec_ok:
            rospy.logwarn("replanner: execute(slice) returned False, will replan next loop")
            rate.sleep()
            continue

        # group.stop()

        # Next loop iteration:
        #   - robot is at the new state
        #   - dynamic_obstacle.py has moved the box again
        #   - we plan another slice from *current* state with the updated planning scene
        rate.sleep()

    rospy.loginfo("replanner: shutting down")
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
        main()

