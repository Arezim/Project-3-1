#!/usr/bin/env python3
import math
from copy import deepcopy

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def make_pose(x, y, z, frame="base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
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


def main():
    rospy.init_node("replanner", anonymous=False)
    moveit_commander.roscpp_initialize([])

    group_name = rospy.get_param("~group_name", "manipulator")
    frame_id = rospy.get_param("~frame_id", "base_link")

    # time-slicing parameters
    slice_sec = float(rospy.get_param("~slice_sec", 1.4))
    goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.03))

    # speed scaling (for safe real-robot later)
    vel_scale = float(rospy.get_param("~vel_scale", 0.25))
    acc_scale = float(rospy.get_param("~acc_scale", 0.25))

    # targets (default: same A/B as marker_points.py & experiment_runner.py)
    A = rospy.get_param("~target_A", {"x": 0.60, "y": 0.10, "z": 0.95})
    B = rospy.get_param("~target_B", {"x": -0.60, "y": -0.50, "z": 1.50})

    targets = [
        ("A", make_pose(A["x"], A["y"], A["z"], frame=frame_id)),
        # ("B", make_pose(B["x"], B["y"], B["z"], frame=frame_id)),
    ]

    robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_pose_reference_frame(frame_id)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)

    rospy.loginfo("replanner: using group '%s' in frame '%s'", group_name, frame_id)
    rospy.loginfo("replanner: slice_sec=%.3f, vel_scale=%.2f, acc_scale=%.2f",
                  slice_sec, vel_scale, acc_scale)
    rospy.loginfo("replanner: targets: A=(%.2f, %.2f, %.2f), B=(%.2f, %.2f, %.2f)",
                  A["x"], A["y"], A["z"], B["x"], B["y"], B["z"])

    rate = rospy.Rate(2.0)  # prevents busy-loop when planning fails
    target_index = 0

    while not rospy.is_shutdown():
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
            rate.sleep()
            continue

        rospy.loginfo("replanner: planning slice towards [%s] (dist=%.3f)", label, dist)

        group.set_pose_target(goal_pose)
        plan_success, traj = extract_plan(group.plan())
        group.clear_pose_targets()

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
    try:
        main()
    except rospy.ROSInterruptException:
        pass