#!/usr/bin/env python3
import os
import csv
import time
import math

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

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

    total_duration = jt.points[-1].time_from_start.to_sec()
    if max_time >= total_duration:
        return traj

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


def ensure_csv_path(csv_path):
    parent = os.path.dirname(csv_path)
    if not parent:
        return csv_path
    try:
        os.makedirs(parent, exist_ok=True)
        return csv_path
    except Exception as e:
        fallback = "/tmp/dyn_replanner_log.csv"
        rospy.logwarn(
            "replanner_experiment: cannot create %s (%s), falling back to %s",
            parent,
            str(e),
            fallback,
        )
        return fallback

def main():
    rospy.init_node("replanner_experiment", anonymous=False)
    moveit_commander.roscpp_initialize([])

    group_name     = rospy.get_param("~group_name", "manipulator")
    frame_id       = rospy.get_param("~frame_id", "base_link")
    slice_sec      = float(rospy.get_param("~slice_sec", 1.4))
    goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.03))
    vel_scale      = float(rospy.get_param("~vel_scale", 0.25))
    acc_scale      = float(rospy.get_param("~acc_scale", 0.25))

    # planner list and how many goal reaches per planner
    default_planner = rospy.get_param("~planner_id", "RRTConnectkConfigDefault")
    planner_ids = rospy.get_param("~planner_ids", [default_planner])
    episodes_per_planner = int(rospy.get_param("~episodes_per_planner", 4))

    csv_path_param = rospy.get_param(
        "~csv_path",
        "/root/dyn_ws/temp_csv/dyn_replanner_log.csv",
    )
    csv_path = ensure_csv_path(csv_path_param)

    # targets (same as marker_points)
    A = rospy.get_param("~target_A", {"x": 0.60, "y": 0.10, "z": 0.95})
    B = rospy.get_param("~target_B", {"x": -0.60, "y": -0.50, "z": 1.50})
    targets = [
        ("A", make_pose(A["x"], A["y"], A["z"], frame=frame_id)),
        ("B", make_pose(B["x"], B["y"], B["z"], frame=frame_id)),
    ]

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_pose_reference_frame(frame_id)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)

    planner_index = 0
    current_planner = planner_ids[planner_index]
    group.set_planner_id(current_planner)

    rospy.loginfo("replanner_experiment: group='%s', frame='%s'", group_name, frame_id)
    rospy.loginfo(
        "replanner_experiment: slice_sec=%.3f, goal_tol=%.3f, episodes_per_planner=%d",
        slice_sec,
        goal_tolerance,
        episodes_per_planner,
    )
    rospy.loginfo(
        "replanner_experiment: planners in test: %s",
        ", ".join(planner_ids),
    )
    rospy.loginfo(
        "replanner_experiment: using first planner '%s'",
        current_planner,
    )
    rospy.loginfo(
        "replanner_experiment: targets: A=(%.2f, %.2f, %.2f), B=(%.2f, %.2f, %.2f)",
        A["x"], A["y"], A["z"],
        B["x"], B["y"], B["z"],
    )

    # CSV
    file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
    f = open(csv_path, "a" if file_exists else "w", newline="")
    writer = csv.writer(f)

    if not file_exists:
        rospy.loginfo("replanner_experiment: creating new CSV at %s", csv_path)
        writer.writerow([
            "timestamp",
            "planner_id",
            "planner_index",
            "episode_id",
            "episode_in_planner",
            "slice_index",
            "from_label",
            "to_label",
            "target_label",
            "episode_time_s",
            "distance_before",
            "distance_after",
            "goal_tolerance",
            "goal_reached_after_slice",      # 0 or 1
            "plan_success",                  # 0 or 1
            "exec_success",                  # 0 or 1
            "blocked_flag",                  # 1 if planning failed
            "planning_time_s",
            "traj_full_duration_s",
            "slice_duration_s",
            "num_traj_points_full",
            "num_traj_points_slice",
            "blocked_slices_so_far_in_episode",
            "total_planned_time_so_far_in_episode",
        ])
        f.flush()

    rate = rospy.Rate(1.0)

    episode_id = 0
    episode_in_planner = 0
    target_index = 0
    from_label = "START"
    slice_index = 0
    episode_start = rospy.Time.now().to_sec()
    blocked_slices_in_episode = 0
    total_planned_time_in_episode = 0.0

    try:
        while not rospy.is_shutdown():
            label, goal_pose = targets[target_index]
            to_label = label

            robot_state = robot.get_current_state()
            group.set_start_state(robot_state)

            current_pose_stamped = group.get_current_pose()
            current_pose = current_pose_stamped.pose
            dist_before = distance(current_pose, goal_pose.pose)
            episode_time = rospy.Time.now().to_sec() - episode_start

            rospy.loginfo(
                "replanner_experiment: planner=%s ep=%d slice=%d target=[%s] dist_before=%.3f",
                current_planner,
                episode_id,
                slice_index,
                label,
                dist_before,
            )

            # planning
            planning_start = time.time()
            group.set_pose_target(goal_pose)
            plan_result = group.plan()
            planning_time = time.time() - planning_start
            group.clear_pose_targets()

            plan_success, traj = extract_plan(plan_result)

            exec_success = False
            dist_after = dist_before
            full_duration = 0.0
            slice_duration = 0.0
            n_full = 0
            n_slice = 0

            # if planning succeeded, execute sliced trajectory
            if plan_success and traj and traj.joint_trajectory.points:
                jt_full = traj.joint_trajectory
                n_full = len(jt_full.points)
                full_duration = jt_full.points[-1].time_from_start.to_sec()

                slice_time = min(slice_sec, full_duration)
                sliced_traj = slice_trajectory(traj, slice_time)
                jt_slice = sliced_traj.joint_trajectory
                if jt_slice.points:
                    n_slice = len(jt_slice.points)
                    slice_duration = jt_slice.points[-1].time_from_start.to_sec()

                    rospy.loginfo(
                        "replanner_experiment: executing SLICE planner=%s ep=%d slice=%d "
                        "(slice_dur=%.3f, full_dur=%.3f)",
                        current_planner,
                        episode_id,
                        slice_index,
                        slice_duration,
                        full_duration,
                    )
                    exec_success = group.execute(sliced_traj, wait=True)

                    if exec_success:
                        new_pose = group.get_current_pose().pose
                        dist_after = distance(new_pose, goal_pose.pose)
                else:
                    rospy.logwarn(
                        "replanner_experiment: sliced trajectory empty, skipping execution",
                    )
            else:
                rospy.logwarn(
                    "replanner_experiment: planning failed for target [%s] (ep=%d slice=%d planner=%s)",
                    label,
                    episode_id,
                    slice_index,
                    current_planner,
                )

            blocked_flag = 1 if not plan_success else 0
            if blocked_flag:
                blocked_slices_in_episode += 1
            total_planned_time_in_episode += slice_duration

            goal_reached_after_slice = 1 if dist_after <= goal_tolerance else 0

            # csv stuff
            writer.writerow([
                rospy.Time.now().to_sec(),
                current_planner,
                planner_index,
                episode_id,
                episode_in_planner,
                slice_index,
                from_label,
                to_label,
                label,
                episode_time,
                dist_before,
                dist_after,
                goal_tolerance,
                goal_reached_after_slice,
                1 if plan_success else 0,
                1 if exec_success else 0,
                blocked_flag,
                planning_time,
                full_duration,
                slice_duration,
                n_full,
                n_slice,
                blocked_slices_in_episode,
                total_planned_time_in_episode,
            ])
            f.flush()

            if goal_reached_after_slice:
                rospy.loginfo(
                    "replanner_experiment: planner=%s reached [%s] within tol %.3f, "
                    "ending episode %d (episode_in_planner=%d)",
                    current_planner,
                    label,
                    goal_tolerance,
                    episode_id,
                    episode_in_planner,
                )

                from_label = label
                target_index = (target_index + 1) % len(targets)

                episode_id += 1
                episode_in_planner += 1
                slice_index = 0
                episode_start = rospy.Time.now().to_sec()
                blocked_slices_in_episode = 0
                total_planned_time_in_episode = 0.0

                # switch planner after enough episodes for this one
                if episode_in_planner >= episodes_per_planner:
                    planner_index = (planner_index + 1) % len(planner_ids)
                    current_planner = planner_ids[planner_index]
                    group.set_planner_id(current_planner)
                    rospy.loginfo(
                        "replanner_experiment: switching to new planner '%s' (index=%d)",
                        current_planner,
                        planner_index,
                    )
                    episode_in_planner = 0
            else:
                slice_index += 1

            rate.sleep()

    finally:
        f.close()
        rospy.loginfo("replanner_experiment: CSV closed (%s)", csv_path)
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
