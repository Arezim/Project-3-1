#!/usr/bin/env python3
import os
import csv
import math
import rospy
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize

# ---------- helpers ----------

def make_pose(x, y, z, frame="base_link"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.w = 1.0
    return p

def interpolate_pose(start, target, alpha):
    """
    alpha = 1.0 -> target, alpha = 0.0 -> start
    (same convention as your move_to_points_v1)
    """
    pose = PoseStamped()
    pose.header.frame_id = start.header.frame_id
    pose.pose.orientation = deepcopy(target.pose.orientation)

    pose.pose.position.x = start.pose.position.x * (1 - alpha) + target.pose.position.x * alpha
    pose.pose.position.y = start.pose.position.y * (1 - alpha) + target.pose.position.y * alpha
    pose.pose.position.z = start.pose.position.z * (1 - alpha) + target.pose.position.z * alpha
    return pose

def try_plan(group, pose):
    group.set_start_state_to_current_state()
    group.set_pose_target(pose)

    start_wall = time.time()
    plan = group.plan()
    planning_time = time.time() - start_wall

    # MoveIt Noetic can return:
    #  - RobotTrajectory
    #  - (success_flag, RobotTrajectory)
    if isinstance(plan, tuple):
        success_flag = bool(plan[0])
        plan_obj = plan[1] if len(plan) > 1 else None
    else:
        plan_obj = plan
        success_flag = bool(
            plan_obj
            and getattr(plan_obj, "joint_trajectory", None)
            and plan_obj.joint_trajectory.points
        )

    group.clear_pose_targets()
    return success_flag, plan_obj, planning_time

def _ensure_csv_path(csv_path):
    parent = os.path.dirname(csv_path)
    if not parent:
        return csv_path
    try:
        os.makedirs(parent, exist_ok=True)
        return csv_path
    except Exception as e:
        fallback = "/tmp/interp_experiment.csv"
        rospy.logwarn("experiment_runner: cannot create %s (%s), falling back to %s",
                      parent, str(e), fallback)
        return fallback

# ---------- main experiment ----------

def main():
    rospy.init_node("experiment_runner", anonymous=False)
    roscpp_initialize([])

    # single-planner fallback param (for compatibility)
    default_planner = rospy.get_param("~planner_id", "RRTConnectkConfigDefault")
    # list of planners to test, in order
    planner_ids = rospy.get_param("~planner_ids", [default_planner])

    csv_path   = rospy.get_param("~csv_path", "/root/dyn_ws/interp_experiment.csv")
    loop_rate  = rospy.get_param("~loop_rate_hz", 0.05)   # ~1 cycle / 20s
    reach_frame = rospy.get_param("~frame_id", "base_link")
    # IMPORTANT: max_trials = attempts PER PLANNER
    max_trials = rospy.get_param("~max_trials", 15)

    group = MoveGroupCommander("manipulator")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.25)
    group.set_max_acceleration_scaling_factor(0.25)

    rospy.loginfo("experiment_runner: planner_ids = %s", ", ".join(planner_ids))
    rospy.loginfo("experiment_runner: max_trials per planner = %d", max_trials)

    # ---- target set: easy + spicy but reachable-ish ----
    targets = {
        "A": make_pose(0.60,  0.10, 0.95, frame=reach_frame),
        "B": make_pose(-0.60, -0.50, 1.50, frame=reach_frame),
        "TOP_DIAG": make_pose(0.45, 0.45, 1.80, frame=reach_frame),
        "LOW_BACK": make_pose(-0.20, -0.80, 0.65, frame=reach_frame),
        "SIDE_SWEEP": make_pose(0.80, -0.15, 1.10, frame=reach_frame),
    }
    labels = list(targets.keys())

    # alpha schedule
    alphas = [1.0, 0.9, 0.8, 0.7, 0.55, 0.4]

    # ---------- CSV setup (APPEND if exists) ----------
    csv_path = _ensure_csv_path(csv_path)
    file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0

    mode = "a" if file_exists else "w"
    f = open(csv_path, mode, newline="")
    writer = csv.writer(f)

    if not file_exists:
        rospy.loginfo("experiment_runner: creating new CSV with header at %s", csv_path)
        writer.writerow([
            "timestamp",
            "trial_id",
            "planner_id",
            "target_label",
            "target_x", "target_y", "target_z",
            "start_x", "start_y", "start_z",

            "success",                   # 0/1
            "success_alpha",             # -1 if failure
            "success_alpha_index",       # index in alphas list, -1 if failure

            "num_alpha_attempts",        # how many alphas we actually tried
            "num_failed_alphas",         # how many of those failed
            "num_interpolation_steps",   # attempts - 1 (0 = direct, >0 = interpolated)

            "alphas_tried",              # "1.00;0.90;0.80"
            "alphas_failed",             # e.g. "1.00;0.90"

            "planning_time_success_s",   # planning time of the successful attempt (0 if none)
            "planning_time_total_s",     # sum of planning times over all attempts

            "num_traj_points",
            "traj_duration_s"
        ])
        f.flush()
        global_trial_id_start = 0
    else:
        rospy.loginfo("experiment_runner: appending to existing CSV %s", csv_path)
        # infer last trial_id from line count (minus header)
        try:
            with open(csv_path, "r") as rf:
                line_count = sum(1 for _ in rf)
            global_trial_id_start = max(line_count - 1, 0)
            rospy.loginfo("experiment_runner: continuing trial_id from %d", global_trial_id_start)
        except Exception as e:
            rospy.logwarn("experiment_runner: could not infer trial_id (%s), starting from 0", str(e))
            global_trial_id_start = 0

    rate = rospy.Rate(loop_rate)
    global_trial_id = global_trial_id_start

    try:
        # ===== outer loop over planners =====
        for planner_id in planner_ids:
            if rospy.is_shutdown():
                break

            rospy.loginfo("experiment_runner: === Starting planner %s ===", planner_id)
            group.set_planner_id(planner_id)

            # per-planner stats in memory (for console)
            stats = {
                name: {
                    "visits": 0,
                    "direct_success": 0,
                    "interp_success": 0,
                    "failures": 0,
                    "sum_success_alpha": 0.0,
                }
                for name in labels
            }

            label_index = 0
            planner_trial_count = 0  # how many attempts we've done for THIS planner

            while (not rospy.is_shutdown()) and (planner_trial_count < max_trials):
                label = labels[label_index]
                target_pose = targets[label]
                stats[label]["visits"] += 1

                # current pose = base for interpolation
                current_pose = group.get_current_pose().pose
                start_x = current_pose.position.x
                start_y = current_pose.position.y
                start_z = current_pose.position.z

                # per-trial stats
                success = False
                success_alpha = -1.0
                success_alpha_index = -1
                planning_time_success = 0.0
                planning_time_total = 0.0
                num_pts = 0
                traj_dur = 0.0

                tried_alphas = []
                failed_alphas = []

                current_trial_idx = planner_trial_count + 1
                rospy.loginfo("experiment_runner: ===== [%s] planner=%s (trial %d/%d) =====",
                              label, planner_id, current_trial_idx, max_trials)

                # try alphas in order
                for idx, alpha in enumerate(alphas):
                    tried_alphas.append(alpha)

                    cand = interpolate_pose(
                        make_pose(start_x, start_y, start_z, frame=reach_frame),
                        target_pose,
                        alpha,
                    )

                    rospy.loginfo(
                        "experiment_runner: trying %s (planner=%s) with alpha=%.2f (%.0f%% distance)",
                        label, planner_id, alpha, alpha * 100.0
                    )

                    ok, plan_obj, pt = try_plan(group, cand)
                    planning_time_total += pt

                    if ok and plan_obj and getattr(plan_obj, "joint_trajectory", None):
                        pts = plan_obj.joint_trajectory.points
                        num_pts = len(pts)
                        if num_pts > 0:
                            traj_dur = pts[-1].time_from_start.to_sec()

                        success = True
                        success_alpha = alpha
                        success_alpha_index = idx
                        planning_time_success = pt

                        rospy.loginfo("experiment_runner: SUCCESS %s (planner=%s) with alpha=%.2f",
                                      label, planner_id, alpha)
                        break
                    else:
                        failed_alphas.append(alpha)
                        rospy.logwarn("experiment_runner: FAIL %s (planner=%s) with alpha=%.2f",
                                      label, planner_id, alpha)

                num_alpha_attempts = len(tried_alphas)
                num_failed_alphas = len(failed_alphas)
                num_interpolation_steps = max(0, num_alpha_attempts - 1)

                # update per-target stats (for console)
                s = stats[label]
                if success:
                    s["sum_success_alpha"] += success_alpha
                    if success_alpha_index == 0:
                        s["direct_success"] += 1
                    else:
                        s["interp_success"] += 1
                else:
                    s["failures"] += 1
                    rospy.logerr("experiment_runner: all alphas failed for target %s (planner=%s)",
                                 label, planner_id)

                total_success = s["direct_success"] + s["interp_success"]
                avg_alpha = (s["sum_success_alpha"] / float(total_success)) if total_success > 0 else float("nan")
                rospy.loginfo(
                    "experiment_runner: STATS [%s] planner=%s visits=%d direct=%d interp=%d fail=%d avg_alpha=%.2f",
                    label, planner_id,
                    s["visits"], s["direct_success"], s["interp_success"], s["failures"], avg_alpha,
                )

                # write a CSV row for this attempt (global trial index)
                global_trial_id += 1
                planner_trial_count += 1

                writer.writerow([
                    rospy.Time.now().to_sec(),
                    global_trial_id,
                    planner_id,
                    label,
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                    start_x, start_y, start_z,

                    1 if success else 0,
                    success_alpha,
                    success_alpha_index,

                    num_alpha_attempts,
                    num_failed_alphas,
                    num_interpolation_steps,

                    ";".join(f"{a:.2f}" for a in tried_alphas),
                    ";".join(f"{a:.2f}" for a in failed_alphas),

                    planning_time_success,
                    planning_time_total,

                    num_pts,
                    traj_dur,
                ])
                f.flush()

                # after a full sweep over labels, print per-planner summary
                if label_index == len(labels) - 1:
                    rospy.loginfo("experiment_runner: ===== PLANNER SUMMARY (planner=%s so far) =====", planner_id)
                    for name in labels:
                        s2 = stats[name]
                        tot2 = s2["direct_success"] + s2["interp_success"]
                        avg2 = (s2["sum_success_alpha"] / float(tot2)) if tot2 > 0 else float("nan")
                        rospy.loginfo(
                            "  [%s] visits=%d direct=%d interp=%d fail=%d avg_alpha=%.2f",
                            name,
                            s2["visits"], s2["direct_success"],
                            s2["interp_success"], s2["failures"], avg2,
                        )

                # next target (round-robin)
                label_index = (label_index + 1) % len(labels)
                rate.sleep()

            rospy.loginfo("experiment_runner: === Finished planner %s ===", planner_id)

    finally:
        f.close()
        rospy.loginfo("experiment_runner: CSV file closed (%s)", csv_path)

if __name__ == "__main__":
    main()
