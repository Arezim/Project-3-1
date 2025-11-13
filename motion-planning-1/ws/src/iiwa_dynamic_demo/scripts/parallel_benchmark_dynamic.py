#!/usr/bin/env python3
import os
import csv
import time
import math
import threading

import rospy
from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
)
import geometry_msgs.msg

# Planners to benchmark
PLANNERS = [
    "RRTConnectkConfigDefault",
    "PRMkConfigDefault",
    "RRTstarkConfigDefault",  # RRT*
]

# Number of runs and time limit per planner (replanning)
RUNS = 100          # adjust if needed
TIME_LIMIT = 5.0    # seconds for replan

# Where to store CSV results
RESULTS_DIR = os.path.expanduser("~/benchmarks")


def joint_path_length(pts):
    """Compute Euclidean joint-space path length for a trajectory."""
    if len(pts) < 2:
        return 0.0
    L = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        pa, pb = a.positions, b.positions
        L += math.sqrt(sum((x - y) ** 2 for x, y in zip(pa, pb)))
    return L


def wait_for_param(name, timeout=30.0):
    """Wait up to `timeout` seconds for a ROS param to appear."""
    t0 = time.time()
    while not rospy.is_shutdown():
        if rospy.has_param(name):
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.1)


def call_plan(group):
    """Call group.plan() and return (ok, traj, tsec), handling tuple or non-tuple."""
    t0 = time.time()
    plan = group.plan()
    dt = time.time() - t0

    ok, traj = False, None
    try:
        # Some MoveIt versions return (success, traj, ...) tuples
        ok = bool(plan[0])
        traj = plan[1] if ok else None
    except Exception:
        # Others return only the trajectory object
        ok = bool(plan)
        traj = plan if ok else None

    return ok, traj, dt


def add_or_move_block(scene, frame, name, present, x=0.0, y=0.0, z=0.0, size=(0.2, 0.2, 0.2)):
    """
    If present=True, remove any old block with this name and add a box at (x,y,z).
    If present=False, just remove the block.
    """
    scene.remove_world_object(name)
    rospy.sleep(0.3)
    if not present:
        return

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    scene.add_box(name, pose, size)
    rospy.sleep(0.7)

    rospy.loginfo(
        f"Moving block '{name}' present at x={x:.2f}, y={y:.2f}, z={z:.2f}"
    )


def main():
    roscpp_initialize([])
    rospy.init_node("parallel_planning_benchmark_dynamic", anonymous=True)

    # Detect iiwa namespace and robot_description
    ns = ""
    desc = "robot_description"
    if rospy.has_param("/iiwa/robot_description"):
        ns = "/iiwa"
        desc = "/iiwa/robot_description"

    # Wait until MoveIt is up
    if not wait_for_param(desc, 30.0):
        raise RuntimeError(
            f"Timeout waiting for {desc}. Is the iiwa MoveIt demo running?"
        )

    robot = RobotCommander(robot_description=desc, ns=ns)
    groups = robot.get_group_names()
    group_name = "manipulator" if "manipulator" in groups else groups[0]

    # Planning scene for the moving obstacle
    scene = PlanningSceneInterface(ns=ns)
    rospy.sleep(1.0)
    world = robot.get_planning_frame()

    # Prepare output CSV
    os.makedirs(RESULTS_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_csv = os.path.join(
        RESULTS_DIR, f"bench_dynamic_{group_name}_{stamp}.csv"
    )

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        # We log only the REPLAN phase
        w.writerow(["run", "planner", "ok", "time_s", "points", "joint_path_len"])

        for r in range(1, RUNS + 1):
            print("=" * 60)
            print(f"[run {r:02d}] starting")

            # 1) Make sure the moving block is NOT in the scene (initial environment)
            add_or_move_block(scene, world, "moving_block", present=False)
            rospy.sleep(0.5)

            # 2) Sample a random goal and try an initial plan with RRTConnect (no obstacle)
            init_group = MoveGroupCommander(group_name, ns=ns)
            init_group.set_start_state_to_current_state()
            init_group.set_planner_id("RRTConnectkConfigDefault")
            init_group.set_planning_time(2.0)  # short initial plan
            init_group.set_num_planning_attempts(1)

            target = init_group.get_random_joint_values()
            init_group.set_joint_value_target(target)

            ok_init, traj_init, t_init = call_plan(init_group)
            if not ok_init:
                print(f"[run {r:02d}] initial plan FAILED (no obstacle), skipping")
                continue

            print(
                f"[run {r:02d}] initial plan ok with RRTConnect, time={t_init:.3f}s"
            )

            # 3) "Moving obstacle" appears: we add a block in the scene
            # Use the same coordinates you liked for the static box
            add_or_move_block(
                scene,
                world,
                "moving_block",
                present=True,
                x=0.40,
                y=0.00,
                z=1.30,
                size=(0.20, 0.20, 0.20),
            )

            # 4) Re-plan with different planners in parallel (same start & goal, new scene)
            results = {}
            threads = []

            def try_replan(pid):
                g = MoveGroupCommander(group_name, ns=ns)
                g.set_start_state_to_current_state()
                g.set_planner_id(pid)
                g.set_planning_time(TIME_LIMIT)
                g.set_num_planning_attempts(1)
                g.set_joint_value_target(target)

                ok, traj, dt = call_plan(g)
                npts, L = 0, 0.0
                if ok and hasattr(traj, "joint_trajectory"):
                    pts = traj.joint_trajectory.points
                    npts = len(pts)
                    L = joint_path_length(pts)

                results[pid] = {
                    "ok": ok,
                    "time": dt,
                    "points": npts,
                    "length": L,
                }
                print(
                    f"[run {r:02d}] REPLAN [{pid}] ok={ok} t={dt:.3f}s "
                    f"points={npts} L={L:.3f}"
                )

            # Spawn threads for each planner
            for pid in PLANNERS:
                t = threading.Thread(target=try_replan, args=(pid,))
                t.start()
                threads.append(t)

            for t in threads:
                t.join()

            # 5) Write per-planner replan rows
            for pid, res in results.items():
                w.writerow(
                    [
                        r,
                        pid,
                        int(res["ok"]),
                        f"{res['time']:.4f}",
                        res["points"],
                        f"{res['length']:.6f}",
                    ]
                )

            # 6) PRM + RRTConnect first-finish baseline (replan)
            ff_planners = ["PRMkConfigDefault", "RRTConnectkConfigDefault"]
            ff_times = [
                res["time"]
                for pid, res in results.items()
                if pid in ff_planners and res["ok"]
            ]

            if ff_times:
                ff_ok = 1
                ff_time = min(ff_times)
            else:
                ff_ok = 0
                ff_time = TIME_LIMIT

            w.writerow(
                [
                    r,
                    "FIRST_FINISH_PRM_RRTConnect",
                    ff_ok,
                    f"{ff_time:.4f}",
                    0,
                    "0.000000",
                ]
            )

            f.flush()

            # Optional debug: best single planner by time
            best_single = min(
                (res for res in results.values() if res["ok"]),
                key=lambda x: x["time"],
                default=None,
            )
            print(f"[run {r:02d}] best replan by time among singles: {best_single}")

        print("Saved:", out_csv)

    roscpp_shutdown()


if __name__ == "__main__":
    main()
