#!/usr/bin/env python3
import os, csv, time, math, threading
import rospy

from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
)
import geometry_msgs.msg

# Planners to benchmark (same as original)
PLANNERS = [
    "RRTConnectkConfigDefault",
    "PRMkConfigDefault",
    "RRTstarkConfigDefault",
]

# Number of runs and time limit per planner
RUNS = 100          # you can lower to e.g. 50 if this feels too slow
TIME_LIMIT = 5.0    # seconds

# Where to store CSV results (in your home, not /root)
RESULTS_DIR = os.path.expanduser("~/benchmarks")


def joint_path_length(pts):
    """Compute Euclidean joint-space path length over the trajectory points."""
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
    """Call group.plan() and return (ok, traj, tsec), handling both tuple and non-tuple returns."""
    t0 = time.time()
    plan = group.plan()
    dt = time.time() - t0

    ok, traj = False, None
    try:
        # MoveIt sometimes returns (success, traj, ...) and sometimes just traj
        ok = bool(plan[0])
        traj = plan[1] if ok else None
    except Exception:
        ok = bool(plan)
        traj = plan if ok else None

    return ok, traj, dt


def sample_valid_goal(group_probe, ns, group_name, tries=30):
    """
    Try up to `tries` random joint goals and quickly verify with RRTConnect (0.8s).
    Returns a joint-value list that is likely valid for other planners to attempt,
    or None if nothing worked.
    """
    gp = MoveGroupCommander(group_name, ns=ns)
    gp.set_planner_id("RRTConnectkConfigDefault")
    gp.set_planning_time(0.8)
    gp.set_num_planning_attempts(1)
    gp.set_start_state_to_current_state()

    for _ in range(tries):
        target = gp.get_random_joint_values()
        gp.set_joint_value_target(target)
        ok, traj, _ = call_plan(gp)
        if ok and hasattr(traj, "joint_trajectory") and traj.joint_trajectory.points:
            return target

    return None


def add_static_box(robot, ns, name="static_block"):
    """
    Add a single static box obstacle into the planning scene in front of the robot.
    Adjust x/y/z if you want it more or less in the way.
    """
    scene = PlanningSceneInterface(ns=ns)
    rospy.sleep(1.0)  # allow scene to connect

    world = robot.get_planning_frame()

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = world
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = 0.4   # forward from base
    pose.pose.position.y = 0.0
    pose.pose.position.z = 1.30   # some height above table

    size = (0.20, 0.20, 0.20)    # 20cm cube

    # remove any old box with the same name, then add a fresh one
    scene.remove_world_object(name)
    rospy.sleep(0.5)
    scene.add_box(name, pose, size)
    rospy.sleep(1.0)

    rospy.loginfo(f"Static box '{name}' added at x={pose.pose.position.x}, "
                  f"y={pose.pose.position.y}, z={pose.pose.position.z}")


def main():
    roscpp_initialize([])
    rospy.init_node("parallel_planning_benchmark_static", anonymous=True)

    # Detect iiwa namespace (same logic as original script)
    ns = ""
    desc = "robot_description"
    if rospy.has_param("/iiwa/robot_description"):
        ns = "/iiwa"
        desc = "/iiwa/robot_description"

    # Wait for the robot_description to be available
    if not wait_for_param(desc, 30.0):
        raise RuntimeError(
            "Timeout waiting for %s. Is the iiwa MoveIt demo running?" % desc
        )

    robot = RobotCommander(robot_description=desc, ns=ns)
    groups = robot.get_group_names()
    group_name = "manipulator" if "manipulator" in groups else groups[0]

    # Add a static obstacle to the planning scene
    add_static_box(robot, ns, name="static_block")

    # Prepare output CSV
    os.makedirs(RESULTS_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_csv = os.path.join(
        RESULTS_DIR, f"bench_static_{group_name}_{stamp}.csv"
    )

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["run", "planner", "ok", "time_s", "points", "joint_path_len"]
        )

        # Probe commander for random goals
        probe = MoveGroupCommander(group_name, ns=ns)

        for r in range(1, RUNS + 1):
            # Choose a goal that is plannable by a quick RRTConnect check
            target = sample_valid_goal(probe, ns, group_name, tries=40)
            if target is None:
                print(
                    f"[run {r:02d}] could not find a valid random target, skipping"
                )
                continue

            results = {}
            threads = []

            def try_plan(pid):
                g = MoveGroupCommander(group_name, ns=ns)  # per-thread commander
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
                    f"[run {r:02d}] [{pid}] ok={ok} t={dt:.3f}s points={npts} L={L:.3f}"
                )

            # Run all planners in parallel, like the original script
            for pid in PLANNERS:
                t = threading.Thread(target=try_plan, args=(pid,))
                t.start()
                threads.append(t)

            for t in threads:
                t.join()

            # Write per-planner rows
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

            # --- PRM + RRTConnect first-finish baseline ---
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
            # --- end first-finish baseline ---

            f.flush()

            # Optional: print best by time among all planners (including RRT*)
            best = min(
                (res for res in results.values() if res["ok"]),
                key=lambda x: x["time"],
                default=None,
            )
            print(f"[run {r:02d}] best by time: {best}")

    print("Saved:", out_csv)
    roscpp_shutdown()


if __name__ == "__main__":
    main()

