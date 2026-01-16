#!/usr/bin/env python3
"""
Planner Benchmark + Integration Handshake

Path:
    hell_yeah -> SLIDER_BOX -> FAULT_DETECTION -> PRESS -> hell_yeah

After each reached waypoint (except HOME), Group1:
  - publishes /group1/motion/isTargetReached = True
  - (optionally) publishes /group1/allowFaultDetection = True only at FAULT_DETECTION
  - resets /group1/getNewGoal = False (prep reset)
  - waits until /group1/getNewGoal becomes True (LEVEL signal, not a pulse)
  - ACK resets /group1/getNewGoal back to False
  - clears isTargetReached + allowFaultDetection
"""
import os
import math
import threading
from typing import Dict, List, Optional, Tuple

import rospy
from std_msgs.msg import Bool
from moveit_commander import (
    roscpp_initialize,
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, RobotTrajectory, DisplayTrajectory


# =========================
# CONFIGURATION
# =========================
PLANNERS: Dict[str, str] = {
    "RRTConnectkConfigDefault": "RRTConnect",
    "PRMkConfigDefault": "PRM",
    "RRTstarkConfigDefault": "RRT*",
}

RUNS = 3
TIME_LIMIT = 10.0
NUM_ATTEMPTS = 3
RESULTS_DIR = os.path.expanduser("~/benchmarks")

USE_OBSTACLES = False
USE_JOINT_TARGETS = True

HELL_YEAH_JOINTS = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]

POSITION_A = {
    "name": "Position_A",
    "position": [0.57105, 0.00817, 0.25738],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "joints": [0.1316, 0.7100, -0.1201, -1.6096, -0.0627, 0.7987, 0.0298],
}

POSITION_B = {
    "name": "Position_B",
    "position": [0.5, -0.3, 0.1],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "joints": [-1.240, 0.702, -0.353, -1.661, 0.472, 0.424, 1.283],
}

POSITION_C = {
    "name": "Position_C",
    "position": [0.5, 0.3, 0.1],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "joints": [1.240, 0.702, -0.353, -1.661, 0.472, 0.424, 1.283],
}

OBSTACLES = [
    {
        "name": "obstacle_A_to_B",
        "type": "box",
        "dimensions": [0.1, 0.2, 0.2],
        "position": [0.25, -0.7, 1.25],
    },
    {
        "name": "obstacle_near_B",
        "type": "box",
        "dimensions": [0.15, 0.15, 0.5],
        "position": [0.45, 0.15, 0.8],
    },
]


# =========================
# HELPERS
# =========================
def joint_path_length(points: List) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(sum((a - b) ** 2 for a, b in zip(first.positions, second.positions)))
    return total


def to_robot_trajectory(plan: object) -> Optional[RobotTrajectory]:
    if isinstance(plan, RobotTrajectory):
        return plan
    if isinstance(plan, tuple) and plan:
        success = bool(plan[0])
        trajectory = plan[1] if len(plan) > 1 else None
        if success and isinstance(trajectory, RobotTrajectory):
            return trajectory
        return trajectory if success else None
    if hasattr(plan, "joint_trajectory"):
        as_traj = RobotTrajectory()
        as_traj.joint_trajectory = plan.joint_trajectory
        as_traj.multi_dof_joint_trajectory = getattr(plan, "multi_dof_joint_trajectory", None)
        return as_traj
    return None


def wait_for_param(name: str, timeout: float = 30.0) -> bool:
    start = rospy.get_time()
    while not rospy.is_shutdown():
        if rospy.has_param(name):
            return True
        if rospy.get_time() - start > timeout:
            return False
        rospy.sleep(0.1)
    return False


def plan_with_planner(
    group: MoveGroupCommander,
    planner_id: str,
    target: dict,
    planning_time: float = TIME_LIMIT,
    num_attempts: int = NUM_ATTEMPTS,
) -> Tuple[bool, Optional[RobotTrajectory], float, int, float]:
    group.set_start_state_to_current_state()
    group.set_planner_id(planner_id)
    group.set_planning_time(planning_time)
    group.set_num_planning_attempts(num_attempts)

    ttype = target.get("type", "joints")
    if ttype == "named":
        group.set_named_target(target["name"])
    elif ttype == "joints":
        group.set_joint_value_target(target["joints"])
    elif ttype == "pose":
        group.set_pose_target(target["pose"])
    else:
        rospy.logwarn(f"Unknown target type: {ttype}")
        return False, None, 0.0, 0, 0.0

    start_time = rospy.get_time()
    plan_result = group.plan()
    wall_time = rospy.get_time() - start_time

    traj = to_robot_trajectory(plan_result)
    success = bool(traj and traj.joint_trajectory.points)

    num_points = 0
    length = 0.0
    if success and traj:
        pts = traj.joint_trajectory.points
        num_points = len(pts)
        length = joint_path_length(pts)

    group.clear_pose_targets()
    return success, traj, wall_time, num_points, length


def create_pose(position: List[float], orientation: List[float]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


def add_obstacles(scene, frame_id="world"):
    rospy.sleep(0.5)
    for obs in OBSTACLES:
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = obs["name"]

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = obs["dimensions"]

        box_pose = Pose()
        box_pose.position.x, box_pose.position.y, box_pose.position.z = obs["position"]
        box_pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)
        co.operation = CollisionObject.ADD

        scene.add_object(co)
        print(f"  Added obstacle: {obs['name']} at {obs['position']}")
    rospy.sleep(0.5)


def remove_obstacles(scene):
    for obs in OBSTACLES:
        scene.remove_world_object(obs["name"])
    rospy.sleep(0.3)


def publish_bool(pub, value: bool, label: str = ""):
    pub.publish(Bool(value))
    if label:
        rospy.loginfo("[handshake] %s = %s", label, value)


def wait_for_new_goal_level(timeout: float = 300.0) -> bool:
    """
    LEVEL handshake: wait until /group1/getNewGoal becomes True (and stays True until we ACK reset).
    """
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/group1/getNewGoal", Bool, timeout=0.5)
            if msg.data:
                return True
        except rospy.ROSException:
            pass
        if rospy.Time.now().to_sec() - t0 > timeout:
            return False
    return False


def do_handoff(stage: str, pub_target_reached, pub_allow_fault, pub_get_new_goal_reset):
    """
    Common handoff logic after reaching a waypoint.
    """
    is_fault_stage = (stage == "FAULT_DETECTION")

    publish_bool(pub_allow_fault, is_fault_stage, "/group1/allowFaultDetection")
    publish_bool(pub_target_reached, True, "/group1/motion/isTargetReached")

    # Prep reset: ensure we won't be fooled by an old True
    publish_bool(pub_get_new_goal_reset, False, "/group1/getNewGoal (prep reset)")

    rospy.loginfo("[handshake] Waiting for /group1/getNewGoal=True (stage=%s)...", stage)
    ok = wait_for_new_goal_level(timeout=300.0)

    if not ok:
        rospy.logwarn("[handshake] Timeout waiting for /group1/getNewGoal. Continuing anyway...")
    else:
        rospy.loginfo("[handshake] Received /group1/getNewGoal=True. Continuing.")
        publish_bool(pub_get_new_goal_reset, False, "/group1/getNewGoal (ACK reset)")

    # Clear for next leg
    publish_bool(pub_target_reached, False, "/group1/motion/isTargetReached")
    publish_bool(pub_allow_fault, False, "/group1/allowFaultDetection")


# =========================
# MAIN
# =========================
def main():
    roscpp_initialize([])
    rospy.init_node("parallel_planning_benchmark", anonymous=True)

    print("[benchmark] Starting parallel planning benchmark...")
    print(f"[benchmark] Planners: {list(PLANNERS.values())}")
    print(f"[benchmark] Planning time: {TIME_LIMIT}s, Attempts: {NUM_ATTEMPTS}")

    ns = ""
    desc = "robot_description"
    if rospy.has_param("/iiwa/robot_description"):
        ns = "/iiwa"
        desc = "/iiwa/robot_description"
    print(f"[benchmark] Using robot_description: {desc}")

    if not wait_for_param(desc, 30.0):
        raise RuntimeError(f"Timeout waiting for {desc}. Is MoveIt running?")

    robot = RobotCommander(robot_description=desc, ns=ns)
    scene = PlanningSceneInterface(ns=ns)
    groups = robot.get_group_names()

    group_name = "arm" if "arm" in groups else ("manipulator" if "manipulator" in groups else groups[0])
    print(f"[benchmark] Using planning group: {group_name}")
    print(f"[benchmark] Available groups: {groups}")

    group = MoveGroupCommander(group_name, ns=ns)
    print(f"[benchmark] End effector: {group.get_end_effector_link()}")
    print(f"[benchmark] Planning frame: {group.get_planning_frame()}")

    display_pub = rospy.Publisher("move_group/display_planned_path", DisplayTrajectory, queue_size=10)

    # Handshake pubs (Group1 -> others + ACK reset of getNewGoal)
    pub_target_reached = rospy.Publisher("/group1/motion/isTargetReached", Bool, queue_size=10, latch=True)
    pub_allow_fault = rospy.Publisher("/group1/allowFaultDetection", Bool, queue_size=10, latch=True)
    pub_get_new_goal_reset = rospy.Publisher("/group1/getNewGoal", Bool, queue_size=10)  # NOT latched

    publish_bool(pub_target_reached, False, "/group1/motion/isTargetReached")
    publish_bool(pub_allow_fault, False, "/group1/allowFaultDetection")
    publish_bool(pub_get_new_goal_reset, False, "/group1/getNewGoal (init reset)")

    cur = group.get_current_joint_values()
    print(f"[benchmark] Current joint values: {[f'{j:.3f}' for j in cur]}")

    os.makedirs(RESULTS_DIR, exist_ok=True)

    if USE_OBSTACLES:
        print("\n=== Adding Obstacles ===")
        add_obstacles(scene, frame_id=group.get_planning_frame())
    else:
        print("\n=== Obstacles DISABLED ===")

    if USE_JOINT_TARGETS:
        waypoints = [
            {"name": "hell_yeah",        "stage": "HOME",            "type": "joints", "joints": HELL_YEAH_JOINTS},
            {"name": POSITION_C["name"], "stage": "SLIDER_BOX",      "type": "joints", "joints": POSITION_C["joints"]},
            {"name": POSITION_A["name"], "stage": "FAULT_DETECTION", "type": "joints", "joints": POSITION_A["joints"]},
            {"name": POSITION_B["name"], "stage": "PRESS",           "type": "joints", "joints": POSITION_B["joints"]},
            {"name": "hell_yeah",        "stage": "HOME",            "type": "joints", "joints": HELL_YEAH_JOINTS},
        ]
        print("\n=== Using JOINT targets ===")
    else:
        waypoints = [
            {"name": "hell_yeah",        "stage": "HOME",            "type": "joints", "joints": HELL_YEAH_JOINTS},
            {"name": POSITION_C["name"], "stage": "SLIDER_BOX",      "type": "pose", "pose": create_pose(POSITION_C["position"], POSITION_C["orientation"])},
            {"name": POSITION_A["name"], "stage": "FAULT_DETECTION", "type": "pose", "pose": create_pose(POSITION_A["position"], POSITION_A["orientation"])},
            {"name": POSITION_B["name"], "stage": "PRESS",           "type": "pose", "pose": create_pose(POSITION_B["position"], POSITION_B["orientation"])},
            {"name": "hell_yeah",        "stage": "HOME",            "type": "joints", "joints": HELL_YEAH_JOINTS},
        ]
        print("\n=== Using POSE targets (Cartesian) ===")

    print("\n=== Benchmark Path ===")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. {wp['name']} ({wp.get('stage','')})")

    for run in range(1, RUNS + 1):
        print(f"\n{'='*60}\nRUN {run}/{RUNS}\n{'='*60}")

        current = group.get_current_joint_values()
        diff = sum(abs(c - h) for c, h in zip(current, HELL_YEAH_JOINTS))
        print(f"\n>>> Start joints: {[f'{j:.3f}' for j in current]}")
        print(f">>> hell_yeah:    {[f'{j:.3f}' for j in HELL_YEAH_JOINTS]}")
        print(f">>> diff: {diff:.3f} rad")

        rospy.sleep(0.5)

        for motion_idx in range(len(waypoints) - 1):
            from_wp = waypoints[motion_idx]
            to_wp = waypoints[motion_idx + 1]
            motion_name = f"{from_wp['name']} -> {to_wp['name']}"
            print(f"\n--- Motion {motion_idx + 1}: {motion_name} ---")

            results: Dict[str, dict] = {}
            threads: List[threading.Thread] = []
            results_lock = threading.Lock()

            def try_plan_thread(planner_id: str, display_name: str):
                g = MoveGroupCommander(group_name, ns=ns)
                success, trajectory, wall_time, num_points, path_length = plan_with_planner(
                    group=g,
                    planner_id=planner_id,
                    target=to_wp,
                    planning_time=TIME_LIMIT,
                    num_attempts=NUM_ATTEMPTS,
                )
                with results_lock:
                    results[planner_id] = {
                        "display_name": display_name,
                        "success": success,
                        "time": wall_time,
                        "points": num_points,
                        "length": path_length,
                        "trajectory": trajectory,
                    }

            for planner_id, display_name in PLANNERS.items():
                t = threading.Thread(target=try_plan_thread, args=(planner_id, display_name))
                t.start()
                threads.append(t)

            for t in threads:
                t.join()

            print("\n  Results:")
            for planner_id, res in results.items():
                status = "✓" if res["success"] else "✗"
                print(
                    f"    {res['display_name']:12} [{planner_id}]: {status} "
                    f"time={res['time']:.3f}s, waypoints={res['points']}, path_len={res['length']:.4f}"
                )

            successful = [(pid, res) for pid, res in results.items() if res["success"]]

            if successful:
                best_pid, best_res = min(successful, key=lambda x: x[1]["length"])
                print(f"\n  Selected: {best_res['display_name']} (shortest path: {best_res['length']:.4f})")

                if best_res["trajectory"]:
                    display_msg = DisplayTrajectory()
                    display_msg.trajectory_start = robot.get_current_state()
                    display_msg.trajectory.append(best_res["trajectory"])
                    display_pub.publish(display_msg)
                    rospy.sleep(0.2)

                print("  Executing trajectory...")
                group.set_start_state_to_current_state()
                group.execute(best_res["trajectory"], wait=True)
                group.stop()
                print("  Execution complete.")
                rospy.sleep(0.5)

                # Handshake after reaching destination (skip HOME stages if you want)
                stage = to_wp.get("stage", "")
                if stage != "HOME":
                    do_handoff(stage, pub_target_reached, pub_allow_fault, pub_get_new_goal_reset)

            else:
                print("\n  WARNING: All planners failed!")
                print("  Attempting fallback with go() command...")

                if to_wp["type"] == "named":
                    group.set_named_target(to_wp["name"])
                elif to_wp["type"] == "joints":
                    group.set_joint_value_target(to_wp["joints"])
                else:
                    group.set_pose_target(to_wp["pose"])

                success = group.go(wait=True)
                group.stop()

                if success:
                    print("  Fallback execution successful.")
                    rospy.sleep(0.5)
                    stage = to_wp.get("stage", "")
                    if stage != "HOME":
                        do_handoff(stage, pub_target_reached, pub_allow_fault, pub_get_new_goal_reset)
                else:
                    print("  Fallback also failed! Skipping this motion.")
                    rospy.sleep(1.0)

        print(f"\n>>> Run {run} complete!")

    if USE_OBSTACLES:
        remove_obstacles(scene)
        print("  Obstacles removed.")

    print("\n[benchmark] Benchmark complete!")
    rospy.spin()


if __name__ == "__main__":
    main()
