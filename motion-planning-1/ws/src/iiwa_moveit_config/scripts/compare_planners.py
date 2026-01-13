#!/usr/bin/env python3
"""Compare several OMPL planners for predefined waypoints and execute the shortest."""

import argparse
import math
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

rospy.loginfo("ROS node started")
print("ROS node started")

PLANNERS: Dict[str, str] = {
    "PRMkConfigDefault": "PRM",
    "RRTkConfigDefault": "RRT",
    "RRTConnectkConfigDefault": "RRTConnect",
}

PRESET_TARGETS: Sequence[Tuple[str, Tuple[float, float, float]]] = (
    ("Waypoint A", (0.35, -0.40, 1.05)),
    ("Waypoint B", (0.20, -0.25, 1.10)),
    ("Waypoint C", (0.10, -0.50, 0.95)),
)


def joint_path_length(points: List) -> float:
    """Return the cumulative joint-space distance for a trajectory."""
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(
            sum((a - b) ** 2 for a, b in zip(first.positions, second.positions))
        )
    return total


def to_robot_trajectory(plan: object) -> Optional[RobotTrajectory]:
    """Normalize MoveGroupCommander.plan results."""
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
        as_traj.multi_dof_joint_trajectory = plan.multi_dof_joint_trajectory
        return as_traj
    return None


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare OMPL planners for pose goals. Without --target, built-in presets are used."
    )
    parser.add_argument(
        "--target",
        action="append",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Add a pose target in planning frame coordinates (can be passed multiple times).",
    )
    parser.add_argument(
        "--planning-group",
        default="arm",
        help="MoveIt planning group to command (default: arm)",
    )
    parser.add_argument(
        "--planning-time",
        type=float,
        default=5.0,
        help="Maximum planning time per planner",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=1,
        help="Number of planning attempts for each planner",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the shortest trajectory for each evaluated target.",
    )
    return parser.parse_args(argv)


class MultiPlannerExecutor:
    def __init__(self, group_name: str, planning_time: float, num_attempts: int):
        print("[compare_planners] Initializing ROS and MoveIt...")
        roscpp_initialize(sys.argv)
        rospy.init_node("multi_planner_executor", anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(group_name)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(num_attempts)

        print(f"[compare_planners] Planning group: {group_name}")
        print(f"[compare_planners] Planning time: {planning_time}, attempts: {num_attempts}")

        self.display_pub = rospy.Publisher(
            "move_group/display_planned_path", DisplayTrajectory, queue_size=10
        )

        rospy.loginfo("Planning frame: %s", self.group.get_planning_frame())
        rospy.loginfo("End-effector link: %s", self.group.get_end_effector_link())
        rospy.loginfo("Default planner ID: %s", self.group.get_planner_id())

        print(f"[compare_planners] Planning frame: {self.group.get_planning_frame()}")
        print(f"[compare_planners] End-effector link: {self.group.get_end_effector_link()}")
        print(f"[compare_planners] Default planner ID: {self.group.get_planner_id()}")

        try:
            ids = self.group.get_known_planner_ids()
            rospy.loginfo("Known planner IDs (MoveGroupCommander): %s", ids)
            print(f"[compare_planners] Known planner IDs: {ids}")
        except Exception as exc:
            rospy.logwarn("Failed to query known planner IDs: %s", exc)
            print(f"[compare_planners] Failed to query known planner IDs: {exc}")

        try:
            param_candidates = [name for name in rospy.get_param_names() if "planner_configs" in name]
            rospy.loginfo("Parameter names mentioning planner_configs: %s", param_candidates)
            print(f"[compare_planners] Params mentioning planner_configs: {param_candidates}")
            for pname in param_candidates:
                try:
                    value = rospy.get_param(pname)
                except Exception as get_exc:
                    rospy.logwarn("Could not read %s: %s", pname, get_exc)
                    print(f"[compare_planners] Could not read {pname}: {get_exc}")
                    continue
                keys = list(value.keys()) if isinstance(value, dict) else type(value).__name__
                rospy.loginfo("Param %s keys/type: %s", pname, keys)
                print(f"[compare_planners] Param {pname} keys/type: {keys}")
        except Exception as exc:
            rospy.logwarn("Failed to inspect parameter server for planner configs: %s", exc)
            print(f"[compare_planners] Failed to inspect parameter server: {exc}")

    def make_goal_pose(self, x: float, y: float, z: float) -> Pose:
        print(f"[compare_planners] make_goal_pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z

        current_pose = self.group.get_current_pose().pose
        goal.orientation = current_pose.orientation
        print("[compare_planners] Goal orientation copied from current pose")
        return goal

    def plan_with(self, planner_id: str, goal: Pose, planning_time: float, attempts: int) -> Tuple[bool, Optional[RobotTrajectory], float]:
        print(f"[compare_planners] plan_with start: planner_id={planner_id}, planning_time={planning_time}, attempts={attempts}")
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(goal)
        self.group.set_planner_id(planner_id)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(attempts)

        start = rospy.get_time()
        plan = self.group.plan()
        duration = rospy.get_time() - start

        trajectory = to_robot_trajectory(plan)
        success = bool(trajectory and trajectory.joint_trajectory.points)

        points_count = len(trajectory.joint_trajectory.points) if trajectory else 0
        print(f"[compare_planners] plan_with done: success={success}, time={duration:.3f}s, points={points_count}")

        self.group.clear_pose_targets()
        return success, trajectory, duration

    def publish_display(self, trajectory: RobotTrajectory) -> None:
        print("[compare_planners] Publishing trajectory to RViz/DisplayTrajectory...")
        msg = DisplayTrajectory()
        msg.trajectory_start = self.robot.get_current_state()
        msg.trajectory.append(trajectory)
        self.display_pub.publish(msg)

    def evaluate_goal(
        self, goal: Pose, planning_time: float, attempts: int
    ) -> List[Tuple[str, str, bool, float, float, Optional[RobotTrajectory]]]:
        results = []
        for planner_id, display_name in PLANNERS.items():
            rospy.loginfo("Planning with %s (%s)", display_name, planner_id)
            print(f"[compare_planners] Evaluating planner {display_name} ({planner_id})...")
            success, trajectory, wall_time = self.plan_with(
                planner_id, goal, planning_time, attempts
            )
            if not success or trajectory is None:
                rospy.logwarn("Planner %s failed to find a solution", planner_id)
                print(f"[compare_planners] Planner {planner_id} failed: time={wall_time:.3f}s")
                results.append((planner_id, display_name, False, wall_time, math.inf, None))
                continue

            points = trajectory.joint_trajectory.points
            path_length = joint_path_length(points)
            rospy.loginfo(
                "Planner %s success. time=%.3fs points=%d length=%.4f",
                planner_id,
                wall_time,
                len(points),
                path_length,
            )
            print(f"[compare_planners] Planner {planner_id} success: time={wall_time:.3f}s, points={len(points)}, length={path_length:.4f}")
            results.append(
                (planner_id, display_name, True, wall_time, path_length, trajectory)
            )
        return results

    def shutdown(self) -> None:
        roscpp_shutdown()


def main() -> None:
    print("[compare_planners] Starting compare_planners.py...")
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    print(f"[compare_planners] Args: planning_group={args.planning_group}, planning_time={args.planning_time}, attempts={args.attempts}, execute={args.execute}")
    planner = MultiPlannerExecutor(args.planning_group, args.planning_time, args.attempts)

    if args.target:
        targets: Iterable[Tuple[str, Tuple[float, float, float]]] = [
            (f"CLI target {idx + 1}", (coords[0], coords[1], coords[2]))
            for idx, coords in enumerate(args.target)
        ]
    else:
        rospy.loginfo("No --target provided; using preset waypoints: %s", PRESET_TARGETS)
        print(f"[compare_planners] No --target provided; using presets: {PRESET_TARGETS}")
        targets = PRESET_TARGETS

    for label, (x, y, z) in targets:
        rospy.loginfo("=== Evaluating %s @ (%.3f, %.3f, %.3f)", label, x, y, z)
        print(f"[compare_planners] === Evaluating {label} @ ({x:.3f}, {y:.3f}, {z:.3f})")
        goal_pose = planner.make_goal_pose(x, y, z)
        results = planner.evaluate_goal(goal_pose, args.planning_time, args.attempts)

        viable = [entry for entry in results if entry[2]]
        if not viable:
            rospy.logerr("No planner produced a valid plan for %s", label)
            print(f"[compare_planners] No valid plan for {label}")
            continue

        best = min(viable, key=lambda entry: entry[4])
        rospy.loginfo(
            "Selected planner %s (%s) with joint path length %.4f",
            best[0],
            best[1],
            best[4],
        )
        print(f"[compare_planners] Selected planner {best[0]} ({best[1]}) length={best[4]:.4f}")

        if best[5]:
            planner.publish_display(best[5])
            print("[compare_planners] Trajectory published for visualization")

        if args.execute and best[5]:
            rospy.loginfo("Executing best trajectory for %s", label)
            planner.group.execute(best[5], wait=True)
            planner.group.stop()
            print(f"[compare_planners] Executed trajectory for {label}")
        else:
            rospy.loginfo(
                "Execution disabled; trajectory for %s was only visualized.", label
            )
            print(f"[compare_planners] Execution disabled; {label} only visualized")

    planner.shutdown()
    print("[compare_planners] Shutdown complete")


if __name__ == "__main__":
    main()
