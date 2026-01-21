#!/usr/bin/env python3

# Standard library
import sys
import math
import random
import argparse
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict, Sequence, Iterable, Callable
# ROS imports
import rospy
from geometry_msgs.msg import Pose
from moveit_commander import (
    MoveGroupCommander,
    PlanningSceneInterface,
    RobotCommander,
    roscpp_initialize,
    roscpp_shutdown,
)
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

rospy.loginfo("ROS node started")
print("ROS node started")


@dataclass(frozen=True)
class PlannerSpec:
    planner_id: str
    label: str
    apply_shortcut: bool

# Make a planner object
@dataclass
class PlannerResult:
    spec: PlannerSpec
    success: bool
    planning_time: float
    path_length: float
    trajectory: Optional[RobotTrajectory]
    notes: str = ""

 # List of planners
PLANNER_CONFIGS: Sequence[PlannerSpec] = (
    PlannerSpec("PRMkConfigDefault", "PRM + Shortcutting", True),
    PlannerSpec("RRTkConfigDefault", "RRT + Shortcutting", True),
    PlannerSpec("PRMstarkConfigDefault", "PRM*", False),
    PlannerSpec("RRTstarkConfigDefault", "RRT*", False),
)

# List of waypoints, to be updated to Hell Yeah
PRESET_TARGETS: Sequence[Tuple[str, Tuple[float, float, float]]] = (
    ("Waypoint A", (0.35, -0.40, 1.05)),
    ("Waypoint B", (0.20, -0.25, 1.10)),
    ("Waypoint C", (0.10, -0.50, 0.95)),
)

# Find length of path
def joint_path_length(points: List) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(
            sum((a - b) ** 2 for a, b in zip(first.positions, second.positions))
        )
    return total

# Plans trajec
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
        as_traj.multi_dof_joint_trajectory = plan.multi_dof_joint_trajectory
        return as_traj
    return None

# Argument parsing based on what is needed
def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare OMPL planners."
    )
    parser.add_argument(
        "--target",
        action="append",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Add a pose target.",
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
        help="Number of planning attempts per planner",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the shortest trajectory.",
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

        self._state_validity_srv: Optional[Callable] = None
        self._shortcut_warning_emitted = False
        try:
            rospy.wait_for_service("check_state_validity", timeout=5.0)
            self._state_validity_srv = rospy.ServiceProxy("check_state_validity", GetStateValidity)
            rospy.loginfo("Shortcutting enabled via check_state_validity service")
            print("[compare_planners] Shortcutting enabled (check_state_validity available)")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self._state_validity_srv = None
            rospy.logwarn("Shortcutting disabled; check_state_validity unavailable: %s", exc)
            print(f"[compare_planners] Shortcutting disabled; check_state_validity unavailable: {exc}")

    # Make goal position
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

    # Plan with the 4 different planners (choose 1)
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

    # Display trajectory in RVIZ
    def publish_display(self, trajectory: RobotTrajectory) -> None:
        print("[compare_planners] Publishing trajectory to RViz/DisplayTrajectory...")
        msg = DisplayTrajectory()
        msg.trajectory_start = self.robot.get_current_state()
        msg.trajectory.append(trajectory)
        self.display_pub.publish(msg)

    # Checks if any calls failed
    def _state_is_valid(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        state_template,
        index_map: Dict[str, int],
    ) -> bool:
        if self._state_validity_srv is None:
            return False
        state_copy = deepcopy(state_template)
        positions_list = list(state_copy.joint_state.position)  # make mutable
        for name, value in zip(joint_names, positions):
            idx = index_map.get(name)
            if idx is None:
                rospy.logwarn(
                    "Shortcutting aborted; joint %s is missing in the current robot state",
                    name,
                )
                print(
                    f"[compare_planners] Shortcutting aborted; joint {name} missing in robot state"
                )
                return False
            positions_list[idx] = value
        state_copy.joint_state.position = tuple(positions_list)

        request = GetStateValidityRequest()
        request.group_name = self.group.get_name()
        request.robot_state = state_copy
        try:
            response = self._state_validity_srv(request)
        except rospy.ServiceException as exc:
            rospy.logwarn(
                "check_state_validity call failed during shortcutting; disabling further shortcut attempts: %s",
                exc,
            )
            print(
                "[compare_planners] check_state_validity failed during shortcutting; disabling further attempts"
            )
            self._state_validity_srv = None
            return False
        return bool(response.valid)
    
    # Checks segment
    def _segment_is_valid(
        self,
        joint_names: Sequence[str],
        start: Sequence[float],
        end: Sequence[float],
        state_template,
        index_map: Dict[str, int],
        samples: int,
    ) -> bool:
        for step in range(1, samples):
            ratio = step / float(samples)
            interp = [s + ratio * (e - s) for s, e in zip(start, end)]
            if not self._state_is_valid(joint_names, interp, state_template, index_map):
                return False
        return True
    
    # Shortcutting code, and how much it improves
    def shortcut_trajectory(
        self,
        trajectory: RobotTrajectory,
        max_iterations: int = 80,
        samples: int = 12,
    ) -> Tuple[RobotTrajectory, float, bool]:
        # Collision-aware random shortcutting mirroring OMPL's PathSimplifier.
        if self._state_validity_srv is None:
            if not self._shortcut_warning_emitted:
                rospy.logwarn("Shortcutting skipped; validity service unavailable.")
                print("[compare_planners] Shortcutting skipped; validity service unavailable")
                self._shortcut_warning_emitted = True
            points = trajectory.joint_trajectory.points if trajectory else []
            return trajectory, joint_path_length(points), False

        if not trajectory or not trajectory.joint_trajectory.points:
            return trajectory, 0.0, False

        original_points = trajectory.joint_trajectory.points
        original_length = joint_path_length(original_points)
        if len(original_points) < 3 or original_length <= 0.0:
            return trajectory, original_length, False

        working = deepcopy(trajectory)
        points = working.joint_trajectory.points
        joint_names = list(working.joint_trajectory.joint_names)

        state_template = self.robot.get_current_state()
        index_map = {name: idx for idx, name in enumerate(state_template.joint_state.name)}

        improved = False
        for _ in range(max_iterations):
            if len(points) < 3:
                break
            first_idx = random.randrange(0, len(points) - 2)
            second_idx = random.randrange(first_idx + 2, len(points))

            start = points[first_idx].positions
            end = points[second_idx].positions

            if not self._segment_is_valid(
                joint_names, start, end, state_template, index_map, samples
            ):
                continue

            del points[first_idx + 1 : second_idx]
            improved = True

        if not improved:
            return trajectory, original_length, False

        new_length = joint_path_length(points)
        rospy.loginfo(
            "Shortcutting reduced path from %.4f to %.4f (%.1f%% shorter)",
            original_length,
            new_length,
            (1 - new_length / original_length) * 100.0 if original_length else 0.0,
        )
        print(
            f"[compare_planners] Shortcutting reduced path from {original_length:.4f} to {new_length:.4f}"
        )
        return working, new_length, True

    def evaluate_goal(
        self, goal: Pose, planning_time: float, attempts: int
    ) -> List[PlannerResult]:
        results: List[PlannerResult] = []
        for spec in PLANNER_CONFIGS:
            rospy.loginfo("Planning with %s (%s)", spec.label, spec.planner_id)
            print(
                f"[compare_planners] Evaluating planner {spec.label} ({spec.planner_id})..."
            )
            success, trajectory, wall_time = self.plan_with(
                spec.planner_id, goal, planning_time, attempts
            )
            if not success or trajectory is None:
                rospy.logwarn("Planner %s failed to find a solution", spec.planner_id)
                print(
                    f"[compare_planners] Planner {spec.planner_id} failed: time={wall_time:.3f}s"
                )
                results.append(
                    PlannerResult(
                        spec=spec,
                        success=False,
                        planning_time=wall_time,
                        path_length=math.inf,
                        trajectory=None,
                        notes="planning failure",
                    )
                )
                continue

            processed = trajectory
            initial_length = joint_path_length(trajectory.joint_trajectory.points)
            final_length = initial_length
            note = ""

            if spec.apply_shortcut:
                processed, final_length, shortened = self.shortcut_trajectory(trajectory)
                if shortened and final_length < initial_length:
                    reduction = initial_length - final_length
                    note = f"shortcut saved {reduction:.4f}"
                elif shortened:
                    note = "shortcut applied"

            rospy.loginfo(
                "Planner %s success. time=%.3fs points=%d length=%.4f",
                spec.planner_id,
                wall_time,
                len(processed.joint_trajectory.points),
                final_length,
            )
            print(
                f"[compare_planners] Planner {spec.planner_id} success: time={wall_time:.3f}s, points={len(processed.joint_trajectory.points)}, length={final_length:.4f}"
            )
            if note:
                print(f"[compare_planners]    Note: {note}")

            results.append(
                PlannerResult(
                    spec=spec,
                    success=True,
                    planning_time=wall_time,
                    path_length=final_length,
                    trajectory=processed,
                    notes=note,
                )
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

        viable = [entry for entry in results if entry.success]
        if not viable:
            rospy.logerr("No planner produced a valid plan for %s", label)
            print(f"[compare_planners] No valid plan for {label}")
            continue

        best = min(viable, key=lambda entry: entry.path_length)
        rospy.loginfo(
            "Selected planner %s (%s) with joint path length %.4f",
            best.spec.planner_id,
            best.spec.label,
            best.path_length,
        )
        print(
            f"[compare_planners] Selected planner {best.spec.planner_id} ({best.spec.label}) length={best.path_length:.4f}"
        )
        if best.notes:
            print(f"[compare_planners] Best planner note: {best.notes}")

        if best.trajectory:
            planner.publish_display(best.trajectory)
            print("[compare_planners] Trajectory published for visualization")

        if args.execute and best.trajectory:
            rospy.loginfo("Executing best trajectory for %s", label)
            planner.group.execute(best.trajectory, wait=True)
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
