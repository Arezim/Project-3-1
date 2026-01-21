#!/usr/bin/env python3
"""
Comprehensive Planner Benchmark with CSV Export

Tests 4 planning algorithms:
  1. RRT*
  2. PRM*
  3. RRT + Shortcut
  4. PRM + Shortcut

Across 3 environments:
  1. No obstacles
  2. Static obstacles
  3. Dynamic obstacles

Results are saved to CSV files in ~/benchmarks/

Usage:
  Normal mode (with RViz): python3 benchmark_planners.py
  Headless mode (no RViz): python3 benchmark_planners.py --headless
  Set runs: python3 benchmark_planners.py --runs 150
"""

import os
import csv
import math
import random
import sys
import threading
import time
import argparse
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
)
from moveit_msgs.msg import CollisionObject, RobotTrajectory, DisplayTrajectory
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest


# =========================
# CONFIGURATION
# =========================
@dataclass(frozen=True)
class PlannerSpec:
    planner_id: str
    label: str
    apply_shortcut: bool


PLANNER_CONFIGS: Tuple[PlannerSpec, ...] = (
    PlannerSpec("PRMstarkConfigDefault", "PRM*", False),
    PlannerSpec("RRTstarkConfigDefault", "RRT*", False),
    PlannerSpec("PRMkConfigDefault", "PRM+Shortcut", True),
    PlannerSpec("RRTkConfigDefault", "RRT+Shortcut", True),
)

# Benchmark parameters (will be overridden by command line args)
RUNS_PER_SCENARIO = 150
PLANNING_TIME = 10.0
NUM_ATTEMPTS = 3
RESULTS_DIR = os.path.expanduser("~/benchmarks")

# Robot positions
HELL_YEAH_JOINTS = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]

WAYPOINTS = {
    "Position_A": {
        "joints": [0.1316, 0.7100, -0.1201, -1.6096, -0.0627, 0.7987, 0.0298],
        "position": [0.57105, 0.00817, 0.25738],
    },
    "Position_B": {
        "joints": [-1.240, 0.702, -0.353, -1.661, 0.472, 0.424, 1.283],
        "position": [0.5, -0.3, 0.1],
    },
}

# Obstacle definitions
STATIC_OBSTACLES = [
    {
        "name": "static_box_1",
        "dimensions": [0.15, 0.15, 0.3],
        "position": [0.32, -0.62, 1.23],
    },
    {
        "name": "static_box_2",
        "dimensions": [0.1, 0.25, 0.25],
        "position": [0.32, 0.03, 1.10],
    },
]

# Dynamic obstacle (same position as static_box_1)
DYNAMIC_OBSTACLES = [
    {
        "name": "dynamic_box",
        "dimensions": [0.15, 0.15, 0.3],
        "position": [0.32, -0.62, 1.23],
    },
]

# Dynamic obstacle timing (seconds)
DYNAMIC_VISIBLE_MIN = 3.0
DYNAMIC_VISIBLE_MAX = 7.0
DYNAMIC_HIDDEN_MIN = 2.0
DYNAMIC_HIDDEN_MAX = 5.0


# =========================
# ARGUMENT PARSING
# =========================
def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Benchmark motion planning algorithms across different environments"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no RViz visualization, no execution, faster)",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of runs per environment (default: 1)",
    )
    args = parser.parse_args()
    return args


# =========================
# DYNAMIC OBSTACLE MANAGER
# =========================
class DynamicObstacleManager:
    def __init__(self, scene: PlanningSceneInterface, frame_id: str):
        self.scene = scene
        self.frame_id = frame_id
        self.is_running = False
        self.thread = None
        self.obstacle_present = False
        self.lock = threading.Lock()
        
    def _add_dynamic_obstacle(self):
        """Add the dynamic obstacle to the scene."""
        obs = DYNAMIC_OBSTACLES[0]
        co = CollisionObject()
        co.header.frame_id = self.frame_id
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

        self.scene.add_object(co)
        with self.lock:
            self.obstacle_present = True
        rospy.loginfo(f"[DYNAMIC] Added: {obs['name']} at {obs['position']}")
    
    def _remove_dynamic_obstacle(self):
        """Remove the dynamic obstacle from the scene."""
        obs = DYNAMIC_OBSTACLES[0]
        self.scene.remove_world_object(obs["name"])
        with self.lock:
            self.obstacle_present = False
        rospy.loginfo(f"[DYNAMIC] Removed: {obs['name']}")
    
    def is_obstacle_present(self) -> bool:
        """Check if the dynamic obstacle is currently present."""
        with self.lock:
            return self.obstacle_present
    
    def _run_loop(self):
        """Main loop for dynamic obstacle appearance/disappearance."""
        rospy.loginfo("[DYNAMIC] Starting dynamic obstacle loop...")
        
        while self.is_running and not rospy.is_shutdown():
            # APPEAR phase
            visible_duration = random.uniform(DYNAMIC_VISIBLE_MIN, DYNAMIC_VISIBLE_MAX)
            rospy.loginfo(f"[DYNAMIC] Obstacle APPEARING for {visible_duration:.1f}s")
            self._add_dynamic_obstacle()
            
            # Wait while visible
            sleep_start = time.time()
            while (time.time() - sleep_start) < visible_duration and self.is_running:
                time.sleep(0.1)
            
            if not self.is_running:
                break
            
            # DISAPPEAR phase
            hidden_duration = random.uniform(DYNAMIC_HIDDEN_MIN, DYNAMIC_HIDDEN_MAX)
            rospy.loginfo(f"[DYNAMIC] Obstacle DISAPPEARING for {hidden_duration:.1f}s")
            self._remove_dynamic_obstacle()
            
            # Wait while hidden
            sleep_start = time.time()
            while (time.time() - sleep_start) < hidden_duration and self.is_running:
                time.sleep(0.1)
        
        # Clean up when stopping
        self._remove_dynamic_obstacle()
        rospy.loginfo("[DYNAMIC] Dynamic obstacle loop stopped")
    
    def start(self):
        """Start the dynamic obstacle loop in a separate thread."""
        if self.is_running:
            rospy.logwarn("[DYNAMIC] Already running!")
            return
        
        self.is_running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        rospy.loginfo("[DYNAMIC] Dynamic obstacle manager started")
    
    def stop(self):
        """Stop the dynamic obstacle loop."""
        if not self.is_running:
            return
        
        rospy.loginfo("[DYNAMIC] Stopping dynamic obstacle manager...")
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self._remove_dynamic_obstacle()  # Ensure cleanup
        rospy.loginfo("[DYNAMIC] Dynamic obstacle manager stopped")


# =========================
# HELPER FUNCTIONS
# =========================
def joint_path_length(points: List) -> float:
    """Calculate the total joint-space path length."""
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(
            sum((a - b) ** 2 for a, b in zip(first.positions, second.positions))
        )
    return total


def to_robot_trajectory(plan: object) -> Optional[RobotTrajectory]:
    """Convert various plan formats to RobotTrajectory."""
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
        as_traj.multi_dof_joint_trajectory = getattr(
            plan, "multi_dof_joint_trajectory", None
        )
        return as_traj
    return None


def add_obstacles(scene: PlanningSceneInterface, obstacles: List[Dict], frame_id: str = "world"):
    """Add obstacles to the planning scene."""
    rospy.sleep(0.5)
    for obs in obstacles:
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
        rospy.loginfo(f"Added obstacle: {obs['name']} at {obs['position']}")
    rospy.sleep(0.5)


def remove_obstacles(scene: PlanningSceneInterface, obstacles: List[Dict]):
    """Remove obstacles from the planning scene."""
    for obs in obstacles:
        scene.remove_world_object(obs["name"])
    rospy.sleep(0.3)


# =========================
# PLANNER CLASS
# =========================
class PlannerBenchmark:
    def __init__(self, group_name: str, planning_time: float, num_attempts: int, headless: bool = False):
        rospy.loginfo("Initializing PlannerBenchmark...")
        roscpp_initialize(sys.argv)
        rospy.init_node("planner_benchmark_full", anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(group_name)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(num_attempts)
        self.headless = headless

        if not headless:
            self.display_pub = rospy.Publisher(
                "move_group/display_planned_path", DisplayTrajectory, queue_size=10
            )
        else:
            self.display_pub = None
            rospy.loginfo("HEADLESS MODE: Visualization and execution disabled")

        rospy.loginfo(f"Planning group: {group_name}")
        rospy.loginfo(f"Planning frame: {self.group.get_planning_frame()}")
        rospy.loginfo(f"End-effector: {self.group.get_end_effector_link()}")

        # Setup state validity service for shortcutting and trajectory validation
        self._state_validity_srv: Optional = None
        try:
            rospy.wait_for_service("check_state_validity", timeout=5.0)
            self._state_validity_srv = rospy.ServiceProxy(
                "check_state_validity", GetStateValidity
            )
            rospy.loginfo("State validity service available (shortcutting + validation enabled)")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(f"State validity service disabled: {exc}")

    def plan_with_planner(
        self,
        planner_id: str,
        target_joints: List[float],
        planning_time: float,
        attempts: int,
    ) -> Tuple[bool, Optional[RobotTrajectory], float]:
        """Plan to a joint target with specified planner."""
        self.group.set_start_state_to_current_state()
        self.group.set_planner_id(planner_id)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(attempts)
        self.group.set_joint_value_target(target_joints)

        start_time = rospy.get_time()
        plan_result = self.group.plan()
        wall_time = rospy.get_time() - start_time

        trajectory = to_robot_trajectory(plan_result)
        success = bool(trajectory and trajectory.joint_trajectory.points)

        self.group.clear_pose_targets()
        return success, trajectory, wall_time

    def _state_is_valid(
        self, joint_names, positions, state_template, index_map
    ) -> bool:
        """Check if a joint state is collision-free."""
        if self._state_validity_srv is None:
            return False

        state_copy = deepcopy(state_template)
        positions_list = list(state_copy.joint_state.position)
        for name, value in zip(joint_names, positions):
            idx = index_map.get(name)
            if idx is None:
                return False
            positions_list[idx] = value
        state_copy.joint_state.position = tuple(positions_list)

        request = GetStateValidityRequest()
        request.group_name = self.group.get_name()
        request.robot_state = state_copy

        try:
            response = self._state_validity_srv(request)
            return bool(response.valid)
        except rospy.ServiceException:
            self._state_validity_srv = None
            return False

    def _segment_is_valid(
        self, joint_names, start, end, state_template, index_map, samples: int = 12
    ) -> bool:
        """Check if a straight-line segment in joint space is collision-free."""
        for step in range(1, samples):
            ratio = step / float(samples)
            interp = [s + ratio * (e - s) for s, e in zip(start, end)]
            if not self._state_is_valid(joint_names, interp, state_template, index_map):
                return False
        return True

    def is_trajectory_valid(self, trajectory: RobotTrajectory) -> bool:
        """
        Validate entire trajectory for collisions.
        Critical for dynamic environments where obstacles may have appeared after planning.
        """
        if self._state_validity_srv is None or not trajectory:
            return True  # Assume valid if we can't check

        joint_names = list(trajectory.joint_trajectory.joint_names)
        state_template = self.robot.get_current_state()
        index_map = {
            name: idx for idx, name in enumerate(state_template.joint_state.name)
        }

        # Check each waypoint in trajectory
        for point in trajectory.joint_trajectory.points:
            if not self._state_is_valid(joint_names, point.positions, state_template, index_map):
                return False
        
        return True

    def shortcut_trajectory(
        self, trajectory: RobotTrajectory, max_iterations: int = 80
    ) -> Tuple[RobotTrajectory, float, bool]:
        """Apply random shortcutting to reduce path length."""
        if self._state_validity_srv is None or not trajectory:
            points = trajectory.joint_trajectory.points if trajectory else []
            return trajectory, joint_path_length(points), False

        original_points = trajectory.joint_trajectory.points
        original_length = joint_path_length(original_points)

        if len(original_points) < 3 or original_length <= 0.0:
            return trajectory, original_length, False

        working = deepcopy(trajectory)
        points = working.joint_trajectory.points
        joint_names = list(working.joint_trajectory.joint_names)

        state_template = self.robot.get_current_state()
        index_map = {
            name: idx for idx, name in enumerate(state_template.joint_state.name)
        }

        improved = False
        for _ in range(max_iterations):
            if len(points) < 3:
                break
            first_idx = random.randrange(0, len(points) - 2)
            second_idx = random.randrange(first_idx + 2, len(points))

            start = points[first_idx].positions
            end = points[second_idx].positions

            if self._segment_is_valid(
                joint_names, start, end, state_template, index_map
            ):
                del points[first_idx + 1 : second_idx]
                improved = True

        new_length = joint_path_length(points)
        if improved:
            reduction_pct = (
                (1 - new_length / original_length) * 100.0 if original_length else 0.0
            )
            rospy.loginfo(
                f"Shortcutting: {original_length:.4f} → {new_length:.4f} ({reduction_pct:.1f}% reduction)"
            )

        return working, new_length, improved

    def benchmark_motion(
        self, from_name: str, to_name: str, to_joints: List[float]
    ) -> List[Dict]:
        """Benchmark all planners for a single motion."""
        results = []

        for spec in PLANNER_CONFIGS:
            rospy.loginfo(f"  Testing {spec.label} ({spec.planner_id})...")

            success, trajectory, wall_time = self.plan_with_planner(
                spec.planner_id, to_joints, PLANNING_TIME, NUM_ATTEMPTS
            )

            if not success or trajectory is None:
                rospy.logwarn(f"  {spec.label} FAILED")
                results.append(
                    {
                        "planner": spec.label,
                        "planner_id": spec.planner_id,
                        "success": False,
                        "planning_time": wall_time,
                        "waypoints": 0,
                        "path_length": float("inf"),
                        "shortcut_applied": False,
                        "shortcut_reduction": 0.0,
                    }
                )
                continue

            initial_length = joint_path_length(trajectory.joint_trajectory.points)
            final_length = initial_length
            shortcut_applied = False
            reduction = 0.0

            if spec.apply_shortcut:
                processed, final_length, shortened = self.shortcut_trajectory(trajectory)
                trajectory = processed
                shortcut_applied = shortened
                if shortened:
                    reduction = initial_length - final_length

            rospy.loginfo(
                f"  {spec.label} SUCCESS: time={wall_time:.3f}s, "
                f"waypoints={len(trajectory.joint_trajectory.points)}, "
                f"length={final_length:.4f}"
            )

            results.append(
                {
                    "planner": spec.label,
                    "planner_id": spec.planner_id,
                    "success": True,
                    "planning_time": wall_time,
                    "waypoints": len(trajectory.joint_trajectory.points),
                    "path_length": final_length,
                    "shortcut_applied": shortcut_applied,
                    "shortcut_reduction": reduction,
                    "trajectory": trajectory,
                }
            )

        return results

    def execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        """
        Execute a planned trajectory.
        Returns True if executed, False if skipped due to collision.
        """
        # CRITICAL: Re-validate trajectory before execution (for dynamic obstacles)
        if not self.is_trajectory_valid(trajectory):
            rospy.logwarn("⚠️  Trajectory is now INVALID (obstacle appeared). Skipping execution!")
            return False
        
        if self.headless:
            # In headless mode, just move to the target directly
            target_joints = trajectory.joint_trajectory.points[-1].positions
            self.group.set_joint_value_target(target_joints)
            self.group.go(wait=True)
            self.group.stop()
        else:
            # Normal execution
            self.group.set_start_state_to_current_state()
            self.group.execute(trajectory, wait=True)
            self.group.stop()
        
        return True

    def display_trajectory(self, trajectory: RobotTrajectory):
        """Publish trajectory for RViz visualization."""
        if self.headless or self.display_pub is None:
            return
        
        msg = DisplayTrajectory()
        msg.trajectory_start = self.robot.get_current_state()
        msg.trajectory.append(trajectory)
        self.display_pub.publish(msg)

    def shutdown(self):
        """Clean shutdown."""
        roscpp_shutdown()


# =========================
# CSV LOGGING
# =========================
def write_csv_header(csv_path: str):
    """Write CSV header."""
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp",
                "environment",
                "run",
                "motion",
                "from_waypoint",
                "to_waypoint",
                "planner",
                "planner_id",
                "success",
                "planning_time_sec",
                "waypoint_count",
                "path_length",
                "shortcut_applied",
                "shortcut_reduction",
                "execution_skipped",
            ]
        )


def append_csv_row(csv_path: str, row_data: Dict):
    """Append a result row to CSV."""
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                row_data["timestamp"],
                row_data["environment"],
                row_data["run"],
                row_data["motion"],
                row_data["from_waypoint"],
                row_data["to_waypoint"],
                row_data["planner"],
                row_data["planner_id"],
                row_data["success"],
                f"{row_data['planning_time']:.4f}",
                row_data["waypoints"],
                f"{row_data['path_length']:.6f}",
                row_data["shortcut_applied"],
                f"{row_data['shortcut_reduction']:.6f}",
                row_data.get("execution_skipped", False),
            ]
        )


# =========================
# MAIN BENCHMARK
# =========================
def main():
    args = parse_arguments()
    
    global RUNS_PER_SCENARIO
    RUNS_PER_SCENARIO = args.runs
    
    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(RESULTS_DIR, f"benchmark_results_{timestamp_str}.csv")

    write_csv_header(csv_path)
    rospy.loginfo(f"Results will be saved to: {csv_path}")
    rospy.loginfo(f"Runs per environment: {RUNS_PER_SCENARIO}")
    rospy.loginfo(f"Headless mode: {args.headless}")

    benchmark = PlannerBenchmark("arm", PLANNING_TIME, NUM_ATTEMPTS, headless=args.headless)

    # Define test path: hell_yeah->A->hell_yeah->B->hell_yeah->A
    test_path = [
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_A", WAYPOINTS["Position_A"]["joints"]),
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_B", WAYPOINTS["Position_B"]["joints"]),
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_A", WAYPOINTS["Position_A"]["joints"]),
    ]

    # Environment scenarios
    environments = [
        ("no_obstacles", []),
        ("static_obstacles", STATIC_OBSTACLES),
        ("dynamic_obstacles", None),  # Special case: managed by DynamicObstacleManager
    ]

    for env_name, obstacles in environments:
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"ENVIRONMENT: {env_name.upper()}")
        rospy.loginfo(f"{'='*60}")

        # Handle dynamic obstacles
        dynamic_manager = None
        if env_name == "dynamic_obstacles":
            rospy.loginfo("Starting dynamic obstacle manager...")
            dynamic_manager = DynamicObstacleManager(
                benchmark.scene, 
                benchmark.group.get_planning_frame()
            )
            dynamic_manager.start()
            rospy.sleep(2.0)  # Give it time to start
        elif obstacles:
            add_obstacles(benchmark.scene, obstacles, benchmark.group.get_planning_frame())

        for run in range(1, RUNS_PER_SCENARIO + 1):
            rospy.loginfo(f"\n--- Run {run}/{RUNS_PER_SCENARIO} ---")

            # Reset to start position
            rospy.loginfo("Resetting to hell_yeah position...")
            benchmark.group.set_joint_value_target(HELL_YEAH_JOINTS)
            benchmark.group.go(wait=True)
            benchmark.group.stop()
            rospy.sleep(1.0)

            # Execute test path
            for motion_idx in range(len(test_path) - 1):
                from_name, _ = test_path[motion_idx]
                to_name, to_joints = test_path[motion_idx + 1]

                rospy.loginfo(f"\nMotion {motion_idx + 1}: {from_name} → {to_name}")

                results = benchmark.benchmark_motion(from_name, to_name, to_joints)

                # Execute best trajectory
                successful = [r for r in results if r["success"]]
                execution_skipped = False
                
                if successful:
                    best = min(successful, key=lambda x: x["path_length"])
                    rospy.loginfo(f"Executing best: {best['planner']}")
                    benchmark.display_trajectory(best["trajectory"])
                    
                    # Execute and check if it was skipped due to dynamic collision
                    executed = benchmark.execute_trajectory(best["trajectory"])
                    if not executed:
                        execution_skipped = True
                        rospy.logwarn("Execution skipped! Using fallback go()")
                        benchmark.group.set_joint_value_target(to_joints)
                        benchmark.group.go(wait=True)
                        benchmark.group.stop()
                    
                    if not args.headless:
                        rospy.sleep(0.5)
                else:
                    rospy.logwarn("All planners failed! Using fallback go()")
                    benchmark.group.set_joint_value_target(to_joints)
                    benchmark.group.go(wait=True)
                    benchmark.group.stop()

                # Log to CSV
                for result in results:
                    append_csv_row(
                        csv_path,
                        {
                            "timestamp": datetime.now().isoformat(),
                            "environment": env_name,
                            "run": run,
                            "motion": motion_idx + 1,
                            "from_waypoint": from_name,
                            "to_waypoint": to_name,
                            "planner": result["planner"],
                            "planner_id": result["planner_id"],
                            "success": result["success"],
                            "planning_time": result["planning_time"],
                            "waypoints": result["waypoints"],
                            "path_length": result["path_length"],
                            "shortcut_applied": result.get("shortcut_applied", False),
                            "shortcut_reduction": result.get("shortcut_reduction", 0.0),
                            "execution_skipped": execution_skipped,
                        },
                    )

        # Cleanup
        if dynamic_manager:
            dynamic_manager.stop()
        elif obstacles:
            remove_obstacles(benchmark.scene, obstacles)

    benchmark.shutdown()
    rospy.loginfo(f"\n{'='*60}")
    rospy.loginfo(f"Benchmark complete! Results saved to:")
    rospy.loginfo(f"  {csv_path}")
    rospy.loginfo(f"{'='*60}")


if __name__ == "__main__":
    main()
