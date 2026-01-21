#!/usr/bin/env python3

"""
Usage:
  rosrun group1 benchmark_planner_run.py
"""

import os
import csv
import math
import random
import sys
import threading
import time
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
)
from moveit_msgs.msg import CollisionObject, RobotTrajectory
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest


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

# Benchmark parameters
RUNS_PER_SCENARIO = 150
PLANNING_TIME = 10.0
NUM_ATTEMPTS = 3
RESULTS_DIR = os.path.expanduser("~/benchmarks")

HELL_YEAH_JOINTS = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]

WAYPOINTS = {
    "Position_A": {
        "joints": [0.1316, 0.7100, -0.1201, -1.6096, -0.0627, 0.7987, 0.0298],
    },
    "Position_B": {
        "joints": [-1.240, 0.702, -0.353, -1.661, 0.472, 0.424, 1.283],
    },
}

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

DYNAMIC_OBSTACLES = [
    {
        "name": "dynamic_box",
        "dimensions": [0.15, 0.15, 0.3],
        "position": [0.32, -0.62, 1.23],
    },
]

DYNAMIC_VISIBLE_MIN = 3.0
DYNAMIC_VISIBLE_MAX = 7.0
DYNAMIC_HIDDEN_MIN = 2.0
DYNAMIC_HIDDEN_MAX = 5.0


def suppress_ros_logging():
    """Suppress ROS logging to terminal."""
    import logging
    logging.getLogger('rosout').setLevel(logging.ERROR)
    rospy.set_param('/move_group/trajectory_execution/execution_duration_monitoring', False)


class DynamicObstacleManager:
    def __init__(self, scene: PlanningSceneInterface, frame_id: str):
        self.scene = scene
        self.frame_id = frame_id
        self.is_running = False
        self.thread = None
        self.obstacle_present = False
        self.lock = threading.Lock()
        
    def _add_dynamic_obstacle(self):
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
    
    def _remove_dynamic_obstacle(self):
        obs = DYNAMIC_OBSTACLES[0]
        self.scene.remove_world_object(obs["name"])
        with self.lock:
            self.obstacle_present = False
    
    def is_obstacle_present(self) -> bool:
        with self.lock:
            return self.obstacle_present
    
    def _run_loop(self):
        while self.is_running and not rospy.is_shutdown():
            # APPEAR
            visible_duration = random.uniform(DYNAMIC_VISIBLE_MIN, DYNAMIC_VISIBLE_MAX)
            self._add_dynamic_obstacle()
            
            sleep_start = time.time()
            while (time.time() - sleep_start) < visible_duration and self.is_running:
                time.sleep(0.1)
            
            if not self.is_running:
                break
            
            # DISAPPEAR
            hidden_duration = random.uniform(DYNAMIC_HIDDEN_MIN, DYNAMIC_HIDDEN_MAX)
            self._remove_dynamic_obstacle()
            
            sleep_start = time.time()
            while (time.time() - sleep_start) < hidden_duration and self.is_running:
                time.sleep(0.1)
        
        self._remove_dynamic_obstacle()
    
    def start(self):
        if self.is_running:
            return
        
        self.is_running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        if not self.is_running:
            return
        
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self._remove_dynamic_obstacle()


def joint_path_length(points: List) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(
            sum((a - b) ** 2 for a, b in zip(first.positions, second.positions))
        )
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
        as_traj.multi_dof_joint_trajectory = getattr(
            plan, "multi_dof_joint_trajectory", None
        )
        return as_traj
    return None


def add_obstacles(scene: PlanningSceneInterface, obstacles: List[Dict], frame_id: str = "world"):
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
    rospy.sleep(0.5)


def remove_obstacles(scene: PlanningSceneInterface, obstacles: List[Dict]):
    for obs in obstacles:
        scene.remove_world_object(obs["name"])
    rospy.sleep(0.3)


class PlannerBenchmark:
    def __init__(self, group_name: str, planning_time: float, num_attempts: int):
        suppress_ros_logging()
        roscpp_initialize(sys.argv)
        rospy.init_node("planner_benchmark_silent", anonymous=True, log_level=rospy.ERROR)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(group_name)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(num_attempts)

        # Setup state validity service
        self._state_validity_srv: Optional = None
        try:
            rospy.wait_for_service("check_state_validity", timeout=5.0)
            self._state_validity_srv = rospy.ServiceProxy(
                "check_state_validity", GetStateValidity
            )
        except (rospy.ROSException, rospy.ServiceException):
            pass

    def plan_with_planner(
        self,
        planner_id: str,
        target_joints: List[float],
        planning_time: float,
        attempts: int,
    ) -> Tuple[bool, Optional[RobotTrajectory], float]:
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

    def _state_is_valid(self, joint_names, positions, state_template, index_map) -> bool:
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

    def _segment_is_valid(self, joint_names, start, end, state_template, index_map, samples: int = 12) -> bool:
        for step in range(1, samples):
            ratio = step / float(samples)
            interp = [s + ratio * (e - s) for s, e in zip(start, end)]
            if not self._state_is_valid(joint_names, interp, state_template, index_map):
                return False
        return True

    def is_trajectory_valid(self, trajectory: RobotTrajectory) -> bool:
        if self._state_validity_srv is None or not trajectory:
            return True

        joint_names = list(trajectory.joint_trajectory.joint_names)
        state_template = self.robot.get_current_state()
        index_map = {
            name: idx for idx, name in enumerate(state_template.joint_state.name)
        }

        for point in trajectory.joint_trajectory.points:
            if not self._state_is_valid(joint_names, point.positions, state_template, index_map):
                return False
        
        return True

    def shortcut_trajectory(self, trajectory: RobotTrajectory, max_iterations: int = 80) -> Tuple[RobotTrajectory, float, bool]:
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

            if self._segment_is_valid(joint_names, start, end, state_template, index_map):
                del points[first_idx + 1 : second_idx]
                improved = True

        new_length = joint_path_length(points)
        return working, new_length, improved

    def benchmark_motion(self, from_name: str, to_name: str, to_joints: List[float]) -> List[Dict]:
        results = []

        for spec in PLANNER_CONFIGS:
            success, trajectory, wall_time = self.plan_with_planner(
                spec.planner_id, to_joints, PLANNING_TIME, NUM_ATTEMPTS
            )

            if not success or trajectory is None:
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
        if not self.is_trajectory_valid(trajectory):
            return False
        
        target_joints = trajectory.joint_trajectory.points[-1].positions
        self.group.set_joint_value_target(target_joints)
        self.group.go(wait=True)
        self.group.stop()
        
        return True

    def shutdown(self):
        roscpp_shutdown()


def write_csv_header(csv_path: str):
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


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(RESULTS_DIR, f"benchmark_results_{timestamp_str}.csv")

    write_csv_header(csv_path)
    
    print(f"\n{'='*60}")
    print(f"SILENT BENCHMARK MODE")
    print(f"{'='*60}")
    print(f"Runs per environment: {RUNS_PER_SCENARIO}")
    print(f"Results: {csv_path}")
    print(f"{'='*60}\n")

    benchmark = PlannerBenchmark("arm", PLANNING_TIME, NUM_ATTEMPTS)

    test_path = [
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_A", WAYPOINTS["Position_A"]["joints"]),
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_B", WAYPOINTS["Position_B"]["joints"]),
        ("hell_yeah", HELL_YEAH_JOINTS),
        ("Position_A", WAYPOINTS["Position_A"]["joints"]),
    ]

    environments = [
        ("no_obstacles", "No Obstacles", []),
        ("static_obstacles", "Static Obstacles", STATIC_OBSTACLES),
        ("dynamic_obstacles", "Dynamic Obstacles", None),
    ]

    start_time = time.time()

    for env_idx, (env_name, env_display, obstacles) in enumerate(environments, 1):
        dynamic_manager = None
        if env_name == "dynamic_obstacles":
            dynamic_manager = DynamicObstacleManager(
                benchmark.scene, 
                benchmark.group.get_planning_frame()
            )
            dynamic_manager.start()
            rospy.sleep(2.0)
        elif obstacles:
            add_obstacles(benchmark.scene, obstacles, benchmark.group.get_planning_frame())

        for run in range(1, RUNS_PER_SCENARIO + 1):
            benchmark.group.set_joint_value_target(HELL_YEAH_JOINTS)
            benchmark.group.go(wait=True)
            benchmark.group.stop()
            rospy.sleep(0.5)

            for motion_idx in range(len(test_path) - 1):
                from_name, _ = test_path[motion_idx]
                to_name, to_joints = test_path[motion_idx + 1]

                results = benchmark.benchmark_motion(from_name, to_name, to_joints)

                successful = [r for r in results if r["success"]]
                execution_skipped = False
                
                if successful:
                    best = min(successful, key=lambda x: x["path_length"])
                    executed = benchmark.execute_trajectory(best["trajectory"])
                    if not executed:
                        execution_skipped = True
                        benchmark.group.set_joint_value_target(to_joints)
                        benchmark.group.go(wait=True)
                        benchmark.group.stop()
                else:
                    benchmark.group.set_joint_value_target(to_joints)
                    benchmark.group.go(wait=True)
                    benchmark.group.stop()


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

            elapsed = time.time() - start_time
            print(f"[Env {env_idx}/3: {env_display}] Run {run}/{RUNS_PER_SCENARIO} complete ({elapsed/60:.1f} min elapsed)")

        # cleanup
        if dynamic_manager:
            dynamic_manager.stop()
        elif obstacles:
            remove_obstacles(benchmark.scene, obstacles)

    benchmark.shutdown()
    
    total_time = time.time() - start_time
    
    # ONLY final summary
    print(f"\n{'='*60}")
    print(f"BENCHMARK COMPLETE!")
    print(f"{'='*60}")
    print(f"Total time: {total_time/3600:.2f} hours ({total_time/60:.1f} minutes)")
    print(f"Total runs: {RUNS_PER_SCENARIO * 3}")
    print(f"Results saved: {csv_path}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
