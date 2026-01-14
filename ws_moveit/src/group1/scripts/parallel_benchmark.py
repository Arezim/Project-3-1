#!/usr/bin/env python3
"""
Planner Benchmark: Tests multiple OMPL planners on a cyclic path:
    hell_yeah -> Position A -> Position B -> hell_yeah
    
With optional obstacles to simulate real-world scenarios.

Uses the planning logic from compare_planners.py for consistency with actual robot operations.
"""
import os
import csv
import math
import threading
from typing import Dict, List, Optional, Tuple

import rospy
from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, 
    RobotCommander, MoveGroupCommander, PlanningSceneInterface
)
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject, RobotTrajectory, DisplayTrajectory
)

# CONFIGURATION
PLANNERS: Dict[str, str] = {
    "RRTConnectkConfigDefault": "RRTConnect",
    "PRMkConfigDefault": "PRM",
    "RRTstarkConfigDefault": "RRT*",
}

RUNS = 1  # Number of complete cycles through all positions
TIME_LIMIT = 10.0  # Planning time per motion (seconds)
NUM_ATTEMPTS = 3  # Number of planning attempts per planner
RESULTS_DIR = os.path.expanduser("~/benchmarks")

# Enable/disable obstacles
USE_OBSTACLES = True

# Use joint targets instead of pose targets
USE_JOINT_TARGETS = True

# hell_yeah joint configuration (from SRDF)
# [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
HELL_YEAH_JOINTS = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]

# Position A: Custom position from RViz (reaching to the left/back side)
POSITION_A = {
    "name": "Position_A",
    "position": [0.5, -0.3, 0.1],  # Approximate Cartesian (not used with joint targets)
    "orientation": [0.0, 0.0, 0.0, 1.0],
    # Exact joint values from RViz
    "joints": [-1.240, 0.702, -0.353, -1.661, 0.472, 0.424, 1.283],
}

# Position B: Custom position from RViz (reaching forward)
POSITION_B = {
    "name": "Position_B",
    "position": [0.5, 0.3, 0.1],  # Approximate Cartesian (not used with joint targets)
    "orientation": [0.0, 0.0, 0.0, 1.0],
    # Exact joint values from RViz
    "joints": [0.029, 0.883, -0.062, -1.423, -0.046, 0.340, 0.057],
}

# Obstacles
OBSTACLES = [
    {
        "name": "obstacle_A_to_B",
        "type": "box",
        "dimensions": [0.1, 0.2, 0.2],  # Size: 10cm x 20cm x 20cm
        "position": [0.25, -0.7, 1.25],  # Placed between A and B
    },
    {
        "name": "obstacle_near_B",
        "type": "box", 
        "dimensions": [0.15, 0.15, 0.5],  # Tall thin obstacle (useless for now)
        "position": [0.45, 0.15, 0.8],
    },
]


def joint_path_length(points: List) -> float:
    """Calculate total joint-space path length."""
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        total += math.sqrt(
            sum((a - b) ** 2 for a, b in zip(first.positions, second.positions))
        )
    return total


def to_robot_trajectory(plan: object) -> Optional[RobotTrajectory]:
    """
    Normalize MoveGroupCommander.plan results to RobotTrajectory.
    Handles different return types from different MoveIt versions.
    """
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


def cartesian_path_length(pts, group):
    """Estimate cartesian path length of end-effector (approximate)."""
    # This would require FK for each point - simplified version
    return len(pts)  # Just return number of waypoints as proxy


def wait_for_param(name: str, timeout: float = 30.0) -> bool:
    """Wait for a ROS parameter to become available."""
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
    """
    Plan with a specific planner to a target.
    
    Args:
        group: MoveGroupCommander instance
        planner_id: OMPL planner ID (e.g., "RRTConnectkConfigDefault")
        target: Dict with 'type' and target data ('joints', 'pose', or 'name')
        planning_time: Maximum planning time in seconds
        num_attempts: Number of planning attempts
    
    Returns:
        Tuple of (success, trajectory, wall_time, num_points, path_length)
    """
    # Configure planner
    group.set_start_state_to_current_state()
    group.set_planner_id(planner_id)
    group.set_planning_time(planning_time)
    group.set_num_planning_attempts(num_attempts)
    
    # Set goal based on target type
    target_type = target.get("type", "joints")
    if target_type == "named":
        group.set_named_target(target["name"])
    elif target_type == "joints":
        group.set_joint_value_target(target["joints"])
    elif target_type == "pose":
        group.set_pose_target(target["pose"])
    else:
        rospy.logwarn(f"Unknown target type: {target_type}")
        return False, None, 0.0, 0, 0.0
    
    # Plan and measure time
    start_time = rospy.get_time()
    plan_result = group.plan()
    wall_time = rospy.get_time() - start_time
    
    # Normalize trajectory using compare_planners logic
    trajectory = to_robot_trajectory(plan_result)
    success = bool(trajectory and trajectory.joint_trajectory.points)
    
    # Calculate metrics
    num_points = 0
    path_length = 0.0
    if success and trajectory:
        points = trajectory.joint_trajectory.points
        num_points = len(points)
        path_length = joint_path_length(points)
    
    # Clear targets
    group.clear_pose_targets()
    
    return success, trajectory, wall_time, num_points, path_length


def create_pose(position: List[float], orientation: List[float]) -> Pose:
    """Create a Pose message from position [x,y,z] and orientation [x,y,z,w]."""
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def add_obstacles(scene, frame_id="world"):
    """Add obstacle boxes to the planning scene."""
    rospy.sleep(0.5)  # Let the scene initialize
    
    for obs in OBSTACLES:
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = obs["name"]
        
        # Create box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = obs["dimensions"]
        
        # Set pose
        box_pose = Pose()
        box_pose.position.x = obs["position"][0]
        box_pose.position.y = obs["position"][1]
        box_pose.position.z = obs["position"][2]
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        scene.add_object(collision_object)
        print(f"  Added obstacle: {obs['name']} at {obs['position']}")
    
    rospy.sleep(0.5)  # Let obstacles propagate


def remove_obstacles(scene):
    """Remove all benchmark obstacles from the scene."""
    for obs in OBSTACLES:
        scene.remove_world_object(obs["name"])
    rospy.sleep(0.3)


# MAIN BENCHMARK
def main():
    roscpp_initialize([])
    rospy.init_node("parallel_planning_benchmark", anonymous=True)
    
    print("[benchmark] Starting parallel planning benchmark...")
    print(f"[benchmark] Planners: {list(PLANNERS.values())}")
    print(f"[benchmark] Planning time: {TIME_LIMIT}s, Attempts: {NUM_ATTEMPTS}")

    # Detect namespace (check for iiwa namespace first, then default)
    ns = ""
    desc = "robot_description"
    if rospy.has_param("/iiwa/robot_description"):
        ns = "/iiwa"
        desc = "/iiwa/robot_description"
    
    print(f"[benchmark] Using robot_description: {desc}")

    # Wait for model
    if not wait_for_param(desc, 30.0):
        raise RuntimeError(f"Timeout waiting for {desc}. Is MoveIt running?")

    robot = RobotCommander(robot_description=desc, ns=ns)
    scene = PlanningSceneInterface(ns=ns)
    groups = robot.get_group_names()
    
    # Find the arm group
    group_name = "arm" if "arm" in groups else ("manipulator" if "manipulator" in groups else groups[0])
    print(f"[benchmark] Using planning group: {group_name}")
    print(f"[benchmark] Available groups: {groups}")
    
    # Initialize move group
    group = MoveGroupCommander(group_name, ns=ns)
    ee_link = group.get_end_effector_link()
    planning_frame = group.get_planning_frame()
    print(f"[benchmark] End effector: {ee_link}")
    print(f"[benchmark] Planning frame: {planning_frame}")
    
    # Setup trajectory display publisher (like compare_planners.py)
    display_pub = rospy.Publisher(
        "move_group/display_planned_path", DisplayTrajectory, queue_size=10
    )
    
    # Print current joint values for debugging
    current_joints = group.get_current_joint_values()
    print(f"[benchmark] Current joint values: {[f'{j:.3f}' for j in current_joints]}")
    
    # Try to get known planner IDs
    try:
        known_planners = group.get_known_planner_ids()
        print(f"[benchmark] Known planner IDs: {known_planners[:10]}..." if len(known_planners) > 10 else f"[benchmark] Known planner IDs: {known_planners}")
    except Exception as e:
        rospy.logwarn(f"Could not get known planner IDs: {e}")
    
    # Setup results directory
    os.makedirs(RESULTS_DIR, exist_ok=True)
    import time as time_module
    stamp = time_module.strftime("%Y%m%d-%H%M%S")
    out_csv = os.path.join(RESULTS_DIR, f"bench_{group_name}_{stamp}.csv")

    # Add obstacles to the scene (only if enabled)
    if USE_OBSTACLES:
        print("\n=== Adding Obstacles ===")
        add_obstacles(scene, frame_id=planning_frame)
    else:
        print("\n=== Obstacles DISABLED ===")
    
    # Define the waypoint sequence - full cyclic path
    if USE_JOINT_TARGETS:
        waypoints = [
            {"name": "hell_yeah", "type": "joints", "joints": HELL_YEAH_JOINTS},
            {"name": POSITION_A["name"], "type": "joints", "joints": POSITION_A["joints"]},
            {"name": POSITION_B["name"], "type": "joints", "joints": POSITION_B["joints"]},
            {"name": "hell_yeah", "type": "joints", "joints": HELL_YEAH_JOINTS},
        ]
        print("\n=== Using JOINT targets ===")
    else:
        waypoints = [
            {"name": "hell_yeah", "type": "joints", "joints": HELL_YEAH_JOINTS},
            {"name": POSITION_A["name"], "type": "pose", "pose": create_pose(POSITION_A["position"], POSITION_A["orientation"])},
            {"name": POSITION_B["name"], "type": "pose", "pose": create_pose(POSITION_B["position"], POSITION_B["orientation"])},
            {"name": "hell_yeah", "type": "joints", "joints": HELL_YEAH_JOINTS},
        ]
        print("\n=== Using POSE targets (Cartesian) ===")
    
    print(f"\n=== Benchmark Path ===")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. {wp['name']}")

    for run in range(1, RUNS + 1):
        print(f"\n{'='*60}")
        print(f"RUN {run}/{RUNS}")
        print(f"{'='*60}")
        
        # Check if robot is at hell_yeah position
        current = group.get_current_joint_values()
        print(f"\n>>> Checking starting position...")
        print(f"    Current joints:   {[f'{j:.3f}' for j in current]}")
        print(f"    Expected (hell_yeah): {[f'{j:.3f}' for j in HELL_YEAH_JOINTS]}")
        
        # Check if close enough to hell_yeah
        diff = sum(abs(c - h) for c, h in zip(current, HELL_YEAH_JOINTS))
        if diff > 0.1:
            print(f"\n    WARNING: Robot is NOT at hell_yeah position! (diff={diff:.3f} rad)")
            print(f"    Continuing anyway...")
        else:
            print(f"    ✓ Robot is at hell_yeah position (diff={diff:.3f} rad)")
        
        rospy.sleep(0.5)
        
        # Execute each motion in the sequence
        for motion_idx in range(len(waypoints) - 1):
            from_wp = waypoints[motion_idx]
            to_wp = waypoints[motion_idx + 1]
            motion_name = f"{from_wp['name']} -> {to_wp['name']}"
            
            print(f"\n--- Motion {motion_idx + 1}: {motion_name} ---")
            
            # Dictionary to store results from parallel planning
            results: Dict[str, dict] = {}
            threads: List[threading.Thread] = []
            results_lock = threading.Lock()
            
            def try_plan_thread(planner_id: str, display_name: str):
                """Plan with a specific planner in a separate thread."""
                # Create a new MoveGroupCommander for thread safety
                g = MoveGroupCommander(group_name, ns=ns)
                
                # Use the unified planning function
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
                        "trajectory": trajectory
                    }
            
            # Run all planners in parallel
            for planner_id, display_name in PLANNERS.items():
                t = threading.Thread(target=try_plan_thread, args=(planner_id, display_name))
                t.start()
                threads.append(t)
            
            # Wait for all planners to complete
            for t in threads:
                t.join()
            
            # Display results
            print(f"\n  Results:")
            for planner_id, res in results.items():
                status = "✓" if res["success"] else "✗"
                print(f"    {res['display_name']:12} [{planner_id}]: {status} "
                      f"time={res['time']:.3f}s, waypoints={res['points']}, path_len={res['length']:.4f}")
            
            # Select and execute the best successful plan
            successful = [(pid, res) for pid, res in results.items() if res["success"]]
            
            if successful:
                # Pick shortest path among successful planners
                best_pid, best_res = min(successful, key=lambda x: x[1]["length"])
                print(f"\n  Selected: {best_res['display_name']} (shortest path: {best_res['length']:.4f})")
                
                # Publish trajectory for visualization (like compare_planners.py)
                if best_res["trajectory"]:
                    display_msg = DisplayTrajectory()
                    display_msg.trajectory_start = robot.get_current_state()
                    display_msg.trajectory.append(best_res["trajectory"])
                    display_pub.publish(display_msg)
                    rospy.sleep(0.5)  # Let RViz update
                
                # Execute the trajectory
                print(f"  Executing trajectory...")
                group.set_start_state_to_current_state()
                group.execute(best_res["trajectory"], wait=True)
                group.stop()
                print(f"  Execution complete.")
                rospy.sleep(1.0)  # Wait for robot to settle
                
            else:
                print("\n  WARNING: All planners failed!")
                print("  Attempting fallback with go() command...")
                
                # Fallback: try using go() directly
                if to_wp["type"] == "named":
                    group.set_named_target(to_wp["name"])
                elif to_wp["type"] == "joints":
                    group.set_joint_value_target(to_wp["joints"])
                else:
                    group.set_pose_target(to_wp["pose"])
                
                success = group.go(wait=True)
                group.stop()
                
                if success:
                    print(f"  Fallback execution successful.")
                else:
                    print(f"  Fallback also failed! Skipping this motion.")
                
                rospy.sleep(1.0)
        
        print(f"\n>>> Run {run} complete!")
    
    # Cleanup
    if USE_OBSTACLES:
        remove_obstacles(scene)
        print("  Obstacles removed.")
    
    print("\n[benchmark] Benchmark complete!")
    roscpp_shutdown()


if __name__ == "__main__":
    main()

