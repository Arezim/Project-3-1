#!/usr/bin/env python3
"""
Motion Planning & Dynamic Replanning Demo for KUKA iiwa14
---------------------------------------------------------
✅ Works in MoveIt "demo.launch" (fake execution)
✅ Uses current robot state as plan start
✅ Demonstrates:
   1. A → B
   2. Add static obstacle and replan
   3. Add dynamic obstacle → replan → remove
"""

import sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped

# Joint goals (example safe poses for iiwa14)
SAFE_A = [0.0, -1.20, 0.0, -1.50, 0.0, 1.30, 0.0]
SAFE_B = [0.70, -0.90, 0.40, -1.60, 0.20, 1.20, 0.0]

def add_box(scene, name, size, xyz, frame="world"):
    """Add a box to the planning scene."""
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.orientation.w = 1.0
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
    scene.add_box(name, ps, size=size)
    rospy.sleep(0.4)

def remove_box(scene, name):
    """Remove a box from the planning scene."""
    scene.remove_world_object(name)
    rospy.sleep(0.2)

def exec_plan(group):
    """Execute the latest planned trajectory."""
    plan = group.plan()
    traj = plan[1] if isinstance(plan, tuple) else plan
    if not hasattr(traj, "joint_trajectory"):
        rospy.logwarn("No valid trajectory produced.")
        return False
    success = group.execute(traj, wait=True)
    rospy.sleep(0.5)
    return success

def move_to(group, q_target):
    """Plan and move from current state to target joint values."""
    group.set_start_state_to_current_state()
    group.set_joint_value_target(q_target)
    return exec_plan(group)

def visualize_pose_marker(pub, q, color, ns):
    """Publish a marker at the tip pose for RViz visualization."""
    from visualization_msgs.msg import Marker
    pose = group.get_current_pose().pose
    marker = Marker()
    marker.header.frame_id = group.get_planning_frame()
    marker.ns = ns
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = marker.scale.y = marker.scale.z = 0.05
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 0.8
    pub.publish(marker)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("iiwa_plan_and_replan_demo")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = rospy.get_param("~group_name", None) or robot.get_group_names()[0]
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Configure planner
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)
    rospy.sleep(0.8)

    rospy.loginfo("Planning group: %s", group_name)
    rospy.loginfo("Planning frame: %s", group.get_planning_frame())
    rospy.loginfo("End-effector link: %s", group.get_end_effector_link())

    marker_pub = rospy.Publisher("visualization_marker", 
                                 __import__("visualization_msgs").msg.Marker, 
                                 queue_size=10)
    rospy.sleep(0.5)

    # ---- 1) Move A→B ----
    rospy.loginfo("=== A → B ===")
    move_to(group, SAFE_A)
    visualize_pose_marker(marker_pub, SAFE_A, (1, 0, 0), "A_pose")
    move_to(group, SAFE_B)
    visualize_pose_marker(marker_pub, SAFE_B, (0, 1, 0), "B_pose")

    # ---- 2) Add static obstacle & plan around it ----
    rospy.loginfo("=== Adding static obstacle ===")
    add_box(scene, "blocker", size=(0.25, 0.25, 0.25), xyz=(0.45, 0.00, 0.30))
    move_to(group, SAFE_A)

    # ---- 3) Dynamic replan demo ----
    rospy.loginfo("=== Adding dynamic obstacle ===")
    add_box(scene, "dynamic_box", size=(0.20, 0.20, 0.20), xyz=(0.35, 0.12, 0.25))
    move_to(group, SAFE_B)
    rospy.loginfo("=== Removing dynamic obstacle ===")
    remove_box(scene, "dynamic_box")
    move_to(group, SAFE_A)

    rospy.loginfo("Demo finished successfully.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
