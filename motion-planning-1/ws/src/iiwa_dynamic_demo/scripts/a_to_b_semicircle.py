#!/usr/bin/env python3
import sys, math, rospy, moveit_commander
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

# ------------ TUNE THESE -------------
SAFE_A = [0.0, -1.20, 0.0, -1.50, 0.0, 1.30, 0.0]
SAFE_B = [0.6, -0.9, 0.3, -1.4, 0.2, 1.1, 0.0]

R = 0.12           # semicircle radius (m)
N = 20             # number of waypoints along arc
# -------------------------------------

def latched_marker(ns, frame, pos, rgba, scale=0.06):
    m = Marker()
    m.header.frame_id = frame
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.ns = ns
    m.id = 0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
    return m

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("a_to_b_semicircle")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(
        rospy.get_param("~group", robot.get_group_names()[0])
    )
    scene = moveit_commander.PlanningSceneInterface()

    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    # Use the active planning frame for everything (often "world" or "base_link")
    frame = group.get_planning_frame()
    ee_link = group.get_end_effector_link() or group.get_link_names()[-1]
    rospy.loginfo("Planning frame: %s | EE link: %s", frame, ee_link)

    # Latched marker publisher so RViz always sees them
    marker_pub = rospy.Publisher("/iiwa_markers", Marker, queue_size=2, latch=True)
    rospy.sleep(0.5)

    # Move to A first (from current)
    group.set_start_state_to_current_state()
    group.set_joint_value_target(SAFE_A)
    plan = group.plan()
    plan = plan[1] if isinstance(plan, tuple) else plan
    if hasattr(plan, "joint_trajectory"):
        group.execute(plan, wait=True)

    # Build a semicircle in the XY plane of the planning frame,
    # centered midway between A and B tips (approx via FK of current pose).
    start_pose = group.get_current_pose().pose  # at A now
    # target orientation = current orientation for the whole arc (easier IK)
    qx, qy, qz, qw = start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w

    # Put arc center slightly in front of the current tip
    Cx, Cy, Cz = start_pose.position.x + 0.0, start_pose.position.y + 0.05, start_pose.position.z

    # Start/goal points of the markers (left/right of center)
    start_pt = (Cx - R, Cy, Cz)
    goal_pt  = (Cx + R, Cy, Cz)
    marker_pub.publish(latched_marker("start", frame, start_pt, (0.0,1.0,0.0,1.0)))
    marker_pub.publish(latched_marker("goal",  frame, goal_pt,  (1.0,0.0,0.0,1.0)))

    # Waypoints along a top semicircle from left to right
    waypoints = []
    for i in range(N + 1):
        t = float(i) / N
        ang = math.pi * (1.0 - t)  # pi -> 0
        x = Cx + R * math.cos(ang)
        y = Cy + R * math.sin(ang)
        z = Cz
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        waypoints.append(p)

    # In Noetic, compute_cartesian_path wants positional args (eef_step, jump_threshold, avoid_collisions)
    cart_traj, fraction = group.compute_cartesian_path(waypoints, 0.01, True)
    rospy.loginfo("Cartesian path fraction = %.2f", fraction)

    if fraction > 0.9 and hasattr(cart_traj, "joint_trajectory"):
        group.execute(cart_traj, wait=True)
    else:
        rospy.logwarn("Could not compute full semicircle (fraction=%.2f). Falling back to joint-space A->B.", fraction)
        group.set_start_state_to_current_state()
        group.set_joint_value_target(SAFE_B)
        jplan = group.plan()
        jplan = jplan[1] if isinstance(jplan, tuple) else jplan
        if hasattr(jplan, "joint_trajectory"):
            group.execute(jplan, wait=True)

    rospy.loginfo("Done.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
