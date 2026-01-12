#!/usr/bin/env python3
import sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

JOINTS     = [f"m_robot_joint_{i}" for i in range(1,8)]
UPRIGHT    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HORIZONTAL = [0.0, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0]

# how much to shift the blocker relative to the geometric midpoint (meters)
# keep your “works for me” default, but these are no longer required at runtime
DEFAULT_DX, DEFAULT_DY, DEFAULT_DZ = (0.0, -0.40, 0.10)
DEFAULT_SIZE = (0.10, 0.10, 0.10)

def marker(ns, frame, pos, rgba, s=0.05):
    from visualization_msgs.msg import Marker as M
    m = M(); m.header.frame_id=frame; m.type=M.SPHERE; m.action=M.ADD
    m.ns=ns; m.id=0; m.scale.x=m.scale.y=m.scale.z=s
    m.color.r,m.color.g,m.color.b,m.color.a = rgba
    m.pose.orientation.w=1.0
    m.pose.position.x,m.pose.position.y,m.pose.position.z = pos
    return m

def go_joints_plan_only(group, names, values):
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(names, values)))
    plan = group.plan(); plan = plan[1] if isinstance(plan, tuple) else plan
    group.stop(); group.clear_pose_targets()
    return plan

def go_joints(group, names, values):
    plan = go_joints_plan_only(group, names, values)
    ok = hasattr(plan, "joint_trajectory") and plan.joint_trajectory.points
    if ok: group.execute(plan, wait=True)
    return ok

def wait_obj(scene, name, exist=True, t=3.0):
    start=rospy.Time.now()
    while (rospy.Time.now()-start).to_sec() < t:
        if (name in scene.get_known_object_names()) == exist: return True
        rospy.sleep(0.05)
    return False

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("a_to_b_with_obstacle_fixed")

    # Optional offsets/size still supported
    dx=rospy.get_param("~blocker_dx", DEFAULT_DX)
    dy=rospy.get_param("~blocker_dy", DEFAULT_DY)
    dz=rospy.get_param("~blocker_dz", DEFAULT_DZ)
    sx,sy,sz = tuple(rospy.get_param("~blocker_size", list(DEFAULT_SIZE)))

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(robot.get_group_names()[0])
    scene = moveit_commander.PlanningSceneInterface()

    group.set_planning_pipeline_id("ompl")
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    frame = group.get_planning_frame()
    ee    = group.get_end_effector_link()
    pub   = rospy.Publisher("/iiwa_markers", Marker, queue_size=4, latch=True)
    rospy.sleep(0.2)

    # 1) Go to Upright (start A)
    rospy.loginfo("Going to UPRIGHT…")
    go_joints(group, JOINTS, UPRIGHT)
    A = group.get_current_pose().pose.position
    pub.publish(marker("start", frame, (A.x,A.y,A.z), (0,1,0,1)))

    # 2) PLAN (don’t execute yet) Upright -> Horizontal to infer where to block
    rospy.loginfo("Planning to HORIZONTAL to position blocker…")
    plan = go_joints_plan_only(group, JOINTS, HORIZONTAL)

    # Midpoint between current EE pose and the expected EE pose at goal
    # (robust proxy for the path midpoint without requiring FK on trajectory)
    # Move temporarily to HORIZONTAL (execute), read pose B, then come back to A
    # so the blocker placement really sits on the path you’ll traverse.
    go_joints(group, JOINTS, HORIZONTAL)
    B = group.get_current_pose().pose.position
    pub.publish(marker("goal", frame, (B.x,B.y,B.z), (1,0,0,1)))
    # geometric midpoint + offset
    mid = ((A.x+B.x)/2.0 + dx, (A.y+B.y)/2.0 + dy, max((A.z+B.z)/2.0 + dz, 0.03))

    # go back to A (so execution starts as requested)
    go_joints(group, JOINTS, UPRIGHT)

    # 3) Spawn blocker at mid
    scene.remove_world_object("blocker"); wait_obj(scene, "blocker", False, 1.0)
    ps = PoseStamped(); ps.header.frame_id=frame; ps.pose.orientation.w=1.0
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = mid
    scene.add_box("blocker", ps, size=(sx,sy,sz)); wait_obj(scene, "blocker", True, 2.0)

    # 4) Execute A -> B -> A around the blocker (same path both ways)
    rospy.loginfo("Executing A → B (around blocker)…")
    go_joints(group, JOINTS, HORIZONTAL)
    rospy.loginfo("Returning B → A (around blocker)…")
    go_joints(group, JOINTS, UPRIGHT)

    rospy.loginfo("Static obstacle A → B → A finished.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
