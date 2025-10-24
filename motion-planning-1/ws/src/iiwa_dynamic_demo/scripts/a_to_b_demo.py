#!/usr/bin/env python3
import sys, rospy, moveit_commander
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

JOINTS     = [f"m_robot_joint_{i}" for i in range(1,8)]
UPRIGHT    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HORIZONTAL = [0.0, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0]

def marker(ns, frame, pos, rgba, s=0.05):
    m = Marker()
    m.header.frame_id = frame
    m.type, m.action, m.ns, m.id = Marker.SPHERE, Marker.ADD, ns, 0
    m.scale.x = m.scale.y = m.scale.z = s
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
    return m

def go_joints(group, names, values):
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(names, values)))
    plan = group.plan(); plan = plan[1] if isinstance(plan, tuple) else plan
    ok = hasattr(plan, "joint_trajectory") and plan.joint_trajectory.points
    if ok: group.execute(plan, wait=True)
    group.stop(); group.clear_pose_targets()
    return ok

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("a_to_b_demo_fixed")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(robot.get_group_names()[0])

    group.set_planning_pipeline_id("ompl")
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    frame = group.get_planning_frame()
    pub   = rospy.Publisher("/iiwa_markers", Marker, queue_size=2, latch=True)
    rospy.sleep(0.2)

    # Upright -> Horizontal -> Upright
    rospy.loginfo("Going to UPRIGHT…")
    if not go_joints(group, JOINTS, UPRIGHT):
        rospy.logwarn("Failed to reach UPRIGHT (planning).")

    A = group.get_current_pose().pose.position
    pub.publish(marker("start", frame, (A.x, A.y, A.z), (0,1,0,1)))

    rospy.loginfo("Going to HORIZONTAL…")
    if not go_joints(group, JOINTS, HORIZONTAL):
        rospy.logwarn("Failed to reach HORIZONTAL (planning).")

    B = group.get_current_pose().pose.position
    pub.publish(marker("goal", frame, (B.x, B.y, B.z), (1,0,0,1)))

    rospy.loginfo("Returning to UPRIGHT…")
    go_joints(group, JOINTS, UPRIGHT)

    rospy.loginfo("A → B → A finished.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
