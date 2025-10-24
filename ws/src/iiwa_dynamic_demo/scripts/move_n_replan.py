#!/usr/bin/env python3
import math, rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    MoveGroupCommander, PlanningSceneInterface, RobotCommander,
    roscpp_initialize, roscpp_shutdown,
)
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry

def make_pose(x,y,z,q,frame):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x,y,z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
    return ps

def allow_all_self_collisions(robot):
    links = robot.get_link_names()
    acm = AllowedCollisionMatrix()
    acm.entry_names = list(links)
    for _ in links:
        e = AllowedCollisionEntry()
        e.enabled = [True]*len(links)
        acm.entry_values.append(e)
    scene_msg = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
    pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=1, latch=True)
    start = rospy.Time.now().to_sec()
    while pub.get_num_connections()==0 and (rospy.Time.now().to_sec()-start)<2.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    pub.publish(scene_msg)
    rospy.logwarn("⚠️  Self-collisions disabled for demo (world collisions still checked).")

def main():
    roscpp_initialize([])
    rospy.init_node("move_n_replan", anonymous=True)

    robot = RobotCommander()
    group  = MoveGroupCommander("manipulator")
    scene  = PlanningSceneInterface(synchronous=True)

    # Make planning more forgiving
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(3.0)
    group.set_num_planning_attempts(1)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    planning_frame = group.get_planning_frame()
    rospy.loginfo("Planning frame: %s", planning_frame)
    rospy.sleep(1.0)

    allow_all_self_collisions(robot)

    # World: just the moving cube
    try: scene.remove_world_object()
    except Exception: pass
    rospy.sleep(0.4)

    box_name = "moving_box"
    box_size = (0.30, 0.30, 0.30)
    scene.add_box(box_name, make_pose(0.70, 0.00, 0.65, [0,0,0,1], planning_frame), box_size)
    rospy.sleep(0.4)

    # === Joint-space target (reachable) ===
    # Uses the group’s active joints in order.
    joints = group.get_active_joints()
    rospy.loginfo("Active joints: %s", joints)
    # Pick a clear goal different from typical zero pose (tweak if needed)
    goal_vals = [0.6, -0.8, 0.2, 1.3, -0.1, -0.7, 0.0]
    goal_vals = goal_vals[:len(joints)]
    joint_goal = {j:v for j,v in zip(joints, goal_vals)}

    # Reactive loop: spin/slide the cube and replan to the same joint goal
    rate = rospy.Rate(2.0)
    angle = 0.0
    while not rospy.is_shutdown():
        angle += 0.25
        qz = tft.quaternion_from_euler(0, 0, angle)
        y  = 0.25*math.sin(angle)
        scene.add_box(box_name, make_pose(0.70, y, 0.65, qz, planning_frame), box_size)

        group.set_start_state_to_current_state()
        group.set_joint_value_target(joint_goal)

        plan_tuple = group.plan()
        ok, traj = (plan_tuple[0], plan_tuple[1]) if isinstance(plan_tuple, tuple) else (bool(plan_tuple), plan_tuple)
        if ok and traj:
            group.stop()
            group.execute(traj, wait=False)  # keep moving with freshest plan
        # else: it’ll try again next tick

        rate.sleep()

    group.stop()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
