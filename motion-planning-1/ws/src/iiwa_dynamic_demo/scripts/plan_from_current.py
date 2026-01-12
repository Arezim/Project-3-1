#!/usr/bin/env python3
import sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped

SAFE_A = [0.0, -1.20, 0.0, -1.50, 0.0, 1.30, 0.0]
SAFE_B = [0.70, -0.90, 0.40, -1.60, 0.20, 1.20, 0.0]

def add_box(scene, name, size, xyz, frame="world"):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.orientation.w = 1.0
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
    scene.add_box(name, ps, size=size); rospy.sleep(0.4)

def remove_box(scene, name):
    scene.remove_world_object(name); rospy.sleep(0.2)

def exec_plan(group):
    plan = group.plan()
    traj = plan[1] if isinstance(plan, tuple) else plan
    return hasattr(traj, "joint_trajectory") and group.execute(traj, wait=True)

def move_from_current_to(group, q_goal):
    group.set_start_state_to_current_state()
    group.set_joint_value_target(q_goal)
    return exec_plan(group)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("iiwa_plan_from_current")
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander(
        rospy.get_param("~group_name", robot.get_group_names()[0])
    )
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    rospy.sleep(0.8)

    # 0) current -> A (so start waypoint == current state)
    move_from_current_to(group, SAFE_A)

    # 1) A -> B
    move_from_current_to(group, SAFE_B)

    # 2) add static obstacle, go back B -> A
    add_box(scene, "blocker", size=(0.25,0.25,0.25), xyz=(0.45, 0.00, 0.30))
    move_from_current_to(group, SAFE_A)

    # 3) dynamic replan: add/remove while moving A -> B -> A
    add_box(scene, "dynamic_box", size=(0.20,0.20,0.20), xyz=(0.35, 0.12, 0.25))
    move_from_current_to(group, SAFE_B)
    remove_box(scene, "dynamic_box")
    move_from_current_to(group, SAFE_A)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
