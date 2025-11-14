#!/usr/bin/env python3
import rospy
import math
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import PoseStamped

def make_pose(x, y, z, frame="base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
        
    return pose

def interpolate_pose(start, target, alpha):
    """alpha = 1.0 -> target, alpha = 0.0 -> start"""
    pose = PoseStamped()
    pose.header.frame_id = start.header.frame_id
    pose.pose.orientation = deepcopy(target.pose.orientation)
    pose.pose.position.x = start.pose.position.x * (1 - alpha) + target.pose.position.x * alpha
    pose.pose.position.y = start.pose.position.y * (1 - alpha) + target.pose.position.y * alpha
    pose.pose.position.z = start.pose.position.z * (1 - alpha) + target.pose.position.z * alpha

    return pose

def try_plan(group, pose):
    group.set_start_state_to_current_state()
    group.set_pose_target(pose)
    plan = group.plan()

    if isinstance(plan, tuple):
        success = plan[0]
        real_plan = plan[1]
    else:
        success = True
        real_plan = plan

    if success and real_plan and real_plan.joint_trajectory.points:
        group.execute(real_plan, wait=True)
        group.stop()
        group.clear_pose_targets()
        return True

    group.stop()
    group.clear_pose_targets()

    return False


def main():
    rospy.init_node("move_to_points", anonymous=True) # anonymous to allow multiple instances

    # init moveit
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planning_time(10)  # seconds
    # group.allow_replanning(True)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.25)
    group.set_max_acceleration_scaling_factor(0.25)

    # define target points - MUST match the markers in marker_points.py
    A = make_pose(0.60, 0.10, 0.95)
    B = make_pose(-0.60, -0.50, 1.50) # x=0.80, y=-0.40, z=0.65

    targets = [A, B]
    index = 0

    # set alpha stepping parameters, for interpolation if direct plan fails
    alpha_start = 1.0
    alpha_end = 0.4
    alpha_step = 0.05

    rate = rospy.Rate(0.1) # one cycle every 10 seconds
    rospy.loginfo("move_to_points: STARTING LOOP A <-> B")
    while not rospy.is_shutdown():
        target = targets[index]
        label = "A" if index == 0 else "B"

        # current pose is base for interpolation
        current = group.get_current_pose()
        current.header.frame_id = "base_link"

        # try to plan directly to target, if fails, do interpolation stepping alpha
        # down from alpha_start -> alpha_end by alpha_step. alpha=1.0 is target.
        success = False
        alpha = alpha_start
        # We use a while loop to avoid floating-point range issues with `range()`.
        while alpha >= alpha_end:
            # Build candidate pose by interpolating current -> target using alpha
            # (alpha=1.0 means fully target, alpha=0.0 means stay at start).
            cand = interpolate_pose(current, target, alpha)
            rospy.loginfo(f"move_to_points: trying to plan to [{label}] with [alpha={alpha:.2f}]")
            if try_plan(group, cand):
                rospy.loginfo(f"move_to_points: successfully moved to [{label}] with [alpha={alpha:.2f}]")
                success = True
                break
            else:
                rospy.logwarn(f"move_to_points: failed to plan to [{label}] with [alpha={alpha:.2f}]")

            # decrement alpha, round to avoid tiny float accumulation errors
            alpha = round(alpha - alpha_step, 5)

        if not success:
            rospy.logerr(f"move_to_points: ABORTING - could not plan to [{label}]")

        # switch target
        index = 1 - index
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass