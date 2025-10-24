#!/usr/bin/env python3
import math
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    MoveGroupCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
)

def make_pose(x, y, z, q_xyzw, frame_id):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = q_xyzw[0]
    ps.pose.orientation.y = q_xyzw[1]
    ps.pose.orientation.z = q_xyzw[2]
    ps.pose.orientation.w = q_xyzw[3]
    return ps

def main():
    roscpp_initialize([])
    rospy.init_node("dynamic_replan_demo", anonymous=True)

    scene = PlanningSceneInterface(synchronous=True)
    group = MoveGroupCommander("manipulator")
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    # Use the exact planning frame MoveIt is using (world/base_link/whatever)
    planning_frame = group.get_planning_frame()
    rospy.loginfo("Planning frame: %s", planning_frame)

    rospy.sleep(1.0)  # let the scene/connectors come up

    # ---- Clear + environment -------------------------------------------------
    try:
        scene.remove_world_object()
    except Exception:
        pass
    rospy.sleep(0.5)

    # Ground plane (normal=(0,0,1), offset=0)
    ground_pose = make_pose(0, 0, 0, [0, 0, 0, 1], planning_frame)
    scene.add_plane("ground", ground_pose, (0, 0, 1), 0.0)

    # Add a big box so it's clearly visible
    box_name = "moving_box"
    box_size = (0.35, 0.35, 0.35)
    box_pose = make_pose(0.50, 0.0, 0.60, [0, 0, 0, 1], planning_frame)
    scene.add_box(box_name, box_pose, box_size)

    rospy.sleep(0.5)
    rospy.loginfo("Known objects after spawn: %s", scene.get_known_object_names())

    # ---- Goal pose (reachable for iiwa14) -----------------------------------
    goal_pose = make_pose(
        0.65, 0.0, 0.75,
        tft.quaternion_from_euler(0.0, math.pi, 0.0),
        planning_frame,
    )

    # ---- Reactive planning loop ---------------------------------------------
    rate = rospy.Rate(2.0)
    angle = 0.0

    while not rospy.is_shutdown():
        # Move/rotate the obstacle: re-add with same name (safe across MoveIt versions)
        angle += 0.15
        qz = tft.quaternion_from_euler(0.0, 0.0, angle)
        x = 0.50
        y = 0.25 * math.sin(angle)
        z = 0.60
        scene.add_box(box_name, make_pose(x, y, z, qz, planning_frame), box_size)

        # Plan to the same goal from current state
        group.set_start_state_to_current_state()
        group.set_pose_target(goal_pose)

        plan_tuple = group.plan()
        if isinstance(plan_tuple, tuple):
            ok, traj = plan_tuple[0], plan_tuple[1]
        else:
            ok, traj = bool(plan_tuple), plan_tuple

        if ok and traj:
            group.stop()
            group.execute(traj, wait=False)
            rospy.loginfo_throttle(2.0, "Executing new plan.")
        else:
            rospy.logwarn_throttle(2.0, "No plan found this tick.")

        rate.sleep()

    group.stop()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
