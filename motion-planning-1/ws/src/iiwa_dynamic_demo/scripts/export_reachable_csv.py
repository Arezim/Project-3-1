#!/usr/bin/env python3
import csv, sys, rospy, moveit_commander
from moveit_msgs.srv import GetStateValidity, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

OUT = "/root/ws/src/iiwa_dynamic_demo/reachable_samples.csv"
SAMPLES = 1000  # increase for denser cloud

def mk_state(names, q):
    js = JointState(name=names, position=q)
    rs = RobotState(joint_state=js)
    return rs

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("export_reachable_csv")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(robot.get_group_names()[0])
    names = group.get_active_joints()
    ee_link = group.get_end_effector_link() or group.get_link_names()[-1]
    planning_frame = group.get_planning_frame()

    rospy.wait_for_service("/check_state_validity")
    rospy.wait_for_service("/compute_fk")
    check = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)

    rows = []
    for _ in range(SAMPLES):
        q = group.get_random_joint_values()  # respects URDF limits
        rs = mk_state(names, q)

        # self-collision & bounds
        valid = check(robot_state=rs, group_name=group.get_name()).valid
        if not valid:
            continue

        # forward kinematics to EE pose
        fk_res = fk(
            header=Header(frame_id=planning_frame, stamp=rospy.Time.now()),
            fk_link_names=[ee_link],
            robot_state=rs
        )
        if not fk_res.pose_stamped:
            continue
        p = fk_res.pose_stamped[0].pose.position
        rows.append([p.x, p.y, p.z] + q)

    with open(OUT, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"] + names)
        writer.writerows(rows)

    rospy.loginfo("Wrote %d reachable samples to %s", len(rows), OUT)
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
