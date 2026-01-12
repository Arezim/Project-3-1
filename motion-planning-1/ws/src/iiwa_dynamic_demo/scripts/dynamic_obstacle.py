#!/usr/bin/env python3
import csv, sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker

CSV_PATHS = [
    "/mnt/data/reachable_samples.csv",
    "/root/ws/src/iiwa_dynamic_demo/reachable_samples.csv",
]
SAFE_HOME = [0.0, -1.1, 0.0, -1.6, 0.0, 1.3, 0.0]

def load_points():
    for path in CSV_PATHS:
        try:
            with open(path) as f:
                r = csv.DictReader(f)
                pts = [(float(row["x"]), float(row["y"]), float(row["z"]))
                       for row in r if float(row["z"]) > 0.10]
                if pts:
                    rospy.loginfo("Loaded %d usable points from %s", len(pts), path)
                    return pts
        except Exception:
            pass
    raise RuntimeError("reachable_samples.csv not found")

def farthest_pair(pts):
    bi, bj, bd2 = 0, 0, -1.0
    for i, (xi, yi, zi) in enumerate(pts):
        for j in range(i+1, len(pts)):
            xj, yj, zj = pts[j]
            d2 = (xi-xj)**2+(yi-yj)**2+(zi-zj)**2
            if d2 > bd2: bi, bj, bd2 = i, j, d2
    return pts[bi], pts[bj]

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("a_to_b_dynamic_obstacle_csv")

    pts = load_points()
    A, B = farthest_pair(pts)
    mid = ((A[0]+B[0])/2.0, (A[1]+B[1])/2.0 + 0.10, (A[2]+B[2])/2.0 + 0.10)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface()
    frame = group.get_planning_frame()

    # safe start
    group.set_joint_value_target(SAFE_HOME)
    group.go(wait=True); group.stop()

    # clear old
    scene.remove_world_object("dynamic_box"); rospy.sleep(0.3)

    # A -> mid
    q = group.get_current_pose().pose.orientation
    def pose_at(p):
        po = Pose(); po.position.x, po.position.y, po.position.z = p; po.orientation = q; return po

    group.set_pose_target(pose_at(A)); group.go(wait=True)
    group.set_pose_target(pose_at(mid)); group.go(wait=True)
    group.stop(); group.clear_pose_targets()

    # Spawn obstacle after reaching mid
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.orientation.w = 1.0
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = mid
    scene.add_box("dynamic_box", ps, size=(0.15,0.15,0.15))
    rospy.sleep(0.4)

    # replan to B
    group.set_pose_target(pose_at(B))
    group.go(wait=True)
    group.stop(); group.clear_pose_targets()

    # remove obstacle and return
    scene.remove_world_object("dynamic_box"); rospy.sleep(0.3)
    group.set_pose_target(pose_at(A)); group.go(wait=True)
    rospy.loginfo("Dynamic obstacle demo complete.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
