#!/usr/bin/env python3
import rospy
from moveit_commander import PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker

def add_static_obstacle(name="static_obstacle",
                        size=(0.20, 0.20, 0.20),
                        pose_dict=None,
                        frame="base_link"):

    if pose_dict is None:
        pose_dict = dict(x=0.0, y=0.20, z=1.225)

    # Initialize moveit internals if this process hasn't already
    roscpp_initialize([])
    psi = PlanningSceneInterface(ns="")
    # wait a moment for the planning scene client/server to be ready
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = pose_dict["x"]
    pose.pose.position.y = pose_dict["y"]
    pose.pose.position.z = pose_dict["z"]
    pose.pose.orientation.w = 1.0

    # Add the box to the planning scene (remove any prior one first)
    psi.remove_world_object(name)
    rospy.sleep(0.5)  # small pause to allow removal
    psi.add_box(name, pose, size=tuple(size))

    # Also publish a red Marker for RViz visualization
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1, latch=True)
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacles"
    marker.id = 42
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose = pose.pose
    marker.pose.orientation = pose.pose.orientation

    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.7  # semi-transparent

    marker.lifetime = rospy.Duration(0)  # persistent
    # publish once; latched publisher keeps it for new subscribers
    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("obstacle_node")
    # tweak the parameters as needed
    size = rospy.get_param("~size", [0.60, 0.60, 0.60])
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", -0.30)
    z = rospy.get_param("~z", 1.5)
    add_static_obstacle(size=tuple(size), pose_dict=dict(x=x, y=y, z=z))
    rospy.loginfo("OBSTACLE_UTILS: Static obstacle added to the planning scene.")
    rospy.spin()