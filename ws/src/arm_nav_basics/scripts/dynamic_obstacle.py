#!/usr/bin/env python3
import math
import re
import rospy
from moveit_commander import PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def create_marker(name, frame, size):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = "obstacles"
    marker.id = 42
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.7
    marker.lifetime = rospy.Duration(0)
    return marker


def update_obstacle(event):
    t = (rospy.Time.now() - start_time).to_sec()

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()

    # ----- motion modes -----
    if motion_type == "circle":
        # circular motion in the XY-plane around (center_x, center_y)
        theta = angular_speed * t + phase
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)

        # optional vertical bobbing
        if z_wave_amp > 0.0:
            z = base_z + z_wave_amp * math.sin(2.0 * math.pi * z_wave_freq * t)
        else:
            z = base_z

    else:
        # default: linear sinusoid along one axis (old behavior)
        offset = amplitude * math.sin(2.0 * math.pi * frequency * t)
        x = base_x + offset if axis == "x" else base_x
        y = base_y + offset if axis == "y" else base_y
        z = base_z + offset if axis == "z" else base_z

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0

    # update planning scene (re-adding same name updates pose)
    try:
        psi.remove_world_object(name)
    except Exception:
        pass
    rospy.sleep(0.02)
    psi.add_box(name, pose, size=size)

    # update and publish marker
    marker.header.stamp = rospy.Time.now()
    marker.pose = pose.pose
    marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("dynamic_obstacle_node")

    # parameters (can be set via ROS params)
    name = rospy.get_param("~name", "dynamic_obstacle")

    raw_size = rospy.get_param("~size", [0.60, 0.60, 0.7])

    if isinstance(raw_size, str):
        s = raw_size.strip()
        s = s.strip('[]()')
        parts = re.split(r"[,;\s]+", s)
        parts = [p for p in parts if p != ""]
        raw_size = parts

    # If raw_size is a list of strings or numbers, coerce to floats safely.
    try:
        size = [float(raw_size[0]), float(raw_size[1]), float(raw_size[2])]
    except (IndexError, ValueError, TypeError) as e:
        rospy.logwarn(
            "DYNAMIC_OBSTACLE: could not parse '~size' param (%r): %s. Using default [0.60, 0.60, 0.7]",
            raw_size, e
        )
        size = [0.60, 0.60, 0.7]

    frame = rospy.get_param("~frame", "base_link")

    # base pose (defaults for circle center)
    base_x = float(rospy.get_param("~x", 0.30))
    base_y = float(rospy.get_param("~y", -0.60))
    base_z = float(rospy.get_param("~z", 1.25))

    # ---- linear motion params (old behavior) ----
    amplitude = float(rospy.get_param("~amplitude", 0.10))   # meters
    frequency = float(rospy.get_param("~frequency", 0.2))    # Hz
    axis = rospy.get_param("~axis", "y")                     # "x", "y", or "z" / moves on this axis

    # ---- circular motion params ----
    motion_type = rospy.get_param("~motion_type", "linear")  # "linear" or "circle"

    # circle in XY-plane around (center_x, center_y)
    center_x = float(rospy.get_param("~center_x", base_x))
    center_y = float(rospy.get_param("~center_y", base_y))
    radius = float(rospy.get_param("~radius", 0.35))    # meters
    angular_speed = float(rospy.get_param("~angular_speed", 0.25))  # rad/s
    phase = float(rospy.get_param("~phase", 0.0))   # rad

    # optional vertical bobbing while circling
    z_wave_amp = float(rospy.get_param("~z_wave_amp", 0.0))  # 0.0 = none
    z_wave_freq = float(rospy.get_param("~z_wave_freq", 0.2))

    # how fast to update the obstacle position
    update_rate = float(rospy.get_param("~update_rate", 10.0))  # Hz

    roscpp_initialize([])
    psi = PlanningSceneInterface(ns="")
    rospy.sleep(1.0)

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1, latch=False)
    marker = create_marker(name, frame, size)

    start_time = rospy.Time.now()

    # add initial obstacle and start timer to move it
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = frame
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.position.x = base_x
    initial_pose.pose.position.y = base_y
    initial_pose.pose.position.z = base_z
    initial_pose.pose.orientation.w = 1.0

    try:
        psi.remove_world_object(name)   # try to remove any prior instance
    except Exception:
        pass
    rospy.sleep(0.05) # small pause to allow removal
    psi.add_box(name, initial_pose, size=size)
    marker.pose = initial_pose.pose
    marker_pub.publish(marker)

    # run periodic updates
    rospy.Timer(rospy.Duration(1.0 / float(update_rate)), update_obstacle)

    if motion_type == "circle":
        rospy.loginfo(
            "DYNAMIC_OBSTACLE: '%s' moving in a CIRCLE around (%.2f, %.2f) with radius=%.2f, angular_speed=%.2f rad/s",
            name, center_x, center_y, radius, angular_speed
        )
    else:
        rospy.loginfo(
            "DYNAMIC_OBSTACLE: '%s' moving linearly along %s-axis (amplitude=%.2f, freq=%.2f Hz)",
            name, axis, amplitude, frequency
        )

    rospy.spin()
