#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def make_marker(marker_id, x, y, z, name):
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp = rospy.Time.now()
    m.ns = "target_points"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD

    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.w = 1.0

    m.scale.x = m.scale.y = m.scale.z = 0.14

    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 0.3 # transparency
    m.lifetime = rospy.Duration(0)
    m.text = name
    return m

def main():
    rospy.init_node("marker_two_points")
    pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10, latch=True)
    rate = rospy.Rate(2)

    # define target points - MUST match the points in move_to_points.py
    A = dict(x=0.60, y= 0.10, z=0.95)
    B = dict(x=-0.60, y=-0.50, z=1.50) # x=0.80, y=-0.40, z=0.65

    while not rospy.is_shutdown():
        pub.publish(make_marker(0, A["x"], A["y"], A["z"], "A"))
        pub.publish(make_marker(1, B["x"], B["y"], B["z"], "B"))
        rate.sleep()

if __name__ == "__main__":
    main()
