#!/usr/bin/env python3
import math, rospy
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject

BOX_ID = "moving_box"        # if you added it in RViz as "Box_1", change this to "Box_1"
FRAME  = "base_link"         # your planning frame from the logs

def publish_add(pub, x,y,z, size=(0.30,0.30,0.30)):
    co = CollisionObject()
    co.id = BOX_ID
    co.header.frame_id = FRAME
    co.operation = CollisionObject.ADD
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
    p = Pose()
    p.position.x, p.position.y, p.position.z = x,y,z
    p.orientation.w = 1.0
    co.primitives.append(box)
    co.primitive_poses.append(p)
    pub.publish(co)

def publish_move(pub, x,y,z, qw,qx,qy,qz, size=(0.30,0.30,0.30)):
    co = CollisionObject()
    co.id = BOX_ID
    co.header.frame_id = FRAME
    co.operation = CollisionObject.MOVE
    p = Pose()
    p.position.x, p.position.y, p.position.z = x,y,z
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw,qx,qy,qz
    co.pose = p  # pose is used for MOVE ops
    pub.publish(co)

def main():
    rospy.init_node("spin_box")
    pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=1)
    rospy.sleep(1.0)

    # Ensure the box exists (ADD once)
    publish_add(pub, 0.70, 0.00, 0.65)
    rospy.loginfo("Added/ensured box '%s'. Spinning…", BOX_ID)

    rate  = rospy.Rate(5)  # fast for smooth visuals, but light
    theta = 0.0
    while not rospy.is_shutdown():
        theta += 0.12
        # rotate around Z and sway in Y
        qw = math.cos(theta/2.0); qx=0.0; qy=0.0; qz = math.sin(theta/2.0)
        y  = 0.25*math.sin(theta)
        publish_move(pub, 0.70, y, 0.65, qw,qx,qy,qz)
        rate.sleep()

if __name__ == "__main__":
    main()
