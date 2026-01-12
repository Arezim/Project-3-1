#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo

caminfo = CameraInfo()
caminfo.width = 848
caminfo.height = 480
caminfo.distortion_model = "plumb_bob"
caminfo.D = [0,0,0,0,0]
caminfo.K = [425.0, 0.0, 424.0,
             0.0, 425.0, 239.5,
             0.0, 0.0, 1.0]
caminfo.R = [1,0,0,
             0,1,0,
             0,0,1]
caminfo.P = [425.0, 0.0, 424.0, 0,
             0.0, 425.0, 239.5, 0,
             0.0, 0.0, 1.0, 0]

pub = rospy.Publisher("/camera/depth/camera_info", CameraInfo, queue_size=10)

def cb(msg):
    caminfo.header.stamp = msg.header.stamp
    caminfo.header.frame_id = msg.header.frame_id
    pub.publish(caminfo)

rospy.init_node("caminfo_sync")
rospy.Subscriber("/camera/depth/image_for_sync", Image, cb)
rospy.spin()

