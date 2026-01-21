#!/usr/bin/env python3

import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from gazebo_ros.gazebo_interface import GazeboRos

class pointCloudVisualizer():
    def __init__(self):
        GazeboRos.__init__(self)
        self.point_cloud_sub = rospy.Subscriber("/custom/point_cloud2/cloud_registered", PointCloud2, self.point_cloud_callback)

    def point_cloud_callback(self, msg):
        cloud = np.array(list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x","y","z"))))

        for i in range(cloud.shape[0]):
            x,y,z = cloud[i,:]

            