#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
import time
import math

# difference = math.inf
def callback(data):
    # corrected_time = data.header.stamp.to_time()-difference
    data.header.stamp = rospy.Time.now()
    pub.publish(data)



rospy.init_node('listener', anonymous=True)
# start_time = rospy.wait_for_message('/clock',Clock,timeout=None)
# difference = time.time()-start_time.clock.to_time()

rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, callback)
# rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
pub = rospy.Publisher('/custom/point_cloud2/cloud_registered',PointCloud2,queue_size=10)
rospy.spin()