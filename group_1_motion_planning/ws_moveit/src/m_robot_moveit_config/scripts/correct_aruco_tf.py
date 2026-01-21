#!/usr/bin/env python3
import rospy 
import tf

from fiducial_msgs.msg import FiducialTransformArray

class CorrectArucoTf:

    def __init__(self) -> None:
        rospy.init_node('corosect_aruco_tf')
        self.tf_publisher = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber('/real_world/fiducial_transforms', FiducialTransformArray, self._tf_callback, queue_size=1)

    def _tf_callback(self, msg: FiducialTransformArray):

        for m in msg.transforms:
            pose = m.transform
            self.tf_publisher.sendTransform((pose.translation.x, pose.translation.y, pose.translation.z),
                     (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                     rospy.Time.now(),
                     f'rs_fiducial_{m.fiducial_id}',
                     "rs_aruco_camera")

if __name__ == '__main__':
    CorrectArucoTf()
    rospy.spin()