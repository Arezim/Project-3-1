#!/usr/bin/env python3  

import rospy
import tf
from iiwa_msgs.msg import JointTorque

class ForceFeedback:

    def __init__(self) -> None:
        rospy.init_node(name='test_force_feedback_node')

        self.tf_listener = tf.listener.TransformListener()
        torque_sub = rospy.Subscriber('/iiwa/state/ExternalJointTorque', JointTorque, self.torque_callback)

        rospy.spin()

    def torque_callback(self, torques: JointTorque):
        time = rospy.Time(0)
        for i in range(7):
            t, _ = self.tf_listener.lookupTransform(f'm_robot_link_{i}', 'm_robot_link_0', time)
            print(t)


def main():
    ForceFeedback()


if __name__ == '__main__':
    main()
