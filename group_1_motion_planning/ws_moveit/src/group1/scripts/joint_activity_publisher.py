#!/usr/bin/env python3  
import rospy
import message_filters

import numpy as np

from corosect_msgs.msg import JointActivity
from iiwa_msgs.msg import JointVelocity

class JointActivityPublisher:

    def __init__(self) -> None:
        rospy.init_node('joint_activity_combiner')
        
        self.iiwa_joint_activity_publisher = rospy.Publisher('/iiwa/State/joint_activity', JointActivity, queue_size=1)
        self.iiwa_joint_activity_msg = JointActivity()
        self.iiwa_joint_activity_msg.header.seq = -1
        self.iiwa_joint_activity_msg.joints = [0,1,2,3,4,5,6,]
        self.iiwa_joint_activity_msg.moving = [False,]*7


        self.joint_activity_publisher = rospy.Publisher('joint_activity', JointActivity, queue_size=1)
        sync_sub = message_filters.ApproximateTimeSynchronizer([
            message_filters.Subscriber('/iiwa/State/joint_activity', JointActivity),
            message_filters.Subscriber('rg6_controller/joint_activity', JointActivity),
        ], 10, .1)

        sync_sub.registerCallback(self.sync_callback)
        
        self.iiwa_joint_velocity = rospy.Subscriber('/iiwa/state/JointVelocity', JointVelocity, self.iiwa_joint_velocity_callback)
        self.iiwa_velocities = [[0,0,0,0,0,0,0],]*10
        self.iiwa_velocity_index = 0

        self.joint_activity_msg = JointActivity()
        self.joint_activity_msg.header.seq = -1

        rospy.spin()
    
    def sync_callback(self, *messages):
        self.joint_activity_msg.header.seq += 1
        self.joint_activity_msg.header.stamp = rospy.Time.now()
        self.joint_activity_msg.joints = []
        self.joint_activity_msg.moving = []

        activities = {}
        for msg in messages:
            for i in range(len(msg.joints)):
                activities[msg.joints[i]] = msg.moving[i]

        for joint in sorted(activities.keys()):
            self.joint_activity_msg.joints.append(joint)
            self.joint_activity_msg.moving.append(activities[joint])

        self.joint_activity_publisher.publish(self.joint_activity_msg)

    def iiwa_joint_velocity_callback(self, msg: JointVelocity):
        
        self.iiwa_velocities[self.iiwa_velocity_index] = [msg.velocity.a1, msg.velocity.a2, msg.velocity.a3,
                                                           msg.velocity.a4, msg.velocity.a5, msg.velocity.a6,
                                                            msg.velocity.a7 ]

        self.iiwa_joint_activity_msg.header.seq += 1
        self.iiwa_joint_activity_msg.header.stamp = rospy.Time.now()
        self.iiwa_joint_activity_msg.moving = np.mean(np.abs(self.iiwa_velocities),axis=0) > 0.01
        self.iiwa_joint_activity_publisher.publish(self.iiwa_joint_activity_msg)

        self.iiwa_velocity_index = (self.iiwa_velocity_index + 1)
        if self.iiwa_velocity_index >= 5:
            self.iiwa_velocity_index = 0

if __name__ == '__main__':
    JointActivityPublisher()


