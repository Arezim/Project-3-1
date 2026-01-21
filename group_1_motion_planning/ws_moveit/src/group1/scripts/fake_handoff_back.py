#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

class FakeHandoffBack:
    def __init__(self):
        self.pub = rospy.Publisher("/group1/getNewGoal", Bool, queue_size=10)
        self.busy = False
        rospy.Subscriber("/group1/motion/isTargetReached", Bool, self.cb)

        rospy.sleep(0.2)
        self.pub.publish(Bool(False))

    def cb(self, msg: Bool):
        if not msg.data or self.busy:
            return
        self.busy = True

        rospy.loginfo("[FAKE] Got isTargetReached=True. Setting getNewGoal=False, working 10s...")
        self.pub.publish(Bool(False))

        rospy.sleep(10.0)

        rospy.loginfo("[FAKE] Sending /group1/getNewGoal=True (will stay True until G1 resets)")
        self.pub.publish(Bool(True))

        self.busy = False

if __name__ == "__main__":
    rospy.init_node("fake_handoff_back")
    FakeHandoffBack()
    rospy.spin()
