import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal


class DataCollector:

    def __init__(self) -> None:
        rospy.init_node('trajectory_subscriber')
        self.latest_trajectory = ""
        rospy.Subscriber('/arm_controller/follow_joint_trajectory/goal',
                         FollowJointTrajectoryActionGoal,
                         self.arm_controller_goal_callback)

    def arm_controller_goal_callback(self, msg):
        buffer = ""
        points = msg.goal.trajectory.points
        for i in range(len(points)):
            point = points[i]
            buffer += ','.join([f"{joint_position:.8f}" for joint_position in point.positions])
            if i != len(points) - 1:
                buffer += "@"
        self.latest_trajectory = buffer