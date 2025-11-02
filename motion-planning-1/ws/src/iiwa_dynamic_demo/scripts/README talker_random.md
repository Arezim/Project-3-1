What is this?: 
Demonstration of communication of positions (By another ROS node) in the environment to plan the robot to and move it to. New position are communicated to the ROS node that instructs what the moveit_group to do (move_group_python_interface_tutorial), by another ROS node (talker_random.py), via the ROS topic, "chatter"

How to?:
Follow the updated readme in the Discord server (make sure you have Rviz running, so the command under section 2 in that readme)

Open up another terminal like is written in the discord readme.
Open up a terminal, run this:
rosrun iiwa_dynamic_demo move_group_python_interface_tutorial.py

Open up another terminal, run this:
rosrun iiwa_dynamic_demo talker_random.py
