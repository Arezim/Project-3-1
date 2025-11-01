Follow the updated readme in the Discord server (make sure you have Rviz running, so the command under section 2 in that readme)

Open up another terminal like is written in the discord readme.
Open up a terminal, run this:
rosrun iiwa_dynamic_demo move_group_python_interface_tutorial.py

Open up another terminal, run this:
rosrun iiwa_dynamic_demo talker.py 

Now type desired goal position and press enter, in the format: float float float float.
The first value is the orientation, then x, y and z
Example: 1.0 0.0137 -0.058 2.005

