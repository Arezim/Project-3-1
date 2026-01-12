The main script that performs iterative planning for a constrained rail orientation is ins linear_drive_constrained_planning.py. 
It relies on a definition of your robot URDF as a planning group in MoveIt! and referenceing the correct link IDs.
This script can be run individually or through ROS. Through ROS, it opens a service (.srv files are included but need to be compiled).

The new planning scene should be launched in a secondary ROS NAMESPACE to not link the with either the primary MoveIt! planning scene or the real robot.
This is done through the fake_world.launch, which can be included to run automatically within another launch file.

