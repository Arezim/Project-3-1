#!/usr/bin/env python3

import rospy

from moveit.task_constructor import core, stages
from moveit_commander.roscpp_initializer import roscpp_initialize
from moveit_commander import PlanningSceneInterface
import moveit_commander

from moveit_msgs.msg import RobotState, OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Vector3, TwistStamped 
from std_msgs.msg import Header

from corosect_object import corosect_object
import pick_and_place
import moveit_object_spawner 
import time
import numpy as np

from pymoveit_mtc.core import Solution, InterfaceState,  PropertyMap, Property

# Symlink warning fix:
# 1: rm ~/corosect_ws/devel/lib/python3/dist-packages/moveit/__init__.py
# 2: ln ~/corosect_ws/devel/.private/moveit_task_constructor_core/lib/python3/dist-packages/moveit/__init__.py ~/corosect_ws/devel/lib/python3/dist-packages/moveit/__init__.py


###########################################################################################################################################################################################
# WARNING: the previous task needs to have been executed before you can design the next one (the CurrentState('current') takes the actual current position and not the one at planning time)
###########################################################################################################################################################################################

roscpp_initialize("mtc")
rospy.init_node("mtc", anonymous=False)
arm_group = "arm"
eef_group = "soft_rg6"

arm_name = "arm"
eef_name = "soft_rg6_ee"

relative_joint = "m_robot_linear_drive_link_ee"

def add_linear_drive_constraints(move_group_commander: moveit_commander.MoveGroupCommander) -> None:
    move_group_commander.clear_path_constraints()

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.stamp = rospy.Time.now()
    orientation_constraint.header.frame_id = 'base_link'
    orientation_constraint.orientation = move_group_commander.get_current_pose("m_robot_linear_drive_link_ee").pose.orientation
    orientation_constraint.link_name = 'm_robot_linear_drive_link_ee'
    orientation_constraint.absolute_x_axis_tolerance = 0.3
    orientation_constraint.absolute_y_axis_tolerance = 0.3
    orientation_constraint.absolute_z_axis_tolerance = 0.3
    orientation_constraint.weight = 1

    m_robot_constraints = Constraints()
    m_robot_constraints.name = 'linear_drive_guiderail_constraint'
    m_robot_constraints.orientation_constraints.append(orientation_constraint)
    move_group_commander.set_path_constraints(m_robot_constraints)

move_group_arm = moveit_commander.MoveGroupCommander(arm_group)
move_group_arm_guide = moveit_commander.MoveGroupCommander("arm_guide_rail")
# add_linear_drive_constraints(move_group)

jointspace = core.JointInterpolationPlanner()
cartesian = core.CartesianPath()

pipeline = core.PipelinePlanner()
pipeline.planner = "RRTConnectkConfigDefault"
# planners_arm_guide_rail = [("arm_guide_rail", pipeline)]
planners_arm = [("arm", pipeline),(eef_group, pipeline)]

task1 = core.Task()
task1.enableIntrospection()

# STEP 3
# TODO add mixing inside inlay
'''
Kamil: 
1) locate crate+inlay
2) start at position at top of crate
3) apply random points at high frequency
4) play spline to all random points
5) hopefully we get some mixing motions without needing a specialized tool 
'''

# STEP 4
object_type = 'cup'
object_name = object_type+" "+str(time.time())

# TODO add the cup mesh to the configs/mesh_path use it to replace the dummy value added currently
cup_object = corosect_object(object_name, object_type)


# TODO make this the actually object's position
cup_object.object_pose.header.frame_id = "world"
cup_object.object_pose.pose.position.x = 0.5
cup_object.object_pose.pose.position.y = 0
cup_object.object_pose.pose.position.z = 0.5

cup_object.target_pose.header.frame_id = "world"
cup_object.target_pose.pose.position.x = 0.5
cup_object.target_pose.pose.position.y = 0
cup_object.target_pose.pose.position.z = 0.5

# spawn in the cup
moveit_object_spawner.spawn_object(cup_object)

# the starting position of the movement
task1.add(stages.CurrentState("current"))

# add connect state
task1.add(stages.Connect("move to pick", planners_arm))

# add the grasp motion
task1.add(pick_and_place.generate_pick(task1,object_name, object_type, "top_down"))

# plan and execute motion
successful = task1.plan()
if not successful:
    pass # TODO implement what to do when planning fails
# STEP 5
#task1.execute(task1.solutions[0])
print("-------------------------")
for _, item in task1.solutions[0].end.properties.items():
    print(item.value(), item.description())
# print(task1.solutions[0].end.properties.property())
print(task1.solutions[0].end.scene.current_state._positions)
method_list = [method for method in dir(Property) if method.startswith('_') is False]
print(method_list)













exit()

'''
Kamil: Scooping motion in two stages:
    1) move to cartesian position relative to crate/inlay location.
    - keep m_robot_linear_drive_dof1_dof2_joint at 0 (pointing down)
    2) move in joint space:
        m_robot_linear_drive_guiderail_sledge_joint -> forward 10-30cm.
         m_robot_linear_drive_dof1_dof2_joint -> -90.   
'''


task2 = core.Task()
task2.enableIntrospection()
task2.add(stages.CurrentState("current"))

# move to the home position
move_to_workspace = stages.MoveTo("move to workspace", jointspace)
move_to_workspace.group = arm_group
move_to_workspace.setGoal("workspace")
task2.add(move_to_workspace)

# plan and execute motion
successful = task2.plan()
if not successful:
    pass # TODO implement what to do when planning fails
task2.execute(task2.solutions[0])


'''
Kamil: the following joint positions are an approximate motion for emptying the sample cup into the plate
'''
task3 = core.Task()
task3.enableIntrospection()
task3.add(stages.CurrentState("current"))


# # Plate location needs to be calibrated
# joint_name = ['m_robot_joint_1', 'm_robot_joint_2', 'm_robot_joint_3', 
#                                     'm_robot_joint_4', 'm_robot_joint_5', 'm_robot_joint_6', 
#                                     'm_robot_joint_7', 'm_robot_linear_drive_guiderail_sledge_joint',
#                                     'm_robot_linear_drive_sledge_dof1_joint', 'm_robot_linear_drive_dof1_dof2_joint']

# joint_target_position = [np.radians(0), np.radians(0),
#                         np.radians(0), np.radians(-90),
#                         np.radians(0), np.radians(90),
#                         np.radians(0), 0.2, np.radians(0),
#                         np.radians(0)]

# res = {joint_name[i]: joint_target_position[i] for i in range(len(joint_name))}

# move_to_plate = stages.MoveTo("move to workspace", jointspace)
# move_to_plate.group = arm_group
# move_to_plate.setGoal(res)

objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.position.x = 0
objectPose.pose.position.y = -0.5
objectPose.pose.position.z = 0.5
# objectPose.pose.orientation.x = -0.707
objectPose.pose.orientation.y = 1
# objectPose.pose.orientation.z = -0.707
# objectPose.pose.orientation.w = 1

move_to_crate_cartesian = stages.MoveTo("test cartesian move", cartesian)
move_to_crate_cartesian.group = arm_group
move_to_crate_cartesian.setGoal(objectPose)

task3.add(move_to_crate_cartesian)
# # Deposit at plate location
# joint_target_position = [np.radians(-45), np.radians(32),
#                         np.radians(-18), np.radians(-33),
#                         np.radians(9), np.radians(114),
#                         np.radians(-4), 0.420, np.radians(-2),
#                         np.radians(0)]

# cup_tip_pose = {joint_name[i]: joint_target_position[i] for i in range(len(joint_name))}

# tip_cup = stages.MoveTo("tip the cup", jointspace)
# tip_cup.group = arm_group
# tip_cup.setGoal(cup_tip_pose)

joint_name = ['m_robot_joint_1', 'm_robot_joint_2', 'm_robot_joint_3', 
                                    'm_robot_joint_4', 'm_robot_joint_5', 'm_robot_joint_6', 
                                    'm_robot_joint_7', 'm_robot_linear_drive_guiderail_sledge_joint',
                                    'm_robot_linear_drive_sledge_dof1_joint', 'm_robot_linear_drive_dof1_dof2_joint']

# joint_target_position = [0, 0,
#                         0, 0,
#                         0, 0,
#                         0, 0.20, 0,
#                         0]

# scoop_position = {joint_name[i]: joint_target_position[i] for i in range(len(joint_name))}

# scooping_motion1 = stages.MoveRelative("scooping motion", jointspace)
# scooping_motion1.group = arm_group
# scooping_motion1.setGoal(scoop_position)


# task3.add(scooping_motion1)

# plan and execute motion
successful = task3.plan()
if not successful:
    pass # TODO implement what to do when planning fails
# STEP 10
# rospy.sleep(20)
task3.execute(task3.solutions[0])



current_joint_pos = move_group.get_current_joint_values()
current_joint_pos[7]=0.42
# use the moveit commander to move
move_group.go(current_joint_pos, wait=True)
move_group.stop()
current_joint_pos[9]=np.radians(-90)
move_group.go(current_joint_pos, wait=True)
move_group.stop()

# TODO SERVICE CALL FOR SHAKING

task4 = core.Task()
task4.enableIntrospection()
task4.add(stages.CurrentState("current"))

move_to_camera = stages.MoveTo("move to camera", jointspace)
move_to_camera.group = arm_group
move_to_camera.setGoal("visual_inspection")
task4.add(move_to_camera)

successful = task4.plan()
if not successful:
    pass # TODO implement what to do when planning fails
task4.execute(task4.solutions[0])

###########################
# let camera do it's work #
###########################

task5 = core.Task()
task5.enableIntrospection()
task5.add(stages.CurrentState("current"))

move_to_workspace2 = stages.MoveTo("move back to workspace", jointspace)
move_to_workspace2.group = arm_group
move_to_workspace2.setGoal("workspace")
task5.add(move_to_workspace2)

successful = task5.plan()
if not successful:
    pass # TODO implement what to do when planning fails
task5.execute(task5.solutions[0])

current_joint_pos = move_group.get_current_joint_values()
current_joint_pos[9]=np.radians(0)
move_group.go(current_joint_pos, wait=True)
move_group.stop()


# STEP 12
del pipeline
del planners
pipeline = core.PipelinePlanner()
pipeline.planner = "RRTConnectkConfigDefault"
planners = [(arm_group, pipeline),(eef_group, pipeline)]

task6 = core.Task()
task6.enableIntrospection()
task6.add(stages.CurrentState("current"))

# Connect the Pick stage with the following Place stage
task6.add(stages.Connect("move to place position", planners))

# Define the pose that the object should have after placing
placePose = cup_object.target_pose

# Generate Cartesian place poses for the object
place_generator = stages.GeneratePlacePose("Generate Place Pose")
place_generator.setMonitoredStage(task6["current"])
place_generator.object = object_name
place_generator.pose = placePose

simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")
simpleUnGrasp.pregrasp = 'open'

# Place container comprises placing, ungrasping, and retracting
place = stages.Place(simpleUnGrasp, "Place")
place.eef = eef_name
place.object = object_name
place.eef_frame = relative_joint

# Twist to retract from the object
retract = TwistStamped()
retract.header.frame_id = "world"
retract.twist.linear.z = 1.0
place.setRetractMotion(retract, 0.03, 0.1)

# Twist to place the object
placeMotion = TwistStamped()
placeMotion.header.frame_id = relative_joint
placeMotion.twist.linear.z = 1.0
place.setPlaceMotion(placeMotion, 0.03, 0.1)

task6.add(place)

# Connect the Pick stage with the following Place stage
# task6.add(stages.Connect("move to place position", planners))

move_to_home_final = stages.MoveTo("move to home final", jointspace)
move_to_home_final.group = arm_group
move_to_home_final.setGoal("default_arm")

task6.add(move_to_home_final)
successful = task6.plan()
if not successful:
    pass # TODO implement what to do when planning fails

# STEP 13
task6.execute(task6.solutions[0])
# rospy.sleep(50)

psi = PlanningSceneInterface(synchronous=True)
psi.remove_world_object(object_name)
del pipeline
del planners