#!/usr/bin/env python3

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist, Vector3Stamped, Vector3
from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init
from moveit_commander import PlanningSceneInterface
import yaml
import random
import time

# initialize ros node
roscpp_init("mtc")

# robot arm planning group name
arm_group = "/arm"
eef_group = "/gripper"

arm_name = "arm"
eef_name = "gripper"

angle_delta = 0.1

jointspace = core.JointInterpolationPlanner()

random.seed(177013)

# read in predefined positions
with open('/home/corosect/ws_moveit/src/iiwa_moveit_config/scripts/configs/initial_position.yaml') as f:
    
    positions = yaml.load(f, Loader=yaml.FullLoader)

with open('/home/corosect/ws_moveit/src/iiwa_moveit_config/scripts/configs/grasp_angle.yaml') as f:
    
    angles = yaml.load(f, Loader=yaml.FullLoader)


with open('/home/corosect/ws_moveit/src/iiwa_moveit_config/scripts/configs/pickup_configurations.yaml') as f:
    
    data = yaml.load(f, Loader=yaml.FullLoader)

def generate_pick(task, object_name, item_name, grasp_name):
    # generate grasps for picking up
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose " + grasp_name)
    grasp_generator.angle_delta = angle_delta
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "close"
    grasp_generator.setMonitoredStage(task["current"])

    # SimpleGrasp container encapsulates IK calculation of arm pose as well as finger closing
    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp " + grasp_name)
    # Set frame for IK calculation in the center between the fingers
    ik_frame = PoseStamped()
    ik_frame.header.frame_id = "rg6_link_0"
    ik_frame.pose.position.z = data[item_name][grasp_name]['zp']
    ik_frame.pose.orientation.x = data[item_name][grasp_name]['xo']
    ik_frame.pose.orientation.y = data[item_name][grasp_name]['yo']
    ik_frame.pose.orientation.z = data[item_name][grasp_name]['zo']
    ik_frame.pose.orientation.w = data[item_name][grasp_name]['wo']
    simpleGrasp.setIKFrame(ik_frame)
    
    pick = stages.Pick(simpleGrasp, "Pick " + grasp_name)
    pick.eef = eef_name
    pick.object = object_name

    # Twist to approach the object
    approach = TwistStamped()
    approach.header.frame_id = "world"
    approach.twist.linear.z = -1.0
    
    pick.setApproachMotion(approach, 0.03, 0.1)

    # Twist to lift the object
    lift = TwistStamped()
    lift.header.frame_id = "rg6_link_0"
    lift.twist.linear.z = -1.0
    pick.setLiftMotion(lift, 0.03, 0.1)

    return pick


def pick_and_place_object(corosect_obj, target_position):
    object_name = corosect_obj.object_name
    
    # get the object that handles the tasks
    task = core.Task()
    task.enableIntrospection()


    # add the current state as the first task in the pipeline
    task.add(stages.CurrentState("current"))

    # initialize planners for connect stages
    pipeline = core.PipelinePlanner()
    pipeline.planner = "RRTConnectkConfigDefault"
    planners = [(arm_group, pipeline),(eef_group,pipeline)]

    # add connect state
    task.add(stages.Connect("move to pick", planners))

    alternatives_pick = core.Alternatives("alternatives pick")

    alternatives_pick.insert(generate_pick(task, object_name, corosect_obj.object_type,"top_down"))
    alternatives_pick.insert(generate_pick(task, object_name, corosect_obj.object_type,"45_degree_angle"))
    alternatives_pick.insert(generate_pick(task, object_name, corosect_obj.object_type,"90_degree_angle"))

    # add pick to the list of tasks
    task.add(alternatives_pick)

    # Connect the Pick stage with the following Place stage
    task.add(stages.Connect("move to place position", planners))


    # Define the pose that the object should have after placing
    placePose = target_position

    # Generate Cartesian place poses for the object
    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["alternatives pick"])
    place_generator.object = object_name
    place_generator.pose = placePose

    simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")

    # Place container comprises placing, ungrasping, and retracting
    place = stages.Place(simpleUnGrasp, "Place")
    place.eef = eef_name
    place.object = object_name
    place.eef_frame = "rg6_link_0"

    # Twist to retract from the object
    retract = TwistStamped()
    retract.header.frame_id = "world"
    retract.twist.linear.z = 1.0
    place.setRetractMotion(retract, 0.03, 0.1)

    # Twist to place the object
    placeMotion = TwistStamped()
    placeMotion.header.frame_id = "rg6_link_0"
    placeMotion.twist.linear.z = 1.0
    place.setPlaceMotion(placeMotion, 0.03, 0.1)

    # Add the place pipeline to the task's hierarchy
    task.add(place)

    moveTo1 = stages.MoveTo("Move To Home", jointspace)
    moveTo1.group = arm_group
    moveTo1.setGoal(positions['home'])
    task.add(moveTo1)

    
    succesful_plan = False
    if task.plan():
        succesful_plan = True
        # task.publish(task.solutions[0])
        # task.execute(task.solutions[0])
    
    del pipeline
    del planners
    return task, succesful_plan

