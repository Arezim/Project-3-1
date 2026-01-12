#!/usr/bin/env python3

from object_generator import object_creation
from moveit_object_spawner import spawn_object
from moveit_commander import PlanningSceneInterface

# generate an object that will have to be moved
corosect_obj = object_creation()
print(corosect_obj)

# randomly initialize the current and target position of the object
corosect_obj.randomly_initialize()

# spawn the object in in moveit
spawn_object(corosect_obj)
print("spawned in the object")

# plan a motion for the object
# corosect_obj.plan()

# execute the planned motion
corosect_obj.execute()

# remove the object from the world
psi = PlanningSceneInterface(synchronous=True)
psi.remove_world_object(corosect_obj.object_name)
