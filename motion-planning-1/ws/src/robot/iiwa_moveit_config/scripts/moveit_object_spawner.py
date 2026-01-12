#!/usr/bin/env python3

'''
This file is made to make object spawn in moveit. 
It does this by adding the object to the moveit planning scene interface.
The visuals for the object are chosen based upon the type of object is taken.
'''

from moveit_commander import PlanningSceneInterface
import yaml

meshes = None

# load in the paths to the meshes
with open('/home/corosect/ws_moveit/src/iiwa_moveit_config/scripts/configs/mesh_path.yaml') as f:
    
    meshes = yaml.load(f, Loader=yaml.FullLoader)


def spawn_object(corosect_obj):
    # create and object of the planningSceneInterface
    psi = PlanningSceneInterface(synchronous=True)

    # add the object with provided details to the planning scene.
    psi.add_mesh(corosect_obj.object_name, corosect_obj.object_pose, corosect_obj.mesh)