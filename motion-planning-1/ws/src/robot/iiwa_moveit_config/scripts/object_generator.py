#!/usr/bin/env python3

'''
This file is a temporary dummy file that randomly generates objects.
This should be replaced in the future with the image recognition software
that recognizes and object and returns it's position
'''
import time
import random
from geometry_msgs.msg import PoseStamped
from corosect_object import corosect_object

object_types = ['water_tray','bird_feeder']

def object_creation():
    # to make sure the object has a unique name, the current unix time is appended to it's name
    object_name = "dummy_object "+str(time.time())

    # The type of object is randomly chosen such that any object could be spawned
    object_type = random.choice(object_types)
    
    # generate a corosect object
    item = corosect_object(object_name, object_type) 

    return item