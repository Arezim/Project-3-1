from geometry_msgs.msg import PoseStamped
from pick_and_place import pick_and_place_object
import random
import yaml

class corosect_object:
    def __init__(self, object_name, object_type):
        self.object_name = object_name
        self.object_type = object_type
        meshes = None

        # load in the paths to the meshes
        with open('/home/corosect/ws_moveit/src/iiwa_moveit_config/scripts/configs/mesh_path.yaml') as f:    
            meshes = yaml.load(f, Loader=yaml.FullLoader)
        
        self.mesh = meshes[self.object_type]

        self.object_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.succesful_plan = False

    def randomly_initialize(self):
        self.object_pose.header.frame_id = "world"
        self.object_pose.pose.position.x = random.uniform(-0.25,0.25)
        self.object_pose.pose.position.y = random.uniform(-0.25,0.25)
        self.object_pose.pose.position.z = 0.9

        self.target_pose.header.frame_id = "world"
        self.target_pose.pose.position.x = random.uniform(-0.25,0.25)
        self.target_pose.pose.position.y = random.uniform(-0.25,0.25)
        self.target_pose.pose.position.z = 0.9

    def plan(self):
        self.task, self.succesful_plan = pick_and_place_object(self, self.target_pose)

    def execute(self):
        if self.succesful_plan:
            print("executing plan")
            self.task.execute(self.task.solutions[0])
            self.task.clear()
        else:
            print("could not plan a motion towards wanted position / no planning has been done yet")

    def __repr__(self):
        return f"object: {self.object_name}\ntype: {self.object_type}\nposition:\n{self.object_pose}"