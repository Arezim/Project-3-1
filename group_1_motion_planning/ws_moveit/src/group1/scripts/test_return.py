from threading import Thread
import time
import json

from PilotDetection.final_panos import Worm_meta_data
from PilotDetection.post_process_utils import get_worm_statistics

import shutil



class threading_tranding():
    def __init__(self):
        
        self.thread_list=[]

    def append_to_total_threads(self):
        self.new_thread = self.create_thread(self.vi_task)
        self.thread_list.append(self.new_thread)

    def execute(self, task_thread):
        task_thread.start()
        return True

    def create_thread(self, task):
        return Thread(target=task)
    
    def start_last_thread(self):
        return self.thread_list[-1]

    def vi_task(self):
        users_folder = f'predict_{time.ctime().replace(" ", "_")}'
        project = r'/home/moveit/corosect_ws/src/corosect/corosect/group1/scripts/'
        name = f'output_results/{users_folder}'
        image_path = '/home/moveit/corosect_ws/src/corosect/corosect/group1/scripts/UEYEcam/capture_frames/rgb_input/frame_01.png'
        

        # Heuristics
        camera_height = 30  # in cm
        conf = 0.05
        max_det = 500

        #VI_results for publishing
        # worm_data = Worm_meta_data(project, "VI_results/")
        # worm_data.get_json(image_path)

        #Write timestamp for historical data
        worm_data = Worm_meta_data(project, name,camera_height=camera_height, conf=conf, max_det=max_det)
        worm_data.get_json(image_path)

        shutil.copyfile(project+name+"/output.json","/home/moveit/corosect_ws/src/corosect/corosect/group1/scripts/VI_results/output.json")

        #Write VITaskStatus property to Finished
        filename=project+"/VI_properties.json"
        with open(filename,"r") as jf:
            data=json.load(jf)
        data["VITaskStatus"]="Finished"   

        with open(filename, 'w') as json_file:
                json.dump(data, json_file,
                        indent=4)
        

if __name__ == "__main__":
    t = threading_tranding()
    t.vi_task()
  