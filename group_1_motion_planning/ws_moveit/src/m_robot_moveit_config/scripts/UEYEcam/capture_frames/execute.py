import os
import subprocess

try:
    from autofocus import calculate_Automatic_focus
except:    
    from UEYEcam.capture_frames.autofocus import calculate_Automatic_focus

camdev_id=1
capture_seconds=1
camera_height_cm=78  # ${afmin} -fmax ${afmax} 
temp_capture_dir='/home/moveit/corosect_ws/src/corosect/corosect/m_robot_moveit_config/scripts/UEYEcam/capture_frames/rgb_input'
relative_dir_in_which_ueye_RUNNABLE_is_installed="/home/moveit/corosect_ws/src/corosect/corosect/m_robot_moveit_config/scripts/UEYEcam/capture_frames/ueye/rec"

def capture():
    try:
        os.makedirs(temp_capture_dir)
        print(f"{temp_capture_dir} dir has been created!")
    except FileExistsError:
        # print(f"{temp_capture_dir} already exists")
        pass

    afmin,afmax=calculate_Automatic_focus(camera_height_cm)
    # afmin=330
    # afmax=360
        
    # subprocess.run(["ueye/rec",'-camdev',str(camdev_id),'-fmin',str(afmin),'-fmax',str(afmax),'-temp_capture_dir',temp_capture_dir,'-cs',str(capture_seconds)])
    subprocess.run([relative_dir_in_which_ueye_RUNNABLE_is_installed,\
                                                                    '-camdev',str(camdev_id),
                                                                    '-fmin',str(afmin),
                                                                    '-fmax',str(afmax),
                                                                    '-temp_capture_dir',temp_capture_dir,
                                                                    '-cs',str(capture_seconds)])

if __name__=="__main__":
    capture()

