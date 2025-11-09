#!/usr/bin/env python3
import os
import re
import time
from pathlib import Path
import rospy
from std_msgs.msg import Float64

pattern = re.compile(r'Final solution cost\s*([0-9]+(?:\.[0-9]+)?)')

def get_latest_log_folder(base_log_dir):
    subdirs = [d for d in Path(base_log_dir).iterdir() if d.is_dir()]
    if not subdirs:
        rospy.logwarn(f"No subfolders found in {base_log_dir}")
        return None
    return max(subdirs, key=lambda d: d.stat().st_mtime)

def get_rosout_log_path():
    base_log_dir = os.path.expanduser("~/.ros/log")
    latest_folder = get_latest_log_folder(base_log_dir)
    if not latest_folder:
        return None
    rosout_path = Path(latest_folder) / "rosout.log"
    if rosout_path.exists():
        return rosout_path
    else:
        rospy.logwarn(f"No rosout.log found in {latest_folder}")
        return None

def follow_rosout(rosout_path):
    with open(rosout_path, 'r', errors='ignore') as f:
        f.seek(0, os.SEEK_END)
        while not rospy.is_shutdown():
            line = f.readline()
            if not line:
                time.sleep(0.5)
                continue
            yield line

if __name__ == "__main__":
    rospy.init_node('publish_final_solution_cost_from_log', anonymous=True)
    pub = rospy.Publisher('/chatter', Float64, queue_size=1)

    rosout_path = get_rosout_log_path()

    last_cost = None

    try:
        for line in follow_rosout(rosout_path):
            match = pattern.search(line)
            if match:
                cost = float(match.group(1))
                if cost != last_cost:
                    pub.publish(cost)
                    rospy.loginfo(f"Published new Final solution cost: {cost}")
                    last_cost = cost
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass