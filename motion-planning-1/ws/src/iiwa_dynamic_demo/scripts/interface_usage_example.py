#!/usr/bin/env python3
#Example of using talker_randomy.py from another python file
# (we pretend this file has originated from another group)
#Make sure the master node is running before running this file
#Run by command: python3 interface_usage_example
import rospy
from talker_random import talker
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass