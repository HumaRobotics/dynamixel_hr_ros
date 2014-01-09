#!/usr/bin/env python

import roslib; roslib.load_manifest("dynamixel_hr_ros")
import rospy
from std_msgs.msg import *
import json

from dynamixel_hr_ros.msg import *

from dxl import *
import logging
import time

logging.basicConfig(level=logging.DEBUG)





if __name__=="__main__":
        
        logging.basicConfig(level=logging.DEBUG)
        
        
        rospy.init_node("dxl_replay")
        enabler=rospy.Publisher("/dxl/enable",Bool)
        commander=rospy.Publisher("/dxl/command_position",CommandPosition)
        time.sleep(1)
        enabler.publish(True)
        time.sleep(1)

        print "Replaying"
        
        f=open("recorded","r")        
        frames=json.loads(f.read())
        print frames
        f.close()
        
        for f in frames:
            command=CommandPosition()
            command.id=f[0]
            command.angle=f[1]
            command.speed=[0.1]*len(f[0])
            commander.publish(command)
            print command
            time.sleep(0.1)
            
        enabler.publish(False)
        
