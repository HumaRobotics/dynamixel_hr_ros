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




frames=[]

def add_frame(msg):
    frames.append((msg.id,msg.angle))

if __name__=="__main__":
        
        logging.basicConfig(level=logging.DEBUG)
        
        
        rospy.init_node("dxl_record")
        logging.basicConfig(level=logging.DEBUG)
        enabler=rospy.Publisher("/dxl/enable",Bool)        
        time.sleep(1)
        enabler.publish(False)
        time.sleep(1)
        
        rospy.Subscriber("/dxl/chain_state",ChainState,add_frame)
        start=time.time()
        print "Recording"
        while not rospy.is_shutdown() and (time.time()-start)<5:
            time.sleep(0.1)
            
        f=open("recorded","w")        
        f.write(json.dumps(frames))
        f.close()
            
            
        
        
