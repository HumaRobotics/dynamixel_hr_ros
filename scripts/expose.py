#!/usr/bin/env python

import roslib; roslib.load_manifest("dynamixel_hr_ros")
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *

from dxl import *
from threading import Thread
import argparse

import logging





class DxlROS(Thread):
    
    def __init__(self,chain,rate=10,motors=None):
        Thread.__init__(self)
        self.chain=chain
        self.rate=rate
        self.motors=motors

        self.publishers=[]
        self.subscribers=[]        
        self.pub={}
        
        self.do_stop=False
        
        if self.motors==None: # Use IDs as names if no binding provided
            self.motors=self.chain.motors.keys()
        else:
            for id  in self.motors:
                if id not in self.chain.motors.keys():
                    raise Exception,"Cannot bind ROS name %s to non-existing motor ID %d"%(name,id)

        logging.info("Creating ROS elements")
        
        
        self.buildPublishers()
        self.buildSubscribers()
        
        self.start()

    def create_subscriber(self,topic,type,callback):
        logging.info("Creating subscriber on ROS topic %s"%topic)
        rospy.Subscriber(topic,type,callback)
        self.subscribers.append(topic)

    def create_publisher(self,topic,type):
        logging.info("Creating publisher on ROS topic %s"%topic)
        self.pub[topic]=rospy.Publisher(topic,type)
        self.publishers.append(topic)
        
        
    def run(self):
        r=rospy.Rate(float(self.rate))
        while not rospy.is_shutdown() and not self.do_stop:
            self.publish()
            r.sleep()
        
    def publish(self):
        data=[]
        for id in self.motors:
            v=self.chain.get_reg_si(id,"present_position")
            data.append(v)
        
        msg=Float64MultiArray()
        msg.data=data
        self.pub["/dxl/present_position"].publish(msg) 
    
        
        
    def stop(self):
        self.do_stop=True
    
    def buildPublishers(self):
        self.create_publisher("/dxl/present_position",Float64MultiArray)

                
    
    def buildSubscribers(self):
        self.create_subscriber("/dxl/enable",Bool,self.enable)
            
                        
        
    def enable(self,msg):
        if msg.data==True:
            self.chain.enable(self.bindings.keys())
        else:
            self.chain.disable(self.bindings.keys())


if __name__=="__main__":
        logging.basicConfig(level=logging.DEBUG)
        parser = argparse.ArgumentParser()
        parser.add_argument("--device", type=str,help="Serial device connected to the motor chain",default="/dev/ttyUSB0")
        parser.add_argument("--baudrate", type=int,help="Baudrate to use on the serial device",default=3000000)
        parser.add_argument("--rate", type=int,help="Publishing rate for ROS",default=10)
        args=parser.parse_args()
        
        logging.info("Connecting to chain on device %s at rate %d"%(args.device,args.baudrate) )
        chain=dxlchain.DxlChain(args.device,rate=args.baudrate)
        motors=chain.get_motor_list()
        if len(motors)==0:
            logging.error("No motors found, exiting")
            exit(1)
        logging.info("Motors found: "+str(motors))

        logging.info("Creating node and bindings")
        rospy.init_node("dxl")
        logging.basicConfig(level=logging.DEBUG)
        dxlros=DxlROS(chain,rate=args.rate)        
        rospy.spin()
