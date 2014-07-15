#!/usr/bin/env python

import roslib; roslib.load_manifest("dynamixel_hr_ros")
import rospy
from std_msgs.msg import *

from dynamixel_hr_ros.msg import ChainState,CommandPosition

from dxl import *
from threading import Thread
import argparse

import logging
logging.basicConfig(level=logging.DEBUG)





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
            self.motors=[ id for id in self.chain.motors.keys() if self.chain.motors[id].is_motor()]
        else:
            for id  in self.motors:
                if id not in self.chain.motors.keys():
                    raise Exception,"Cannot bind ROS name %s to non-existing motor ID %d"%(name,id)

        logging.info("Creating ROS elements")
        
        
        self.create_publisher("/dxl/chain_state",ChainState)
        self.create_subscriber("/dxl/enable",Bool,self.enable)
        self.create_subscriber("/dxl/command_position",CommandPosition,self.command_position)
        
        self.start()

    def create_subscriber(self,topic,type,callback):
        logging.info("Creating subscriber on ROS topic %s"%topic)
        rospy.Subscriber(topic,type,callback)
        self.subscribers.append(topic)

    def create_publisher(self,topic,type):
        logging.info("Creating publisher on ROS topic %s"%topic)
        self.pub[topic]=rospy.Publisher(topic,type)
        self.publishers.append(topic)
        

    def command_position(self,msg):
        # check command validity
        ids=msg.id
        angles=msg.angle
        speeds=msg.speed
        if len(ids)!=len(angles):
            logging.error("CommandPosition id and angle vector length do not match")
            return
        if len(speeds)>0 and len(ids)!=len(speeds):
            logging.error("CommandPosition id and speed vector length do not match")
            return
        cangles=[]
        cspeeds=[]
        for i in range(0,len(ids)):
            id=ids[i]
            if id not in self.motors:
                logging.error("CommandPosition id %d invalid"%id)
                return
            angle=angles[i]
            cangle=self.chain.motors[id].registers["goal_pos"].fromsi(angle)
            cangles.append(cangle)
            if len(speeds)>0:
                speed=speeds[i]
                cspeed=self.chain.motors[id].registers["moving_speed"].fromsi(speed)
                cspeeds.append(cspeed)
        
        if len(cspeeds)>0:
            self.chain.sync_write_pos_speed(ids,cangles,cspeeds)
        else:
            self.chain.sync_write_pos(ids,cangles)
                
                
        pass
        
    def run(self):
        r=rospy.Rate(float(self.rate))
        while not rospy.is_shutdown() and not self.do_stop:
            self.publish()
            r.sleep()
        
    def publish(self):
        angle=[]
        speed=[]
        moving=[]
        for id in self.motors:            
            a=self.chain.get_reg_si(id,"present_position")
            angle.append(a)
            #~ s=self.chain.get_reg_si(id,"present_speed")
            #~ speed.append(s)
            #~ m=self.chain.get_reg(id,"moving")
            #~ if m==0: moving.append(False)
            #~ else: moving.append(True)
        
        msg=ChainState()
        msg.id=self.motors
        msg.angle=angle
        #~ msg.speed=speed
        #~ msg.moving=moving
        
        
        self.pub["/dxl/chain_state"].publish(msg) 
    
        
        
    def stop(self):
        self.do_stop=True
    
        
    def enable(self,msg):
        if msg.data==True:
            self.chain.enable(self.motors)
        else:
            self.chain.disable(self.motors)


if __name__=="__main__":
        logging.basicConfig(level=logging.DEBUG)
        
        parser = argparse.ArgumentParser()
        parser.add_argument("--device", type=str,help="Serial device connected to the motor chain",default="/dev/ttyUSB0")
        parser.add_argument("--baudrate", type=int,help="Baudrate to use on the serial device",default=3000000)
        parser.add_argument("--rate", type=int,help="Publishing rate for ROS",default=10)
        args=parser.parse_args()
        
        logging.info("Connecting to chain on device %s at rate %d"%(args.device,args.baudrate) )
        chain=dxlchain.DxlChain(args.device,rate=args.baudrate,timeout=0.5)
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
