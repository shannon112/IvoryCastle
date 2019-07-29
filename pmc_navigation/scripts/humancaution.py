#!/usr/bin/env python
import rospy
import tf
import os
import sys
import numpy as np
import smach
import smach_ros
from math import cos ,sin ,pi ,sqrt ,atan2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose , Point
#from neuronbot_msgs.srv import Alignment, AlignmentResponse
#from ira_factory_msgs.msg import RobotStatus
from ira_factory_msgs.srv import RequestRobotStatus, UpdateRobotStatus, RobotTF
from people_msgs.msg import PositionMeasurementArray



CPub = None
PPub = None
namespace = None
tagFrame = None
robotBaseFrame = None
tfReq = None
rotateVel = 0.25 #0.10
transVel = 0.25 #0.05
angleTH = 0.05 #0.05
nearstTagDist = None
hpause1 = False
hcaution1 =False
hpause2 = False
hcaution2 =False
currX = 10
currY = 10
currYaw=None
def problempub():
    global hpause1,hpause2, hcaution1,hcaution2 ,CPub ,PPub
    ctopic = Bool()
    ptopic = Bool()
    if(hcaution1==True or hcaution2 ==True):
        ctopic.data = True
    else:
        ctopic.data = False
    if(hpause1==True or hpause2 ==True):
        ptopic.data = True
    else:
        ptopic.data = False
    CPub.publish(ctopic)
    PPub.publish(ptopic)


def ppdistance(x1,y1,x2,y2):
    return sqrt(((x1-x2)**2)+((y1-y2)**2))

def humancallback1(hmsg):
    global hpause1 , hcaution1 ,CPub ,PPub ,tfReq ,currX ,currY ,currYaw
    ns = rospy.myargv(argv=sys.argv)[1]
    link = os.path.join(ns,'base_link')
    resp = tfReq(link,"/map")
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    currY = resp.trans[1]
    currX= resp.trans[0]
    #rpos= Point(currX,currY)
    hcaution1 =False
    hpause1 = False
    ctopic = Bool()
    ptopic = Bool()
    for i in range(len(hmsg.people)):
	   if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<1.2):
            hcaution1 =True
            print("warmimg")
            if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<0.8):
                hpause1 = True
                print("stop")
                break
    problempub()
    #ctopic.data = hcaution
    #ptopic.data = hpause
    #if(hcaution):
    #CPub.publish(ctopic)
    #if(hpause):
    #PPub.publish(ptopic)
def humancallback2(hmsg):
    global hpause2 , hcaution2 ,CPub ,PPub ,tfReq ,currX ,currY ,currYaw
    #resp = tfReq("/neuronbot/mmp0/base_link","/map")
    #quat = tuple(resp.quat)
    #currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    #currY = resp.trans[1]
    #currX= resp.trans[0]
    #rpos= Point(currX,currY)
    hcaution2 =False
    hpause2 = False
    ctopic = Bool()
    ptopic = Bool()
    for i in range(len(hmsg.people)):
       if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<1.2):
            hcaution2 =True
            print("warmimg")
            if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<0.8):
                hpause2 = True
                print("stop")
                break
    problempub()
    #ctopic.data = hcaution
    #ptopic.data = hpause
    #if(hcaution):
    #CPub.publish(ctopic)
    #if(hpause):
    #PPub.publish(ptopic)


if __name__ == '__main__':
    rospy.init_node('humancause', anonymous=True)
    rospy.wait_for_service('robot_tf_server')
    tfReq = rospy.ServiceProxy('robot_tf_server', RobotTF)
    rospy.Subscriber("/people_tracker_measurements2", PositionMeasurementArray, humancallback2)
    rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, humancallback1)
    cautiontopic = "/caution"
    pausetopic = "/pause"
    CPub = rospy.Publisher(cautiontopic, Bool, queue_size=1)
    PPub = rospy.Publisher(pausetopic, Bool, queue_size=1)

    rospy.spin()
