#!/usr/bin/env python
import rospy
import tf
import os
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
hpause = False
hcaution =False
currX = 10
currY = 10
currYaw=None


def ppdistance(x1,y1,x2,y2):
    return sqrt(((x1-x2)**2)+((y1-y2)**2))

def humancallback1(hmsg):
    global hpause , hcaution ,CPub ,PPub ,tfReq ,currX ,currY ,currYaw
    resp = tfReq("/neuronbot/mmp0/base_link","/map")
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    currY = resp.trans[1]
    currX= resp.trans[0]
    #rpos= Point(currX,currY)
    hcaution =False
    hpause = False
    ctopic = Bool()
    ptopic = Bool()
    for i in range(len(hmsg.people)):
	   if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<1.2):
            hcaution =True
            print("warmimg")
            if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<0.8):
                hpause = True
                print("stop")
                break
    ctopic.data = hcaution
    ptopic.data = hpause
    if(hcaution):
        CPub.publish(ctopic)
    if(hpause):
        PPub.publish(ptopic)
def humancallback2(hmsg):
    global hpause , hcaution ,CPub ,PPub ,tfReq ,currX ,currY ,currYaw
    #resp = tfReq("/neuronbot/mmp0/base_link","/map")
    #quat = tuple(resp.quat)
    #currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    #currY = resp.trans[1]
    #currX= resp.trans[0]
    #rpos= Point(currX,currY)
    hcaution =False
    hpause = False
    ctopic = Bool()
    ptopic = Bool()
    for i in range(len(hmsg.people)):
       if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<1.2):
            hcaution =True
            print("warmimg")
            if(ppdistance(hmsg.people[i].pos.x,hmsg.people[i].pos.y,currX,currY)<0.8):
                hpause = True
                print("stop")
                break
    ctopic.data = hcaution
    ptopic.data = hpause
    if(hcaution):
        CPub.publish(ctopic)
    if(hpause):
        PPub.publish(ptopic)


if __name__ == '__main__':
    rospy.init_node('humancause', anonymous=True)
    rospy.Subscriber("/people_tracker_measurements2", PositionMeasurementArray, humancallback2)
    rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, humancallback1)
    cautiontopic = "/caution"
    pausetopic = "/pause"
    CPub = rospy.Publisher(cautiontopic, Bool, queue_size=1)
    PPub = rospy.Publisher(pausetopic, Bool, queue_size=1)
    rospy.wait_for_service('robot_tf_server')
    tfReq = rospy.ServiceProxy('robot_tf_server', RobotTF)
    rospy.spin()
