#!/usr/bin/env python
import rospy
import tf
import os
import sys
import numpy as np
import smach
import smach_ros
from math import atan2, pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose
#from neuronbot_msgs.srv import Alignment, AlignmentResponse
#from ira_factory_msgs.msg import RobotStatus
from ira_factory_msgs.srv import RequestRobotStatus, UpdateRobotStatus, RobotTF
from pmc_navigation.srv import navigoal, navigoalResponse

br = None
listener = None
twistPub = None
namespace = None
tagFrame = None
robotBaseFrame = None
tfReq = None
rotateVel = 0.25 #0.10
transVel = 0.25 #0.05
angleTH = 0.05 #0.05
nearstTagDist = None
hpause = False
rx = None
ry = None
rtheta =None
def hpausehandler(x,y,state):
    while(hpause):
        print("people! waiting")
    if(state==1):
        movea(x,y,0)
    elif(state==3):
        movec(x,y,0)
    else:
        moveb(x,y,0)
def hpausedegree(theta):
    while(hpause):
        print("waiting")
    absRotation(theta,"map")



def movea(goalx,goaly,goaltheta):
    global twistPub, tfReq, transVel ,hpause
    print("A")
    ns = rospy.myargv(argv=sys.argv)[1]
    goal_frame=os.path.join(ns,'base_link')
    base_frame='/map'
    resp = tfReq(goal_frame, base_frame)
    quat = tuple(resp.quat)
    currY = resp.trans[1]
    currX= resp.trans[0]
    print(currX)
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    ts = Twist()
    ts.linear.x = transVel
    r = rospy.Rate(10)
    if(abs(currX-goalx)>0.05 and hpause != True):
        absRotation(0,"map")
    ts.linear.x = transVel
    while(abs(currX-goalx)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currX = resp.trans[0]
        print(hpause)
        #print("a:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    ts.linear.x = transVel
    if(abs(currY-goaly)>0.05 and hpause != True):
        absRotation(1.57,"map")
    while(abs(currY-goaly)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currY = resp.trans[1]
        print(hpause)
        #print("a:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    if(hpause):
        hpausehandler(goalx,goaly,1)
    else:
        return True
def moveb(goalx,goaly,goaltheta):
    global twistPub, tfReq, transVel ,hpause
    print("B")
    ns = rospy.myargv(argv=sys.argv)[1]
    goal_frame=os.path.join(ns,'base_link')
    base_frame='/map'
    resp = tfReq(goal_frame, base_frame)
    quat = tuple(resp.quat)
    currY = resp.trans[1]
    currX= resp.trans[0]
    print(currX)
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    ts = Twist()
    ts.linear.x = transVel
    r = rospy.Rate(10)
    #if(abs(currY-goaly)>0.05 and hpause != True):
    #   absRotation(1.57,"map")
    ts.linear.x = -transVel
    while(abs(currY-goaly)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currY = resp.trans[1]
        print(hpause)
        #print("b:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    ts.linear.x = -transVel
    if(abs(currX-goalx)>0.05 and hpause != True):
        absRotation(0,"map")
    while(abs(currX-goalx)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currX = resp.trans[0]
        print(hpause)
        #print("b:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    if(hpause):
        hpausehandler(goalx,goaly,2)
    else:
        return True
def movec(goalx,goaly,goaltheta):
    global twistPub, tfReq, transVel ,hpause
    print("C")
    ns = rospy.myargv(argv=sys.argv)[1]
    goal_frame=os.path.join(ns,'base_link')
    base_frame='/map'
    resp = tfReq(goal_frame, base_frame)
    quat = tuple(resp.quat)
    currY = resp.trans[1]
    currX= resp.trans[0]
    print(currX)
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    ts = Twist()
    ts.linear.x = transVel
    r = rospy.Rate(10)
    if(abs(currX-goalx)>0.05 and hpause != True):
        absRotation(0,"map")
    ts.linear.x = transVel
    while(abs(currX-goalx)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currX = resp.trans[0]
        #print("c:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    ts.linear.x = transVel
    if(abs(currY-goaly)>0.05 and hpause != True):
        absRotation(-1.57,"map")
    while(abs(currY-goaly)>0.05 and hpause != True):
        resp = tfReq(goal_frame, base_frame)
        currY = resp.trans[1]
        #print("c:moveing")
        twistPub.publish(ts)
        r.sleep()
    ts.linear.x = 0.0
    twistPub.publish(ts)
    if(hpause):
        hpausehandler(goalx,goaly,3)
    else:
        return True


def absRotation(tarAngle, base_frame):
    global twistPub, rotateVel, angleTH,tfReq ,hpause
    ns = rospy.myargv(argv=sys.argv)[1]
    goal_frame=os.path.join(ns,'base_link')
    #base_frame = "charger"
    resp = tfReq(goal_frame , base_frame)
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    #print("c",currYaw)
    # angleDiff is always smaller than 3.14, and used to judge whether the angle is reached with angleTH
    angleDiff =  abs(tarAngle - currYaw) if abs(tarAngle - currYaw) < 3.14 else (6.28 - abs(tarAngle - currYaw))
    ts = Twist()
    r = rospy.Rate(10)
    #print("rotate",angleDiff)
    while angleDiff> angleTH and not rospy.is_shutdown() and hpause != True:
      # For a specific range, the robot should turn right
        if tarAngle - currYaw > 3.14 or ((tarAngle - currYaw) < 0 and (tarAngle - currYaw) > -3.14):
            ts.angular.z = -rotateVel # Turn right
        else:
            ts.angular.z = rotateVel  # Turn left
        twistPub.publish(ts)
        resp = tfReq(goal_frame, base_frame)
        quat = tuple(resp.quat)
        currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    # Update angleDiff
        angleDiff =  abs(tarAngle - currYaw) if abs(tarAngle - currYaw) < 3.14 else (6.28 - abs(tarAngle - currYaw))
        r.sleep()

    ts.angular.z = 0.0
    twistPub.publish(ts)
    if(hpause):
        hpausedegree(tarAngle)

def callback(lmsg):
    global hpause
    hpause=lmsg.data

def naviflow(req):
    state=req.goal_status
    if(state==1):
        movea(0.25,0.58,0)#-0.6 -0.9
        #movea(-0.6,-0.9,0)#-0.6 -0.9
        goal = "A"
    elif(state==3):
        movec(0.25,-0.58,0) #-0
        #movec(-0.6,0.9,0) #-0
        goal = "B"
    else:
        #moveb(0,0,0)
        moveb(-0.1,0,0)
        goal = "O"
    return navigoalResponse(
        success=True,
        message="navigate to "+goal+" is executed!"
    )


if __name__ == '__main__':
    rospy.init_node('pmcT', anonymous=True)
    rospy.Subscriber("/pause", Bool, callback)
    ns = rospy.myargv(argv=sys.argv)[1]
    cmdVelTopic=os.path.join(ns,'diff_drive_controller','cmd_vel')
    twistPub = rospy.Publisher(cmdVelTopic, Twist, queue_size=1)
    rospy.wait_for_service('robot_tf_server')
    tfReq = rospy.ServiceProxy('robot_tf_server', RobotTF)
    navi_srv = rospy.Service('triggerNavigating',navigoal, naviflow)
    print("aaa")

    rospy.spin()
