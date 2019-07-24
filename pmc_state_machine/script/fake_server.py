#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

intent=""
intentId=0
sentId=0

def executeCaption(req):
    rospy.sleep(10.)
    return TriggerResponse(
        success=True,
        message="Caption is executed!"
    )

def executeGrasping(req):
    rospy.sleep(10.)
    return TriggerResponse(
        success=True,
        message="Grasping is executed!"
    )

def executePlacing(req):
    rospy.sleep(10.)
    return TriggerResponse(
        success=True,
        message="Placing is executed!"
    )

def executeFetching(req):
    rospy.sleep(10.)
    return TriggerResponse(
        success=True,
        message="Fetching is executed!"
    )

def executeStacking(req):
    rospy.sleep(10.)
    return TriggerResponse(
        success=True,
        message="Stacking is executed!"
    )

def sendingVCommand(req):
    global intent
    global intentId
    global sentId
    if sentId < intentId:
        sentId = intentId
        return TriggerResponse(
            success=True,
            message=intent
        )
    else:
        return TriggerResponse(
            success=True,
            message="Others"
        )

def storeIntent(data):
    global intent
    global intentId
    intent = data.data
    intentId += 1
    print "server echo /Intent = ",intent

def main():
    rospy.init_node('fake_server')
    rospy.Subscriber('/Intent', String, storeIntent)
    s1 = rospy.Service('triggerCaption', Trigger, executeCaption)
    s2 = rospy.Service('triggerGrasping', Trigger, executeGrasping)
    s3 = rospy.Service('triggerPlacing', Trigger, executePlacing)
    s4 = rospy.Service('triggerFetching', Trigger, executeFetching)
    s5 = rospy.Service('triggerStacking', Trigger, executeStacking)
    s6 = rospy.Service('triggerVCommand', Trigger, sendingVCommand)
    rospy.spin()

if __name__ == "__main__":
    main()
