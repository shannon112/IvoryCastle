#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

intent=""
intentId=0
sentId=0

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
    rospy.init_node('intent_transfer_server')
    rospy.Subscriber('/Intent', String, storeIntent)
    s = rospy.Service('triggerVCommand', Trigger, sendingVCommand)
    rospy.spin()

if __name__ == "__main__":
    main()
