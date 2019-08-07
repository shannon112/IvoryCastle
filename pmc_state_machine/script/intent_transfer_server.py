#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from smach_msgs.msg import SmachContainerStatus

intent=""
intentId=0
sentId=0
waitingFlag = 1

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

def resetStatus(data):
    global waitingFlag
    if data.path == "/SM_ROOT":
        if data.active_states[0] == "VoiceCommand" and waitingFlag == 0:
            pub = rospy.Publisher('chatter', String, queue_size=10)
            pub.publish(String("reset current state to blank"))
            waitingFlag = 1
        elif data.active_states[0] == "VoiceCommand":
            waitingFlag = 1
        elif  data.active_states[0] == "DELIVERING":
            waitingFlag = 0
        elif  data.active_states[0] == "COLLECTING":
            waitingFlag = 0

def main():
    rospy.init_node('intent_transfer_server')
    # subscribing /Intent continuously and call service once when change
    rospy.Subscriber('/Intent', String, storeIntent)
    # subscribing /server_pmc/smach/container_status and pub to /chatter once when change
    rospy.Subscriber('/server_pmc/smach/container_status', SmachContainerStatus, resetStatus)
    s = rospy.Service('triggerVCommand', Trigger, sendingVCommand)
    rospy.spin()

if __name__ == "__main__":
    main()
