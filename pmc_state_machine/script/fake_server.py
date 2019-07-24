#!/usr/bin/env python

from std_srvs.srv import Trigger, TriggerResponse
import rospy

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

def main():
    rospy.init_node('fake_server')
    s1 = rospy.Service('triggerCaption', Trigger, executeCaption)
    s2 = rospy.Service('triggerGrasping', Trigger, executeGrasping)
    s3 = rospy.Service('triggerPlacing', Trigger, executePlacing)
    s4 = rospy.Service('triggerFetching', Trigger, executeFetching)
    s5 = rospy.Service('triggerStacking', Trigger, executeStacking)

    rospy.spin()

if __name__ == "__main__":
    main()
