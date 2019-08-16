#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import random

pub = None
cautionFlag = 0
cautionTime = 0
pauseTime = 0
pauseFlag = 0

def pauseCallback(data):
    global pauseFlag
    pauseFlag = data.data

def cautionCallback(data):
    global cautionFlag
    cautionFlag = data.data
    pubWarning()

def pubWarning():
    global cautionFlag, pauseFlag, pub, cautionTime, pauseTime
    print "cautionFlag",cautionFlag
    print "pauseFlag",pauseFlag
    if not cautionFlag and not pauseFlag:
        pass #peace
    elif cautionFlag and not pauseFlag:
        if time.time() - cautionTime > 10:
            style = random.randint(1,3)
            if style == 1:
                pub.publish(String("Be careful, you are very close to me."))
            elif style == 2:
                pub.publish(String("Watch out, I am near you."))
            elif style == 3:
                pub.publish(String("Be careful of your body."))
            cautionTime = time.time()
            pass #caution
    else:
        if time.time() - pauseTime > 10:
            style = random.randint(1,3)
            if style == 1:
                pub.publish(String("I would wait until you pass through."))
            elif style == 2:
                pub.publish(String("I am waiting, you can go first."))
            elif style == 3:
                pub.publish(String("After you."))
            pauseTime = time.time()
            pass #pause

def main():
    global pub
    rospy.init_node('humancaution_chatter')
    rospy.Subscriber('/caution', Bool, cautionCallback)
    rospy.Subscriber('/pause', Bool, pauseCallback)
    pub = rospy.Publisher('response', String, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
