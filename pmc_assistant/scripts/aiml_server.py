#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import aiml
import os
import sys
import re
from gtts import gTTS
from pygame import mixer
import tempfile

from std_msgs.msg import String

rospy.init_node('aiml_server')
mybot = aiml.Kernel()
response_publisher = rospy.Publisher('response',String,queue_size=10)
pub=rospy.Publisher('/Intent',String,queue_size=10)
rate=rospy.Rate(1)

def is_chinese(uchar):
	if uchar >u'\u4e00' and uchar<=u'\u9fa5' :
		return True
	else:
		return False

def load_aiml(xml_file):
	data_path = rospy.get_param("aiml_path")
	print data_path
	os.chdir(data_path)

	if os.path.isfile("standard.brn"):
		mybot.bootstrap(brainFile = "standard.brn")

	else:
		mybot.bootstrap(learnFiles = xml_file, commands = "load aiml b")
		mybot.saveBrain("standard.brn")

def callback(data):
	input = data.data
	response = mybot.respond(input)
	result=re.search(r"the current state is (?P<state>.+)",response)
	if result:
		state=result.group('state')
		pub.publish(state)
		rate.sleep()
	response=re.sub(r",and the current state is (?P<state>.+)","",response)

	rospy.logwarn("I heard:: %s",data.data)
	rospy.logwarn("I response:: %s",response)
	response_publisher.publish(response)

def listener():
	rospy.loginfo("Starting ROS AIML Server")
	rospy.Subscriber("chatter", String, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	load_aiml('startup.xml')
	listener()
