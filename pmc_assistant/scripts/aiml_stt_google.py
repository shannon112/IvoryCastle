#!/usr/bin/env python
#-*- coding: utf-8 -*-
import speech_recognition
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/kinetic/share")
import rospy
from std_msgs.msg import String
#from pynupt.keyboard import Key,controller
import keyboard

r=speech_recognition.Recognizer()

pub=rospy.Publisher('/chatter',String,queue_size=10)
rospy.init_node('google_stt_bridge',anonymous=False)
rate=rospy.Rate(1)

def listen():
	with speech_recognition.Microphone() as source:
		r.adjust_for_ambient_noise(source, duration=0.5)
		audio=r.record(source,duration=3)
		#audio=r.listen(source)
		#print("You said " + r.recognize_google(audio,language='zh-TW'))
		#print("You said " + r.recognize_google(audio,language='en-US'))

		#r.recognize_google(audio,language='en-US')
		#pub.publish(r.recognize_google(audio,language='zh-TW').encode('utf-8'))
		rospy.logwarn("I heard:: %s",r.recognize_google(audio,language='en-US'))
		pub.publish(r.recognize_google(audio,language='en-US'))

		rate.sleep()

while not rospy.is_shutdown():
	#keyboard.wait('enter')
	raw_input()
	try:
		rospy.logwarn("<<<<<< you can start to talking >>>>>>>>>")
		listen()
	except:
		#rospy.loginfo("Could not understand ")
		pass
