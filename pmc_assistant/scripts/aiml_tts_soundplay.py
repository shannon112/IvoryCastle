#!/usr/bin/env python
import rospy, os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

rospy.init_node('soundplay_tts_bridge', anonymous = True)
soundhandle = SoundClient()
rospy.sleep(1)
soundhandle.stopAll()
rospy.loginfo('Starting TTS')

def is_chinese(uchar):
	if uchar >u'\u4e00' and uchar<=u'\u9fa5' :
		return True
	else:
		return False

def get_response(data):
	response = data.data
	rospy.loginfo("Response ::%s",response)
	if is_chinese(response)==False:
		soundhandle.say(response)

def listener():
	rospy.loginfo("Starting listening to response")
	rospy.Subscriber("response",String, get_response,queue_size=10)
	rospy.spin()

if __name__ == '__main__':
	listener()
