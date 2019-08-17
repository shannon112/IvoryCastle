#!/usr/bin/env python
import rospy, os, sys
import re
from gtts import gTTS
from pygame import mixer
from std_msgs.msg import String
import tempfile
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

rospy.init_node('google_tts_bridge', anonymous = False)

soundhandle = SoundClient()
rospy.sleep(1)
soundhandle.stopAll()
rospy.loginfo('Starting TTS')

def speak(sentence):
	with tempfile.NamedTemporaryFile(delete=True) as fp:
		tts=gTTS(text=sentence,lang='zh-tw')
		tts.save("{}.mp3".format(fp.name))
		mixer.init()
		mixer.music.load('{}.mp3'.format(fp.name))
		mixer.music.play(1)
def speak_english(sentence):
	with tempfile.NamedTemporaryFile(delete=True) as fp:
		tts=gTTS(text=sentence,lang='en-us')
		tts.save("{}.mp3".format(fp.name))
		mixer.init()
		mixer.music.load('{}.mp3'.format(fp.name))
		mixer.music.play(1)



def is_chinese(uchar):
	if uchar >u'\u4e00' and uchar<=u'\u9fa5' :
		return True
	else:
		return False

def get_response(data):

	response = data.data
	#rospy.logwarn("I response:: %s",response)
	#rospy.loginfo("Response ::%s",response)
	#local tts
	#if is_chinese(response)==False:
		#soundhandle.say(response)

	if is_chinese(response.decode('utf-8'))==True:
		try:
			print("Now the language is chinese")
			speak(response.decode('utf-8'))
		except:
			print("changing state")
	else :
		try:
			print("Now the language is english")
			speak_english(response)
		except:
			print("changing state")

def listener():

	rospy.loginfo("Starting listening to response")
	rospy.Subscriber("response",String, get_response,queue_size=10)
	rospy.spin()


if __name__ == '__main__':

	listener()
