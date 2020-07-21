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
import warnings
from std_msgs.msg import String
import time
warnings.filterwarnings("ignore")
import requests
import bs4

from bs4 import BeautifulSoup

rospy.init_node('aiml_server')
mybot = aiml.Kernel()
response_publisher = rospy.Publisher('response',String,queue_size=10)
pub=rospy.Publisher('/Intent',String,queue_size=10)

rate=rospy.Rate(1)
localtime=time.asctime(time.localtime(time.time()))

def is_chinese(uchar):
	if uchar >u'\u4e00' and uchar<=u'\u9fa5' :
		return True
	else:
		return False



def load_aiml(xml_file):

	data_path = rospy.get_param("aiml_path")
	#data_path="/home/jkllbn2563/catkin_ws/src/ros_aiml/data"
	print data_path
	os.chdir(data_path)


	if os.path.isfile("standard.brn"):
		mybot.bootstrap(brainFile = "standard.brn")

	else:
		mybot.bootstrap(learnFiles = xml_file, commands = "load aiml b")
		mybot.saveBrain("standard.brn")

def Wiki_response(data):
	
	response = data.data
	rospy.loginfo("Start to process ::%s",response)
	result=re.search(r"維基百科(?P<state>.+)",response)
	result_number=re.search(r"威力彩(?P<state>.+)",response)
	if result:
		state=result.group('state')
		print(state)
		res=requests.get("https://zh.wikipedia.org/wiki/{}".format(state))
		soup=BeautifulSoup(res.text,"lxml")
		para=soup.select_one(".mw-parser-output p").text
		rate.sleep()
		response_publisher.publish(para)		#pub.publish(state)
	if result_number:
		#state=result_number.group('state')
		number1=""
		url="http://www.taiwanlottery.com.tw"
		html=requests.get(url)
		html.raise_for_status()
		objSoup=bs4.BeautifulSoup(html.text,'lxml')
		dataTag=objSoup.select(".contents_box02")
		print("串列長度",len(dataTag))
		while(len(dataTag)==0):
		
			dataTag=objSoup.select(".contents_box02")
		balls=dataTag[0].find_all("div",{"class":"ball_tx ball_green"})
		

		#print("第一區",end="")
		for i in range(6,len(balls)):
			#print(balls[i].text, end="")
			number1=number1+","+balls[i].text
		redball=dataTag[0].find_all("div",{"class":"ball_red"})
		number2=redball[0].text
		print("第二區:{}".format(redball[0].text))
		print(number1)
		total="First section is "+number1+"and the special number is "+number2
		response_publisher.publish(total)

		












