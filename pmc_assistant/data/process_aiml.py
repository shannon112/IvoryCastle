#!/usr/bin/env python
import aiml
import sys
import os
os.chdir('/home/jkllbn2563/catkin_ws/src/ros_aiml/data') 
bot=aiml.Kernel()
if os.path.isfile("standard.brn"):
	bot.bootstrap(brainFile="standard.brn") 
else:
	bot.bootstrap(learnFile="startup.xml",commands="load aiml b")
	bot.saveBrain("standard.brn")

	while True: print bot.respond(raw_input("Enter input >"))