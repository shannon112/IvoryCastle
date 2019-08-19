#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose 
from pmc_msgs.srv import PoseSrv
from smach_ros import ServiceState

objName = 'mouse'
#objName = 'banana'
	
class GraspMachine(smach.StateMachine):
	def __init__(self):
		global objName
		smach.StateMachine.__init__(self, outcomes = ['temp'])
		

		with self:
			smach.StateMachine.add('IDLE0000',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE000',
											 'aborted':'IDLE000',
											 'preempted':'IDLE000'})
			smach.StateMachine.add('IDLE000',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE001',
											 'aborted':'IDLE0000',
											 'preempted':'IDLE002'})
			smach.StateMachine.add('IDLE001',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE003',
											 'aborted':'IDLE003',
											 'preempted':'IDLE004'})
			smach.StateMachine.add('IDLE002',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE003',
											 'aborted':'IDLE004',
											 'preempted':'IDLE004'})
			smach.StateMachine.add('IDLE003',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE005',
											 'aborted':'IDLE005',
											 'preempted':'IDLE006'})
			smach.StateMachine.add('IDLE004',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE005',
											 'aborted':'IDLE006',
											 'preempted':'IDLE006'})
			smach.StateMachine.add('IDLE005',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE00',
											 'aborted':'IDLE00',
											 'preempted':'IDLE01'})
			smach.StateMachine.add('IDLE006',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE00',
											 'aborted':'IDLE01',
											 'preempted':'IDLE01'})
			smach.StateMachine.add('IDLE00',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE1',
											 'aborted':'IDLE2',
											 'preempted':'IDLE3'})
			smach.StateMachine.add('IDLE01',
								ServiceState('/triggerStacking', Trigger),
								transitions={'succeeded':'IDLE1',
											 'aborted':'IDLE2',
											 'preempted':'IDLE3'})
			for i in range (0, 50):
				IDLE1 = "IDLE"+str(3*i+1)
				IDLE2 = "IDLE"+str(3*i+2)
				IDLE3 = "IDLE"+str(3*i+3)
				IDLE4 = "IDLE"+str(3*i+4)
				IDLE5 = "IDLE"+str(3*i+5)
				IDLE6 = "IDLE"+str(3*i+6)
				smach.StateMachine.add(IDLE1,
									   ServiceState('/triggerStacking', Trigger),
									   transitions={'succeeded':IDLE4,
													'aborted':IDLE4,
													'preempted':IDLE5})
				smach.StateMachine.add(IDLE2,
									   ServiceState('/triggerStacking', Trigger),
									   transitions={'succeeded':IDLE4,
													'aborted':IDLE5,
													'preempted':IDLE6})
				smach.StateMachine.add(IDLE3,
									   ServiceState('/triggerStacking', Trigger),
									   transitions={'succeeded':IDLE5,
													'aborted':IDLE6,
													'preempted':IDLE6})
			
			for i in range (0, 10):
				IDLE1 = "IDLEE"+str(2*i)
				IDLE2 = "IDLEE"+str(2*i+1)
				IDLE3 = "IDLE"+str(15*i+1)
				IDLE4 = "IDLE"+str(15*i+3)
				IDLE5 = "IDLE"+str(15*i+13)
				IDLE6 = "IDLE"+str(15*i+15)
				smach.StateMachine.add(IDLE1,
								 ServiceState('/triggerStacking', Trigger),
								 transitions={'succeeded':IDLE3,
											'aborted':IDLE3,
											'preempted':IDLE5})
				smach.StateMachine.add(IDLE2,
								 ServiceState('/triggerStacking', Trigger),
								 transitions={'succeeded':IDLE4,
											'aborted':IDLE4,
											'preempted':IDLE6})		
			
			smach.StateMachine.add('IDLE151',
								   ServiceState('/triggerStacking', Trigger),
								   transitions={'succeeded':'temp',
												'aborted':'temp',
												'preempted':'temp'})
			smach.StateMachine.add('IDLE152',
								   ServiceState('/triggerStacking', Trigger),
								   transitions={'succeeded':'temp',
												'aborted':'temp',
												'preempted':'temp'})
			smach.StateMachine.add('IDLE153',
								   ServiceState('/triggerStacking', Trigger),
								   transitions={'succeeded':'temp',
												'aborted':'temp',
												'preempted':'temp'})

if __name__ == '__main__':

	while not rospy.is_shutdown():
		rospy.init_node('grasp_sm', log_level=rospy.INFO)
		#clickedPointSub = rospy.Subscriber('/clicked_point', PointStamped, clickedPointCB)

		SM = GraspMachine()
		sis = smach_ros.IntrospectionServer('grasp_server', SM, '/GRASP_SM')
		sis.start()
		SM.execute()
		sis.stop()

