#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose 
from neuronbot_msgs.srv import PickPlace
from smach_ros import ServiceState

class FetchMachine(smach.StateMachine):
  def __init__(self):
    global objName
    smach.StateMachine.__init__(self, outcomes = ['temp'])
    self.userdata.graspPose = None
    self.userdata.placePose = None
    
    ### Fixed positions ###
    # grasp from amir
    grasp_ps = Pose()
    grasp_ps.position.x = -2.0; grasp_ps.position.y = -2.0; grasp_ps.position.z = -2.5 #TODO
    self.userdata.graspPose = grasp_ps
    # place on B station
    place_ps = Pose()
    place_ps.position.x = -1.0; place_ps.position.y = -1.0; place_ps.position.z = -10.0 #TODO
    self.userdata.placePose = place_ps
    #############################

    with self:
      smach.StateMachine.add('IDLE',
                ServiceState('/triggerPlacing', Trigger),
                transitions={'succeeded':'PICK_PLACE',
                             'aborted':'temp',
                             'preempted':'temp'})
	
      smach.StateMachine.add('PICK_PLACE',
                ServiceState('pick_and_place', PickPlace,
                              request_slots = ['pick_pose', 'place_pose']),
                remapping = {'pick_pose':'graspPose',
                             'place_pose':'placePose'},
                transitions={'succeeded':'temp',
                             'aborted':'temp',
                             'preempted':'temp'})

if __name__ == '__main__':

  while not rospy.is_shutdown():
    rospy.init_node('grasp_sm', log_level=rospy.INFO)
    #clickedPointSub = rospy.Subscriber('/clicked_point', PointStamped, clickedPointCB)

    SM = FetchMachine()
    sis = smach_ros.IntrospectionServer('fetch_server', SM, '/FETCH_SM')
    sis.start()
    SM.execute()
    sis.stop()

