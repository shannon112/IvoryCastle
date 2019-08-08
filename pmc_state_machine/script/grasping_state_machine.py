#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose 
from neuronbot_msgs.srv import Alignment, ObjDetectionWithName, GraspPoseEst_direct, PickPlace
from pmc_msgs.srv import PoseSrv
from smach_ros import ServiceState

objName = 'mouse'
#objName = 'banana'

class GraspMachine(smach.StateMachine):
  def __init__(self):
    global objName
    smach.StateMachine.__init__(self, outcomes = ['temp'])
    self.userdata.objName = objName 
    self.userdata.pc = None
    self.userdata.bboxCorner1 = None
    self.userdata.bboxCorner2 = None
    self.userdata.attackPose = None
    self.userdata.graspPose = None
    self.userdata.placePose = None
    
    ### Fixed positions ###
	# take picture at A station
    attack_ps = Pose()
    attack_ps.position.x = -2.0; attack_ps.position.y = -2.0; attack_ps.position.z = -2.5 #TODO
    self.userdata.attackPose = attack_ps
    # place on amir
    place_ps = Pose()
    place_ps.position.x = -1.0; place_ps.position.y = -1.0; place_ps.position.z = -10.0 #TODO
    self.userdata.placePose = place_ps
    #############################

    with self:
      smach.StateMachine.add('IDLE',
                ServiceState('/triggerGrasping', Trigger),
                transitions={'succeeded':'ATTACK_POSE',
                             'aborted':'temp',
                             'preempted':'temp'})

      smach.StateMachine.add('ATTACK_POSE',
                ServiceState('attacking', PoseSrv,
                              request_slots = ['pose']),
                remapping = {'pose':'attackPose'},
                transitions={'succeeded':'OBJECT_DETECTION',
                             'aborted':'temp',
                             'preempted':'temp'})
      
      def objDetectionCB(userdata, result):
        #self.userdata.pc = result.input_pc
        self.userdata.bboxCorner1 = result.bbox_corner1
        self.userdata.bboxCorner2 = result.bbox_corner2
    
      smach.StateMachine.add('OBJECT_DETECTION',
                ServiceState('obj_detection_with_name', ObjDetectionWithName,
                              output_keys = ['pc', 'bboxCorner1', 'bboxCorner2'],
                              request_slots = ['obj_name'], 
                              response_cb = objDetectionCB),
                remapping = {'obj_name':'objName'},
                transitions={'succeeded':'GRASPING_POSE_ESTIMATION',
                             'aborted':'temp',
                             'preempted':'temp'})

      def GraspPoseEstCB(userdata, result):
        self.userdata.graspPose = result.grasp_pose

      smach.StateMachine.add('GRASPING_POSE_ESTIMATION',
                ServiceState('grasping_pose_estimation', GraspPoseEst_direct, 
                              output_keys = ['graspPose'],
                              request_slots = ['bbox_corner1', 'bbox_corner2'],
                              response_cb = GraspPoseEstCB),
                remapping = {'bbox_corner1':'bboxCorner1',
                             'bbox_corner2':'bboxCorner2',
                             'input_pc':'pc'},
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

    SM = GraspMachine()
    sis = smach_ros.IntrospectionServer('grasp_server', SM, '/GRASP_SM')
    sis.start()
    SM.execute()
    sis.stop()

