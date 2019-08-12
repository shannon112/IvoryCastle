#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerRequest
from pmc_navigation.srv import navigoal
from pmc_msgs.srv import detection_PMC_half, GraspPoseEst_direct, PickPlace, PoseSrv
from pmc_msgs.srv import detection_PMC_halfRequest, GraspPoseEst_directRequest, PickPlaceRequest, PoseSrvRequest

# define state VoiceCommand
class VoiceCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['what_happened','others','find_material','delivery_product','all_task_clear','aborted'])
        rospy.wait_for_service('/triggerVCommand')
        self.triggerVCommand_service = rospy.ServiceProxy('/triggerVCommand', Trigger)

    def execute(self, userdata):
        rospy.loginfo('Executing state VoiceCommand')
        result = self.triggerVCommand_service(TriggerRequest())

        if result.success:
            if result.message == "Others":
                rospy.sleep(2.)
                return 'others'
            elif result.message == "IntentFind":
                return 'find_material'
            elif result.message == "IntentDelivery":
                return 'delivery_product'
            elif result.message == "IntentWhat":
                return 'what_happened'
            elif result.message == "Clear":
                return 'all_task_clear'
            else:
                return 'others'
        else:
            return 'aborted'


# define state ImageCaption
class ImageCaption(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['telling_done','retry','aborted'])
        rospy.wait_for_service('/triggerCaption')
        self.triggerCaption_service = rospy.ServiceProxy('/triggerCaption', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ImageCaption')
        result = self.triggerCaption_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'telling_done'
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        else:
            return 'aborted'


# define state GraspingObject
class GraspingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['grasping_start'],
                             output_keys=['task']
                            )

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspingObject')
        userdata.task = 'grasp'
        return 'grasping_start'


class PlacingBasket(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['placing_start'],
                             output_keys=['task']
                            )

    def execute(self, userdata):
        rospy.loginfo('Executing state PlacingBasket')
        userdata.task = 'place'
        return 'placing_start'


# define state FetchingBox
class FetchingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fetching_start'],
                             output_keys=['task']
                            )

    def execute(self, userdata):
        rospy.loginfo('Executing state FetchingBox')
        userdata.task = 'fetch'
        return 'fetching_start'


# define state StackingBox
class StackingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['stacking_start'],
                             output_keys=['task']
                            )

    def execute(self, userdata):
        rospy.loginfo('Executing state StackingBox')
        userdata.task = 'stack'
        return 'stacking_start'


# define state NavigationC
class NavigationC(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['prepare_to_go','reach_goal','back_to_home','done'])
        rospy.wait_for_service('/triggerNavigating')
        self.triggerNavigating_service = rospy.ServiceProxy('/triggerNavigating', navigoal)
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NavigationC')
        if self.state == 0:
            self.state += 1
            return 'prepare_to_go'
        if self.state == 1:
            result = self.triggerNavigating_service(1)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'reach_goal'
        elif self.state == 2:
            result = self.triggerNavigating_service(2)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'back_to_home'
        elif self.state == 3:
            self.state = 0
            return 'done'


# define state NavigationD
class NavigationD(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['prepare_to_go','reach_goal','back_to_home','done'])
        rospy.wait_for_service('/triggerNavigating')
        self.triggerNavigating_service = rospy.ServiceProxy('/triggerNavigating', navigoal)
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NavigationD')
        if self.state == 0:
            self.state += 1
            return 'prepare_to_go'
        elif self.state == 1:
            result = self.triggerNavigating_service(3)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'reach_goal'
        elif self.state == 2:
            result = self.triggerNavigating_service(4)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'back_to_home'
        elif self.state == 3:
            self.state = 0
            return 'done'

# define state SetDefault
class SetDefault(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['attack','pickplace','aborted'],
                             input_keys=['mani_task'],
                             output_keys=['attackPose','pickPose','placePose','objectNum'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state SetDefault')
        rospy.loginfo('Current task is %s', userdata.mani_task)
        
        ### TODO: Set these default parameters ###
        if userdata.mani_task == "grasp":
            userdata.objectNum=3
            ps = [Pose(), Pose(), Pose(), Pose()]
            # take picture at A station
            ps[0].position.x = 1.0; ps[0].position.y = 1.0; ps[0].position.z = 1.0
            ps[0].orientation.x = 1.0; ps[0].orientation.y = 1.0; ps[0].orientation.z = 1.0; ps[0].orientation.w = 1.0
            userdata.attackPose = ps[0]
            # place on amir
            ps[1].position.x = 2.0; ps[1].position.y = 2.0; ps[1].position.z = 2.0
            ps[1].orientation.x = 2.0; ps[1].orientation.y = 2.0; ps[1].orientation.z = 2.0; ps[1].orientation.w = 2.0
            ps[2].position.x = 3.0; ps[2].position.y = 3.0; ps[2].position.z = 3.0
            ps[2].orientation.x = 3.0; ps[2].orientation.y = 3.0; ps[2].orientation.z = 3.0; ps[2].orientation.w = 3.0
            ps[3].position.x = 4.0; ps[3].position.y = 4.0; ps[3].position.z = 4.0
            ps[3].orientation.x = 4.0; ps[3].orientation.y = 4.0; ps[3].orientation.z = 4.0; ps[3].orientation.w = 4.0
            userdata.placePose = ps[1:]
            return 'attack'
        
        elif userdata.mani_task == "place":
            userdata.objectNum=0
            ps = [Pose(), Pose()]
            # grasp from amir
            ps[0].position.x = 5.0; ps[0].position.y = 5.0; ps[0].position.z = 5.0
            ps[0].orientation.x = 5.0; ps[0].orientation.y = 5.0; ps[0].orientation.z = 5.0; ps[0].orientation.w = 5.0
            userdata.pickPose = [ps[0]]
            # place on B station
            ps[1].position.x = 6.0; ps[1].position.y = 6.0; ps[1].position.z = 6.0
            ps[1].orientation.x = 6.0; ps[1].orientation.y = 6.0; ps[1].orientation.z = 6.0; ps[1].orientation.w = 6.0
            userdata.placePose = [ps[1]]
            return 'pickplace'
        
        elif userdata.mani_task == "fetch":
            userdata.objectNum=1
            ps = [Pose(), Pose()]
            # take picture at B station
            ps[0].position.x = 7.0; ps[0].position.y = 7.0; ps[0].position.z = 7.0
            ps[0].orientation.x = 7.0; ps[0].orientation.y = 7.0; ps[0].orientation.z = 7.0; ps[0].orientation.w = 7.0
            userdata.attackPose = ps[0]
            # place on amir
            ps[1].position.x = 8.0; ps[1].position.y = 8.0; ps[1].position.z = 8.0
            ps[1].orientation.x = 8.0; ps[1].orientation.y = 8.0; ps[1].orientation.z = 8.0; ps[1].orientation.w = 8.0
            userdata.placePose = [ps[1]]
            return 'attack'
        
        elif userdata.mani_task == "stack":
            userdata.objectNum=1
            ps = [Pose(), Pose()]
            # take picture at C station
            ps[0].position.x = 9.0; ps[0].position.y = 9.0; ps[0].position.z = 9.0
            ps[0].orientation.x = 9.0; ps[0].orientation.y = 9.0; ps[0].orientation.z = 9.0; ps[0].orientation.w = 9.0
            userdata.attackPose = ps[0]
            # grasp from amir
            ps[1].position.x = 8.0; ps[1].position.y = 8.0; ps[1].position.z = 8.0
            ps[1].orientation.x = 8.0; ps[1].orientation.y = 8.0; ps[1].orientation.z = 8.0; ps[1].orientation.w = 8.0
            userdata.pickPose = [ps[1]]
            return 'attack'
        else:
            return 'aborted'

# define state Attacking
class Attacking(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','aborted'],
                             input_keys=['attackPose'])
        rospy.wait_for_service('/attacking')
        self.AttackingSrv = rospy.ServiceProxy('/attacking', PoseSrv)

    def execute(self, userdata):
        rospy.loginfo('Executing state Attacking')
        result = self.AttackingSrv(userdata.attackPose)
        if result.result:
            return 'success'
        return 'aborted'        

#define state objects Detection
class Detection(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             output_keys=['BBoxs'])
        rospy.wait_for_service('/object_detection_willie')
        self.DetectionSrv = rospy.ServiceProxy('/object_detection_willie', detection_PMC_half)

    def execute(self, userdata):
        rospy.loginfo('Executing state objects Detection')
        result = self.DetectionSrv(detection_PMC_halfRequest())
        #rospy.loginfo(result.data)
        userdata.BBoxs = result.data
        return 'success'

#define state pose Estimation
class Estimation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','aborted'],
                             input_keys=['mani_task','BBoxs','execNum'],
                             output_keys=['PoseEst'])
        rospy.wait_for_service('/grasping_pose_estimation')
        self.DetectionSrv = rospy.ServiceProxy('/grasping_pose_estimation', GraspPoseEst_direct)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state pose Estimation')
        BBox = []
        if userdata.mani_task == "grasp":
            BBox = userdata.BBoxs[userdata.execNum*4:userdata.execNum*4+4]
        elif userdata.mani_task == "fetch" or userdata.mani_task == "stack":
            BBox = userdata.BBoxs[12:16]
        else:
            return 'aborted'
        req = GraspPoseEst_directRequest()
        req.bbox_corner1.x = BBox[0]
        req.bbox_corner1.y = BBox[1]
        req.bbox_corner2.x = BBox[2]
        req.bbox_corner2.y = BBox[3]
        result = self.DetectionSrv(req)
        userdata.PoseEst = result.grasp_pose
        return 'success'

#define state PickPlace
class PicknPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','aborted'],
                             input_keys=['mani_task','pickPose','placePose','estPose','execNum'])
        rospy.wait_for_service('/pick_and_place')
        self.PickPlaceSrv = rospy.ServiceProxy('/pick_and_place', PickPlace)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state pose PickPlace')
        req = PickPlaceRequest()
        if userdata.mani_task == "place":
            rospy.loginfo(userdata.pickPose)
            rospy.loginfo(userdata.placePose)
            req.pick_pose = userdata.pickPose[userdata.execNum]
            req.place_pose = userdata.placePose[userdata.execNum]
        elif userdata.mani_task == "stack":
            req.pick_pose = userdata.pickPose[userdata.execNum]
            req.place_pose = userdata.estPose
        else:
            req.pick_pose = userdata.estPose
            req.place_pose = userdata.placePose[userdata.execNum]
        result = self.PickPlaceSrv(req)
        if not result.result:
            return 'aborted'
        return 'success'

class TaskEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['graspdone','placedone','fetchdone','stackdone','continue','aborted'],
                             input_keys=['mani_task','objectNum','execNumIn'],
                             output_keys=['execNumOut'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state TaskEnd')
        if userdata.objectNum > userdata.execNumIn+1:
            userdata.execNumOut = userdata.execNumIn+1
            return 'continue'
        else:
            userdata.execNumOut = 0
            if userdata.mani_task == "grasp":
                return 'graspdone'
            if userdata.mani_task == "place":
                return 'placedone'
            if userdata.mani_task == "fetch":
                return 'fetchdone'
            if userdata.mani_task == "stack":
                return 'stackdone'
        return 'aborted'

def main():
    rospy.init_node('smach_example_state_machine')


    # ************************************************
    # *********** SM_ROOT ****************************
    # ************************************************
    sm_top = smach.StateMachine(outcomes=['demo_done'])
    sis_root = smach_ros.IntrospectionServer('server_pmc', sm_top, '/SM_ROOT')
    sm_naviD = smach.StateMachine(outcomes=['delivering_done', 'manipulating'],
                                  output_keys=['mani_task'])
    sis_naviD = smach_ros.IntrospectionServer('server_pmc', sm_naviD, '/SM_ROOT/DELIVERING')
    sm_naviF = smach.StateMachine(outcomes=['collecting_done', 'manipulating'],
                                  output_keys=['mani_task'])
    sis_naviF = smach_ros.IntrospectionServer('server_pmc', sm_naviF, '/SM_ROOT/COLLECTING')
    sm_arm = smach.StateMachine(outcomes=['grasping_done', 'placing_done', 'fetching_done', 'stacking_done', 'task_aborted'],
                                input_keys=['mani_task'])
    sis_arm = smach_ros.IntrospectionServer('server_pmc', sm_arm, '/SM_ROOT/MANIPULATION')
    
    with sm_top:
        smach.StateMachine.add('VoiceCommand', VoiceCommand(),
                               transitions={'others':'VoiceCommand',
                                            'find_material':'COLLECTING',
                                            'delivery_product':'DELIVERING',
                                            'what_happened':'ImageCaption',
                                            'all_task_clear':'demo_done',
                                            'aborted':'VoiceCommand'})

        smach.StateMachine.add('ImageCaption', ImageCaption(),
                                transitions={'telling_done':'VoiceCommand',
                                             'retry':'ImageCaption',
                                             'aborted':'VoiceCommand'})


        # ************************************************
        # *********** SM_ROOT/COLLECTING *********************
        # ************************************************
        with sm_naviF:
            smach.StateMachine.add('NavigationC', NavigationC(),
                                   transitions={'prepare_to_go':'NavigationC',
                                                'reach_goal':'GraspingObject',
                                                'back_to_home':'PlacingBasket',
                                                'done':'collecting_done'})
            smach.StateMachine.add('GraspingObject', GraspingObject(),
                                   transitions={'grasping_start':'manipulating'},
                                   remapping={'task':'mani_task'})
            smach.StateMachine.add('PlacingBasket', PlacingBasket(),
                                   transitions={'placing_start':'manipulating'},
                                   remapping={'task':'mani_task'})
        smach.StateMachine.add('COLLECTING', sm_naviF,
                               transitions={'collecting_done':'VoiceCommand',
                                            'manipulating':'MANIPULATION'})


        # ************************************************
        # *********** SM_ROOT/DELIVERING *********************
        # ************************************************
        with sm_naviD:
            smach.StateMachine.add('NavigationD', NavigationD(),
                                   transitions={'prepare_to_go':'FetchingBox',
                                                'reach_goal':'StackingBox',
                                                'back_to_home':'NavigationD',
                                                'done':'delivering_done'
                                               })
            smach.StateMachine.add('FetchingBox', FetchingBox(),
                                   transitions={'fetching_start':'manipulating'},
                                   remapping={'task':'mani_task'})
            smach.StateMachine.add('StackingBox', StackingBox(),
                                   transitions={'stacking_start':'manipulating'},
                                   remapping={'task':'mani_task'})
        smach.StateMachine.add('DELIVERING', sm_naviD,
                               transitions={'delivering_done':'VoiceCommand',
                                            'manipulating':'MANIPULATION'})
        
        
        # ************************************************
        # *********** SM_ROOT/MANIPULATION ***************
        # ************************************************
        sm_arm.userdata.sm_arm_atk_pose = Pose()
        sm_arm.userdata.sm_arm_pck_pose = []
        sm_arm.userdata.sm_arm_plc_pose = []
        sm_arm.userdata.sm_arm_est_pose = Pose()
        sm_arm.userdata.sm_arm_object_count = 0
        sm_arm.userdata.sm_arm_exec_count = 0
        sm_arm.userdata.sm_arm_bboxs = []
        
        with sm_arm:
            smach.StateMachine.add('SetDefaultParams', SetDefault(),
                                   transitions={'attack':'PoseAttacking',
                                                'pickplace':'PickAndPlace',
                                                'aborted':'task_aborted'},
                                   remapping={'attackPose':'sm_arm_atk_pose',
                                              'pickPose':'sm_arm_pck_pose',
                                              'placePose':'sm_arm_plc_pose',
                                              'objectNum':'sm_arm_object_count'})            
            smach.StateMachine.add('PoseAttacking', Attacking(),
                                   transitions={'success':'ObjectsDetection',
                                                'aborted':'task_aborted'},
                                   remapping={'attackPose':'sm_arm_atk_pose'})
            smach.StateMachine.add('ObjectsDetection', Detection(),
                                   transitions={'success':'PoseEstimation'},
                                   remapping={'BBoxs':'sm_arm_bboxs'})
            smach.StateMachine.add('PoseEstimation', Estimation(),
                                   transitions={'success':'PickAndPlace',
                                                'aborted':'task_aborted'},
                                   remapping={'BBoxs':'sm_arm_bboxs',
                                              'execNum':'sm_arm_exec_count',
                                              'PoseEst':'sm_arm_est_pose'})
            smach.StateMachine.add('PickAndPlace', PicknPlace(),
                                   transitions={'success':'TaskEnd',
                                                'aborted':'task_aborted'},
                                   remapping={'pickPose':'sm_arm_pck_pose',
                                              'placePose':'sm_arm_plc_pose',
                                              'estPose':'sm_arm_est_pose',
                                              'execNum':'sm_arm_exec_count'})
            smach.StateMachine.add('TaskEnd', TaskEnd(),
                                   transitions={'graspdone':'grasping_done',
                                                'placedone':'placing_done',
                                                'fetchdone':'fetching_done',
                                                'stackdone':'stacking_done',
                                                'continue':'SetDefaultParams',
                                                'aborted':'task_aborted'},
                                   remapping={'objectNum':'sm_arm_object_count',
                                              'execNumIn':'sm_arm_exec_count',
                                              'execNumOut':'sm_arm_exec_count'})
            
        smach.StateMachine.add('MANIPULATION', sm_arm,
                               transitions={'grasping_done':'COLLECTING',
                                            'placing_done':'COLLECTING',
                                            'fetching_done':'DELIVERING',
                                            'stacking_done':'DELIVERING',
                                            'task_aborted':'VoiceCommand'})

    # Execute SMACH plan
    sis_root.start()
    sis_naviF.start()
    sis_naviD.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis_root.stop()
    sis_naviD.stop()
    sis_naviF.stop()

if __name__ == '__main__':
    main()
