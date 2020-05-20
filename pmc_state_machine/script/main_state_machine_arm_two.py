#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerRequest
from pmc_navigation.srv import navigoal
from pmc_msgs.srv import detection_PMC_half, GraspPoseEst_direct, PickPlace, PoseSrv
from pmc_msgs.srv import detection_PMC_halfRequest, GraspPoseEst_directRequest, PickPlaceRequest, PoseSrvRequest
from haf_grasping.srv import BBoxCenter, BBoxCenterRequest

# define state VoiceCommand
class VoiceCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['others','find_material','delivery_product','all_task_clear','ending_charge','aborted'])
        rospy.wait_for_service('/triggerVCommand')
        self.triggerVCommand_service = rospy.ServiceProxy('/triggerVCommand', Trigger)

    def execute(self, userdata):
        rospy.loginfo('Executing state VoiceCommand')
        result = self.triggerVCommand_service(TriggerRequest())

        if result.success:
            if result.message == 'Others':
                rospy.sleep(2.)
                return 'others'
            elif result.message == 'IntentFind':
                return 'find_material'
            elif result.message == 'IntentDelivery':
                return 'delivery_product'
            elif result.message == 'Clear':
                return 'all_task_clear'
            elif result.message == 'IntentEnd':
                return 'ending_charge'
            else:
                return 'others'
        else:
            return 'aborted'

# define state AutoCharge
class AutoCharge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charge_done','retry','aborted'])
        rospy.wait_for_service('/docking')
        self.triggerAutoCharge_service = rospy.ServiceProxy('/docking', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state AutoCharge')
        result = self.triggerAutoCharge_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'charge_done'
        else:
            return 'aborted'
        """
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        """

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
        userdata.task = 'skip'
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
        userdata.task = 'place'
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
        userdata.task = 'skip'
        return 'stacking_start'


# define state NavigationC
class NavigationC(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['prepare_to_go','reach_goal','done'])
        rospy.wait_for_service('/triggerNavigating')
        self.triggerNavigating_service = rospy.ServiceProxy('/triggerNavigating', navigoal)
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NavigationC %d', self.state)
        if self.state == 0:
            self.state += 1
            return 'prepare_to_go'
        if self.state == 1:
            result = self.triggerNavigating_service(1)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'reach_goal'
            self.state += 1
            return 'reach_goal'
        elif self.state == 2:
            self.state = 0
            return 'done'


# define state NavigationD
class NavigationD(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['prepare_to_go','reach_goal','done'])
        rospy.wait_for_service('/triggerNavigating')
        self.triggerNavigating_service = rospy.ServiceProxy('/triggerNavigating', navigoal)
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NavigationD %d', self.state)
        if self.state == 0:
            result = self.triggerNavigating_service(5)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'prepare_to_go'
        elif self.state == 1:
            result = self.triggerNavigating_service(4)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'reach_goal'
        elif self.state == 2:
            self.state = 0
            return 'done'


# define state NavigationE
class NavigationE(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['prepare_to_go','reach_goal','done'])
        rospy.wait_for_service('/triggerNavigating')
        self.triggerNavigating_service = rospy.ServiceProxy('/triggerNavigating', navigoal)
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NavigationE %d', self.state)
        if self.state == 0:
            result = self.triggerNavigating_service(10)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'prepare_to_go'
        elif self.state == 1:
            result = self.triggerNavigating_service(9)
            rospy.loginfo(result.message)
            if result.success:
                self.state += 1
                return 'reach_goal'
        elif self.state == 2:
            result = self.triggerNavigating_service(3)
            rospy.loginfo(result.message)
            self.state = 0
            return 'done'


# define state SetDefault
class SetDefault(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['attack','pickplace','aborted'],
                             input_keys=['mani_task'],
                             output_keys=['initPose','attackPose','pickPose','placePose','objectNum'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SetDefault')
        rospy.loginfo('Current task is %s', userdata.mani_task)

        ### TODO: Set these default parameters ###
        psInit = Pose()
        psInit.position.x = -0.603; psInit.position.y = 0.097; psInit.position.z = 1.322
        psInit.orientation.x = -0.891; psInit.orientation.y = -0.030; psInit.orientation.z = 0.453; psInit.orientation.w = 0.023
        userdata.initPose = psInit
        if userdata.mani_task == 'grasp':
            userdata.objectNum=2
            ps = [Pose(), Pose(), Pose(), Pose(), Pose()]
            # take picture at A station
            ps[0].position.x = 0.116; ps[0].position.y = -0.530; ps[0].position.z = 1.196
            ps[0].orientation.x = 0.705; ps[0].orientation.y = 0.709; ps[0].orientation.z = -0.004; ps[0].orientation.w = 0.012
            userdata.attackPose = ps[0]

            # place on amir (camera)
            ps[1].position.x = -0.252; ps[1].position.y = 0.053; ps[1].position.z = 0.530
            ps[1].orientation.x = 0.905; ps[1].orientation.y = -0.425; ps[1].orientation.z = -0.035; ps[1].orientation.w = 0.003
            # place on amir (tripod)
            ps[2].position.x = -0.252; ps[2].position.y = 0.053; ps[2].position.z = 0.530
            ps[1].orientation.x = 0.905; ps[1].orientation.y = -0.425; ps[1].orientation.z = -0.035; ps[1].orientation.w = 0.003
            # place on amir (USB)
            ps[3].position.x = -0.252; ps[3].position.y = 0.053; ps[3].position.z = 0.530
            ps[1].orientation.x = 0.905; ps[1].orientation.y = -0.425; ps[1].orientation.z = -0.035; ps[1].orientation.w = 0.003
            # place on amir (box) z=0.505
            ps[4].position.x = -0.288; ps[4].position.y = -0.189; ps[4].position.z = 0.455
            ps[4].orientation.x = 1.000; ps[4].orientation.y = 0.002; ps[4].orientation.z = -0.015; ps[4].orientation.w = 0.004
            userdata.placePose = ps[1:]
            return 'attack'

        elif userdata.mani_task == 'place':
            userdata.objectNum=0
            ps = [Pose(), Pose()]
            # grasp from amir (box)
            ps[0].position.x = -0.288; ps[0].position.y = -0.189; ps[0].position.z = 0.475
            ps[0].orientation.x = 1.000; ps[0].orientation.y = 0.002; ps[0].orientation.z = -0.015; ps[0].orientation.w = 0.004
            userdata.pickPose = [ps[0]]
            # place on B station
            ps[1].position.x = 0.245; ps[1].position.y = -0.706; ps[1].position.z = 0.77
            ps[1].orientation.x = 0.707; ps[1].orientation.y = 0.707; ps[1].orientation.z = -0.027; ps[1].orientation.w = 0.027
            userdata.placePose = [ps[1]]
            return 'pickplace'

        else:
            return 'aborted'

# define state Attacking
class Attacking(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','aborted'],
                             input_keys=['mani_task','initPose','attackPose','countIn'],
                             output_keys=['countOut'])
        rospy.wait_for_service('/attacking_pose')
        self.AttackingSrv = rospy.ServiceProxy('/attacking_pose', PoseSrv)
        self.count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Attacking')
        req = PoseSrvRequest()
        #req.pose = userdata.initPose
        #req.str_box_ind = 'i'
        #result = self.AttackingSrv(req)
        #if not result.result:
        #    return 'aborted'
        req.pose.position.x = userdata.attackPose.position.x
        req.pose.position.y = userdata.attackPose.position.y
        req.pose.position.z = userdata.attackPose.position.z
        req.pose.orientation = userdata.attackPose.orientation
        if userdata.mani_task == 'grasp':
            req.str_box_ind = 'a'
        elif userdata.mani_task == 'place':
            req.str_box_ind = 'b'
        else:
            return 'aborted'
        if userdata.countIn % 10 == 2 or userdata.countIn % 10 == 3:
            req.pose.position.x = req.pose.position.x + 0.04
        elif userdata.countIn % 10 == 4 or userdata.countIn % 10 == 5:
            req.pose.position.y = req.pose.position.y + 0.04
        elif userdata.countIn % 10 == 6 or userdata.countIn % 10 == 7:
            req.pose.position.x = req.pose.position.x - 0.04
        elif userdata.countIn % 10 == 8 or userdata.countIn % 10 == 9:
            req.pose.position.y = req.pose.position.y - 0.04
        result = self.AttackingSrv(req)
        #rospy.sleep(3)
        userdata.countOut = userdata.countIn + 1
        if result.result:
            return 'success'
        return 'aborted'

#define state objects Detection
class Detection(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'retry'],
                             output_keys=['BBoxs'])
        rospy.wait_for_service('/object_detection_willie')
        self.DetectionSrv = rospy.ServiceProxy('/object_detection_willie', detection_PMC_half)

    def execute(self, userdata):
        rospy.loginfo('Executing state objects Detection')
        result = self.DetectionSrv(detection_PMC_halfRequest())
        if sum(result.data) == 0:
            return 'retry'
        userdata.BBoxs = result.data
        return 'success'

#define state pose Estimation
class Estimation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','retry','aborted'],
                             input_keys=['mani_task','BBoxs','execNum'],
                             output_keys=['PoseEst','ObjectID'])
        rospy.wait_for_service('/grasping_pose_estimation')
        self.DetectionSrv = rospy.ServiceProxy('/grasping_pose_estimation', GraspPoseEst_direct)
        #rospy.wait_for_service('/PC_center')
        #self.PCCenterSrv = rospy.ServiceProxy('/PC_center', BBoxCenter)

    def execute(self, userdata):
        rospy.loginfo('Executing state pose Estimation')
        BBox = []
        id = 0
        if userdata.mani_task == 'grasp':
            for i in range(4):
                BBox = userdata.BBoxs[i*4:(i+1)*4]
                if BBox != (0., 0., 0., 0.):
                    id = i
                    break
        else:
            return 'aborted'
        userdata.ObjectID = id
        rospy.loginfo(BBox)
        req = GraspPoseEst_directRequest()
        req.bbox_corner1.x = BBox[0]
        req.bbox_corner1.y = BBox[1]
        req.bbox_corner2.x = BBox[2]
        req.bbox_corner2.y = BBox[3]
        result = self.DetectionSrv(req)
        if result.grasp_pose.position.x == 0.0 and  result.grasp_pose.position.y == 0.0 and result.grasp_pose.position.z == 0.0:
            return 'retry'
        OffsetPose = Pose()
        OffsetPose.position.x = result.grasp_pose.position.x
        OffsetPose.position.y = result.grasp_pose.position.y
        OffsetPose.position.z = result.grasp_pose.position.z
        if id == 3:
            OffsetPose.position.z = result.grasp_pose.position.z
        OffsetPose.orientation = result.grasp_pose.orientation
        userdata.PoseEst = OffsetPose
        """
        if userdata.mani_task == 'fetch' or userdata.mani_task == 'stack':
            resultPS = Pose()
            resultPS.orientation = result.grasp_pose.orientation
            req = BBoxCenterRequest()
            req.bboxx = (BBox[0]+BBox[2])/2
            req.bboxy = (BBox[1]+BBox[3])/2
            result = self.PCCenterSrv(req)
            print (result)
            resultPS.position.x = result.centerx
            resultPS.position.y = result.centery
            resultPS.position.z = result.centerz
            userdata.PoseEst = resultPS
        """
        return 'success'

#define state PickPlace
class PicknPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success','restart','aborted'],
                             input_keys=['mani_task','initPose','pickPose','placePose','estPose','execNum','ObjectID','BBoxs'])
        #rospy.wait_for_service('/attacking_pose')
        #self.AttackingSrv = rospy.ServiceProxy('/attacking_pose', PoseSrv)
        rospy.wait_for_service('/pick_and_place')
        self.PickPlaceSrv = rospy.ServiceProxy('/pick_and_place', PickPlace)
        self.restart = 0
        self.pick = True

    def execute(self, userdata):
        rospy.loginfo('Executing state pose PickPlace')
        #req = PoseSrvRequest()
        #req.pose = userdata.initPose
        #req.str_box_ind = 'i'
        #result = self.AttackingSrv(req)
        #if not result.result:
        #    return 'aborted'
        req = PickPlaceRequest()
        rospy.loginfo(userdata.ObjectID)
        #rospy.loginfo(userdata.placePose)
        if userdata.mani_task == 'grasp':
            req.str_box_ind = 'a'
            #req.pick_pose = userdata.estPose
            req.pick_pose.position = userdata.estPose.position
            req.pick_pose.position.z = 0.845
            #req.pick_pose.orientation.x = 0.936; req.pick_pose.orientation.y = 0.352; req.pick_pose.orientation.z = 0.002; req.pick_pose.orientation.w = 0.007
            if userdata.ObjectID == 3:
                req.pick_pose.orientation.x = 0.055; req.pick_pose.orientation.y = 0.998; req.pick_pose.orientation.z = -0.031; req.pick_pose.orientation.w = 0.006
            else:
                #req.pick_pose.orientation.x = 0.936; req.pick_pose.orientation.y = 0.352; req.pick_pose.orientation.z = 0.002; req.pick_pose.orientation.w = 0.007
            	req.pick_pose.orientation.x = 0.905; req.pick_pose.orientation.y = -0.425; req.pick_pose.orientation.z = -0.035; req.pick_pose.orientation.w = 0.003
            	#req.pick_pose.orientation = userdata.estPose.orientation
            req.place_pose = userdata.placePose[userdata.ObjectID]
        elif userdata.mani_task == 'place':
            req.str_box_ind = 'b'
            req.pick_pose = userdata.pickPose[userdata.execNum]
            req.place_pose = userdata.placePose[userdata.execNum]
        else:
            return 'aborted'
        if self.pick:
            req.str_box_ind += '1'
        else:
            req.str_box_ind += '2'
        result = self.PickPlaceSrv(req)
        #rospy.logwarn(req.pick_pose)
        #rospy.logwarn(req.place_pose)
        if result.result:
            self.pick = True
            self.restart = 0
            return 'success'
        else:
            self.restart += 1
            if self.restart > 2 and req.str_box_ind == 'a' and userdata.mani_task == 'grasp':
                self.restart = 0
                return 'restart'
            self.pick = True
            if result.state == 'place':
                self.pick = False
        return 'aborted'

class TaskEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['graspdone','placedone','continue','aborted'],
                             input_keys=['initPose','mani_task','objectNum','execNumIn'],
                             output_keys=['execNumOut','countOut'])
        rospy.wait_for_service('/attacking_pose')
        self.AttackingSrv = rospy.ServiceProxy('/attacking_pose', PoseSrv)

    def execute(self, userdata):
        rospy.loginfo('Executing state TaskEnd')
        #self.InitPub.publish(String("trig"))
        userdata.countOut = 0
        req = PoseSrvRequest()
        req.pose = userdata.initPose
        req.str_box_ind = 'i'
        if userdata.objectNum > userdata.execNumIn+1:
            userdata.execNumOut = userdata.execNumIn+1
            return 'continue'
        else:
            status = ''
            userdata.execNumOut = 0
            if userdata.mani_task == 'grasp':
                status = 'graspdone'
            elif userdata.mani_task == 'place':
                status = 'placedone'
            else:
                return 'aborted'
            result = self.AttackingSrv(req)
            if result.result:
                return status
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
    sm_naviE = smach.StateMachine(outcomes=['ending_done'])
    sis_naviE = smach_ros.IntrospectionServer('server_pmc', sm_naviE, '/SM_ROOT/ENDING')
    sm_arm = smach.StateMachine(outcomes=['grasping_done', 'placing_done', 'task_aborted'],
                                input_keys=['mani_task'])
    sis_arm = smach_ros.IntrospectionServer('server_pmc', sm_arm, '/SM_ROOT/MANIPULATION')

    with sm_top:
        smach.StateMachine.add('VoiceCommand', VoiceCommand(),
                               transitions={'others':'VoiceCommand',
                                            'find_material':'COLLECTING',
                                            'delivery_product':'DELIVERING',
                                            'all_task_clear':'demo_done',
                                            'aborted':'VoiceCommand',
                                            'ending_charge':'ENDING'})

        # ************************************************
        # *********** SM_ROOT/COLLECTING *********************
        # ************************************************
        with sm_naviE:
            smach.StateMachine.add('NavigationE', NavigationE(),
                                   transitions={'prepare_to_go':'NavigationE',
                                                'reach_goal':'NavigationE',
                                                'done':'ending_done'})
        smach.StateMachine.add('ENDING', sm_naviE,
                               transitions={'ending_done':'VoiceCommand'})

        with sm_naviF:
            smach.StateMachine.add('NavigationC', NavigationC(),
                                   transitions={'prepare_to_go':'NavigationC',
                                                'reach_goal':'GraspingObject',
                                                'done':'collecting_done'})
            smach.StateMachine.add('GraspingObject', GraspingObject(),
                                   transitions={'grasping_start':'manipulating'},
                                   remapping={'task':'mani_task'})
            #smach.StateMachine.add('PlacingBasket', PlacingBasket(),
            #                       transitions={'placing_start':'manipulating'},
            #                       remapping={'task':'mani_task'})
        smach.StateMachine.add('COLLECTING', sm_naviF,
                               transitions={'collecting_done':'VoiceCommand',
                                            'manipulating':'MANIPULATION'})


        # ************************************************
        # *********** SM_ROOT/DELIVERING *********************
        # ************************************************
        with sm_naviD:
            smach.StateMachine.add('NavigationD', NavigationD(),
                                   transitions={'prepare_to_go':'NavigationD',
                                                'reach_goal':'AutoCharge',
                                                'done':'delivering_done'
                                               })
            smach.StateMachine.add('AutoCharge', AutoCharge(),
                                transitions={'charge_done':'NavigationD',
                                             'retry':'AutoCharge',
                                             'aborted':'NavigationD'})

            #smach.StateMachine.add('StackingBox', StackingBox(),
            #                       transitions={'stacking_start':'manipulating'},
            #                       remapping={'task':'mani_task'})
        smach.StateMachine.add('DELIVERING', sm_naviD,
                               transitions={'delivering_done':'VoiceCommand',
                                            'manipulating':'MANIPULATION'})


        # ************************************************
        # *********** SM_ROOT/MANIPULATION ***************
        # ************************************************
        sm_arm.userdata.sm_arm_ini_pose = Pose()
        sm_arm.userdata.sm_arm_atk_pose = Pose()
        sm_arm.userdata.sm_atk_count = 0
        sm_arm.userdata.sm_arm_pck_pose = []
        sm_arm.userdata.sm_arm_plc_pose = []
        sm_arm.userdata.sm_arm_est_pose = Pose()
        sm_arm.userdata.sm_arm_object_count = 0
        sm_arm.userdata.sm_arm_object_id = 0
        sm_arm.userdata.sm_arm_exec_count = 0
        sm_arm.userdata.sm_arm_bboxs = []

        with sm_arm:
            smach.StateMachine.add('SetDefaultParams', SetDefault(),
                                   transitions={'attack':'PoseAttacking',
                                                'pickplace':'PickAndPlace',
                                                'aborted':'task_aborted'},
                                   remapping={'initPose':'sm_arm_ini_pose',
											  'attackPose':'sm_arm_atk_pose',
                                              'pickPose':'sm_arm_pck_pose',
                                              'placePose':'sm_arm_plc_pose',
                                              'objectNum':'sm_arm_object_count'})
            smach.StateMachine.add('PoseAttacking', Attacking(),
                                   transitions={'success':'ObjectsDetection',
                                                'aborted':'PoseAttacking'},
                                   remapping={'initPose':'sm_arm_ini_pose',
											  'attackPose':'sm_arm_atk_pose',
                                              'countIn':'sm_atk_count',
                                              'countOut':'sm_atk_count'})
            smach.StateMachine.add('ObjectsDetection', Detection(),
                                   transitions={'success':'PoseEstimation',
                                                'retry':'ObjectsDetection'},
                                   remapping={'BBoxs':'sm_arm_bboxs'})
            smach.StateMachine.add('PoseEstimation', Estimation(),
                                   transitions={'success':'PickAndPlace',
												'retry':'PoseAttacking',
                                                'aborted':'task_aborted'},
                                   remapping={'BBoxs':'sm_arm_bboxs',
                                              'execNum':'sm_arm_exec_count',
                                              'PoseEst':'sm_arm_est_pose',
                                              'ObjectID':'sm_arm_object_id'})
            smach.StateMachine.add('PickAndPlace', PicknPlace(),
                                   transitions={'success':'TaskEnd',
                                                'restart':'PoseAttacking',
                                                'aborted':'PickAndPlace'},
                                   remapping={'initPose':'sm_arm_ini_pose',
											  'pickPose':'sm_arm_pck_pose',
                                              'placePose':'sm_arm_plc_pose',
                                              'estPose':'sm_arm_est_pose',
                                              'execNum':'sm_arm_exec_count',
                                              'ObjectID':'sm_arm_object_id',
                                              'BBoxs':'sm_arm_bboxs',})
            smach.StateMachine.add('TaskEnd', TaskEnd(),
                                   transitions={'graspdone':'grasping_done',
                                                'placedone':'placing_done',
                                                'continue':'SetDefaultParams',
                                                'aborted':'TaskEnd'},
                                   remapping={'initPose':'sm_arm_ini_pose',
											  'objectNum':'sm_arm_object_count',
                                              'execNumIn':'sm_arm_exec_count',
                                              'execNumOut':'sm_arm_exec_count',
                                              'countOut':'sm_atk_count'})

        smach.StateMachine.add('MANIPULATION', sm_arm,
                               transitions={'grasping_done':'COLLECTING',
                                            'placing_done':'DELIVERING',
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
