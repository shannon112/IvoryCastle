#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from pmc_navigation.srv import navigoal

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
        smach.State.__init__(self, outcomes=['grasping_done','retry','aborted'])
        rospy.wait_for_service('/triggerGrasping')
        self.triggerGrasping_service = rospy.ServiceProxy('/triggerGrasping', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspingObject')
        result = self.triggerGrasping_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'grasping_done'
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        else:
            return 'aborted'


class PlacingBasket(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['placing_done','retry','aborted'])
        rospy.wait_for_service('/triggerPlacing')
        self.triggerPlacing_service = rospy.ServiceProxy('/triggerPlacing', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PlacingBasket')
        result = self.triggerPlacing_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'placing_done'
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        else:
            return 'aborted'


# define state FetchingBox
class FetchingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fetching_done','retry','aborted'])
        rospy.wait_for_service('/triggerFetching')
        self.triggerFetching_service = rospy.ServiceProxy('/triggerFetching', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FetchingBox')
        result = self.triggerFetching_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'fetching_done'
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        else:
            return 'aborted'


# define state StackingBox
class StackingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stacking_done','retry','aborted'])
        rospy.wait_for_service('/triggerStacking')
        self.triggerStacking_service = rospy.ServiceProxy('/triggerStacking', Trigger)
        self.error_counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state StackingBox')
        result = self.triggerStacking_service(TriggerRequest())
        rospy.loginfo(result.message)

        if result.success:
            return 'stacking_done'
        elif error_counter<3:
            error_counter+=1
            return 'retry'
        else:
            return 'aborted'


# define state NavigationC
class NavigationC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['prepare_to_go','reach_goal','back_to_home','done'])
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
        smach.State.__init__(self, outcomes=['prepare_to_go','reach_goal','back_to_home','done'])
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

def main():
    rospy.init_node('smach_example_state_machine')


    # ************************************************
    # *********** SM_ROOT ****************************
    # ************************************************
    sm_top = smach.StateMachine(outcomes=['demo_done'])
    sis_root = smach_ros.IntrospectionServer('server_pmc', sm_top, '/SM_ROOT')
    sm_naviD = smach.StateMachine(outcomes=['delivering_done'])
    sis_naviD = smach_ros.IntrospectionServer('server_pmc', sm_naviD, '/SM_ROOT/DELIVERING')
    sm_naviF = smach.StateMachine(outcomes=['collecting_done'])
    sis_naviF = smach_ros.IntrospectionServer('server_pmc', sm_naviF, '/SM_ROOT/COLLECTING')
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
                                             'aborted':'VoiceCommand',
                                            })


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
                                    transitions={'grasping_done':'NavigationC',
                                                 'retry':'GraspingObject',
                                                 'aborted':'NavigationC',
                                                })
            smach.StateMachine.add('PlacingBasket', PlacingBasket(),
                                    transitions={'placing_done':'NavigationC',
                                                 'retry':'PlacingBasket',
                                                 'aborted':'NavigationC',
                                                })
        smach.StateMachine.add('COLLECTING', sm_naviF, transitions={'collecting_done':'VoiceCommand'})


        # ************************************************
        # *********** SM_ROOT/DELIVERING *********************
        # ************************************************
        with sm_naviD:
            smach.StateMachine.add('NavigationD', NavigationD(),
                                   transitions={'prepare_to_go':'FetchingBox',
                                                'reach_goal':'StackingBox',
                                                'back_to_home':'NavigationD',
                                                'done':'delivering_done'})
            smach.StateMachine.add('FetchingBox', FetchingBox(),
                                    transitions={'fetching_done':'NavigationD',
                                                 'retry':'FetchingBox',
                                                 'aborted':'NavigationD',
                                                })
            smach.StateMachine.add('StackingBox', StackingBox(),
                                    transitions={'stacking_done':'NavigationD',
                                                 'retry':'StackingBox',
                                                 'aborted':'NavigationD',
                                                })
        smach.StateMachine.add('DELIVERING', sm_naviD, transitions={'delivering_done':'VoiceCommand'})

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
