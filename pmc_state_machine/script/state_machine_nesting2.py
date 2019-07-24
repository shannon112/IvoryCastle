#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

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


# define state NaviFind
class NaviFind(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start','reachGoal','restart','home','done'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NaviFind')
        if self.state == 0:
            rospy.sleep(2.)
            self.state += 1
            return 'start'
        elif self.state == 1:
            rospy.sleep(2.)
            self.state += 1
            return 'reachGoal'
        elif self.state == 2:
            rospy.sleep(2.)
            self.state += 1
            return 'restart'
        elif self.state == 3:
            rospy.sleep(2.)
            self.state += 1
            return 'home'
        elif self.state == 4:
            self.state = 0
            return 'done'

# define state NaviDelivery
class NaviDelivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start','reachGoal','restart','home','done'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NaviDelivery')
        if self.state == 0:
            rospy.sleep(2.)
            self.state += 1
            return 'start'
        elif self.state == 1:
            rospy.sleep(2.)
            self.state += 1
            return 'reachGoal'
        elif self.state == 2:
            rospy.sleep(2.)
            self.state += 1
            return 'restart'
        elif self.state == 3:
            rospy.sleep(2.)
            self.state += 1
            return 'home'
        elif self.state == 4:
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
    sis_naviD = smach_ros.IntrospectionServer('server_pmc', sm_naviD, '/SM_ROOT/NAVI_D')
    sm_naviF = smach.StateMachine(outcomes=['finding_done'])
    sis_naviF = smach_ros.IntrospectionServer('server_pmc', sm_naviF, '/SM_ROOT/NAVI_F')
    with sm_top:
        smach.StateMachine.add('VoiceCommand', VoiceCommand(),
                               transitions={'others':'VoiceCommand',
                                            'find_material':'NAVI_F',
                                            'delivery_product':'NAVI_D',
                                            'what_happened':'ImageCaption',
                                            'all_task_clear':'demo_done',
                                            'aborted':'VoiceCommand'})

        smach.StateMachine.add('ImageCaption', ImageCaption(),
                                transitions={'telling_done':'VoiceCommand',
                                             'retry':'ImageCaption',
                                             'aborted':'VoiceCommand',
                                            })


        # ************************************************
        # *********** SM_ROOT/NAVI_F *********************
        # ************************************************
        with sm_naviF:
            smach.StateMachine.add('NaviFind', NaviFind(),
                                   transitions={'start':'NaviFind',
                                                'reachGoal':'GraspingObject',
                                                'restart':'NaviFind',
                                                'home':'PlacingBasket',
                                                'done':'finding_done'})
            smach.StateMachine.add('GraspingObject', GraspingObject(),
                                    transitions={'grasping_done':'NaviFind',
                                                 'retry':'GraspingObject',
                                                 'aborted':'NaviFind',
                                                })
            smach.StateMachine.add('PlacingBasket', PlacingBasket(),
                                    transitions={'placing_done':'NaviFind',
                                                 'retry':'PlacingBasket',
                                                 'aborted':'NaviFind',
                                                })
        smach.StateMachine.add('NAVI_F', sm_naviF, transitions={'finding_done':'VoiceCommand'})


        # ************************************************
        # *********** SM_ROOT/NAVI_D *********************
        # ************************************************
        with sm_naviD:
            smach.StateMachine.add('NaviDelivery', NaviDelivery(),
                                   transitions={'start':'FetchingBox',
                                                'reachGoal':'StackingBox',
                                                'restart':'NaviDelivery',
                                                'home':'NaviDelivery',
                                                'done':'delivering_done'})
            smach.StateMachine.add('FetchingBox', FetchingBox(),
                                    transitions={'fetching_done':'NaviDelivery',
                                                 'retry':'FetchingBox',
                                                 'aborted':'NaviDelivery',
                                                })
            smach.StateMachine.add('StackingBox', StackingBox(),
                                    transitions={'stacking_done':'NaviDelivery',
                                                 'retry':'StackingBox',
                                                 'aborted':'NaviDelivery',
                                                })
        smach.StateMachine.add('NAVI_D', sm_naviD, transitions={'delivering_done':'VoiceCommand'})

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
