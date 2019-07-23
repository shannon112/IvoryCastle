#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String

# define state VoiceCommand
class VoiceCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['what_happened','others','find_material','delivery_product','all_task_clear'])
        self.intent = 0

    def callback(self, data):
        if data.data == "Others": self.intent = 0
        elif data.data == "IntentFind": self.intent = 1
        elif data.data == "IntentDelivery": self.intent = 2
        elif data.data == "IntentWhat": self.intent = 3
        elif data.data == "Clear": self.intent = 4

    def execute(self, userdata):
        rospy.loginfo('Executing state VoiceCommand')
        self.subscriber = rospy.Subscriber('/Intent', String, self.callback)

        if self.intent == 0:
            rospy.sleep(2.)
            return 'others'
        elif self.intent == 1:
            rospy.sleep(2.)
            return 'find_material'
        elif self.intent == 2:
            rospy.sleep(2.)
            return 'delivery_product'
        elif self.intent == 3:
            rospy.sleep(2.)
            return 'what_happened'
        elif self.intent == 4:
            rospy.sleep(2.)
            return 'all_task_clear'
        else:
            rospy.sleep(2.)
            return 'others'


# define state ImageCaption
class ImageCaption(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['telling_done'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ImageCaption')
        if self.state == 0:
            rospy.sleep(2.)
            return 'telling_done'


# define state GraspingObject
class GraspingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasping_done','someone_is_hit'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspingObject')
        if self.state == 0:
            rospy.sleep(2.)
            return 'grasping_done'
        elif self.state == 1:
            rospy.sleep(2.)
            return 'someone_is_hit'

# define state PlacingBasket
class PlacingBasket(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['placing_done','someone_is_hit'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PlacingBasket')
        if self.state == 0:
            rospy.sleep(2.)
            return 'placing_done'
        elif self.state == 1:
            rospy.sleep(2.)
            return 'someone_is_hit'

# define state FetchingBox
class FetchingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fetching_done','someone_is_hit'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FetchingBox')
        if self.state == 0:
            rospy.sleep(2.)
            return 'fetching_done'
        elif self.state == 1:
            rospy.sleep(2.)
            return 'someone_is_hit'

# define state StackingBox
class StackingBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stacking_done','someone_is_hit'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state StackingBox')
        if self.state == 0:
            rospy.sleep(2.)
            return 'stacking_done'
        elif self.state == 1:
            rospy.sleep(2.)
            return 'someone_is_hit'

# define state PauseNavi
class PauseNavi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume_navi'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PauseNavi')
        if self.state == 0:
            rospy.sleep(2.)
            return 'resume_navi'


# define state PauseMoveit
class PauseMoveit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume_1','resume_2'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PauseMoveit')
        if self.state == 0:
            rospy.sleep(2.)
            return 'resume_grasping'
        elif self.state == 1:
            rospy.sleep(2.)
            return 'resume_fetching'


# define state NaviFind
class NaviFind(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start','reachGoal','restart','home','someone_is_here'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NaviFind')
        if self.state == 0:
            self.state += 1
            rospy.sleep(2.)
            return 'start'
        elif self.state == 1:
            self.state += 1
            rospy.sleep(2.)
            return 'reachGoal'
        elif self.state == 2:
            self.state += 1
            rospy.sleep(2.)
            return 'restart'
        elif self.state == 3:
            self.state = 0
            rospy.sleep(2.)
            return 'home'
        elif self.state == 4:
            rospy.sleep(2.)
            return 'someone_is_here'


# define state NaviDelivery
class NaviDelivery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start','reachGoal','restart','home','someone_is_here'])
        self.state = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state NaviDelivery')
        if self.state == 0:
            self.state += 1
            rospy.sleep(2.)
            return 'start'
        elif self.state == 1:
            rospy.sleep(2.)
            self.state += 1
            return 'reachGoal'
        elif self.state == 2:
            self.state += 1
            rospy.sleep(2.)
            return 'restart'
        elif self.state == 3:
            self.state = 0
            rospy.sleep(2.)
            return 'home'
        elif self.state == 4:
            rospy.sleep(2.)
            return 'someone_is_here'


def main():
    rospy.init_node('smach_example_state_machine')


    # ************************************************
    # *********** SM_ROOT ****************************
    # ************************************************
    sm_top = smach.StateMachine(outcomes=['demo_done'])
    sis_root = smach_ros.IntrospectionServer('server_pmc', sm_top, '/SM_ROOT')
    sis_root.start()
    with sm_top:

        smach.StateMachine.add('VoiceCommand', VoiceCommand(),
                               transitions={'others':'VoiceCommand',
                                            'find_material':'NAVI_F',
                                            'delivery_product':'NAVI_D',
                                            'what_happened':'ImageCaption',
                                            'all_task_clear':'demo_done'})

        smach.StateMachine.add('ImageCaption', ImageCaption(), transitions={'telling_done':'VoiceCommand'})
        #smach.StateMachine.add('PauseMoveit', PauseMoveit(), transitions={'resume_moveit':'GraspingObject'})

        # ************************************************
        # *********** SM_ROOT/NAVI_F *********************
        # ************************************************
        sm_naviF = smach.StateMachine(outcomes=['finding_done'])
        sis_naviF = smach_ros.IntrospectionServer('server_pmc', sm_naviF, '/SM_ROOT/NAVI_F')
        sis_naviF.start()
        with sm_naviF:
            smach.StateMachine.add('NaviFind', NaviFind(),
                                   transitions={'start':'NaviFind',
                                                'reachGoal':'GraspingObject',
                                                'restart':'NaviFind',
                                                'home':'PlacingBasket',
                                                'someone_is_here':'PauseNavi'})
            smach.StateMachine.add('GraspingObject', GraspingObject(), transitions={'grasping_done':'NaviFind','someone_is_hit':'PauseMoveit'})
            smach.StateMachine.add('PlacingBasket', PlacingBasket(), transitions={'placing_done':'NaviFind','someone_is_hit':'PauseMoveit'})
            smach.StateMachine.add('PauseNavi', PauseNavi(), transitions={'resume_navi':'NaviFind'})
            smach.StateMachine.add('PauseMoveit', PauseMoveit(), transitions={'resume_1':'GraspingObject','resume_2':'PlacingBasket'})
        smach.StateMachine.add('NAVI_F', sm_naviF, transitions={'finding_done':'VoiceCommand'})

        # ************************************************
        # *********** SM_ROOT/NAVI_D *********************
        # ************************************************
        sm_naviD = smach.StateMachine(outcomes=['delivering_done'])
        sis_naviD = smach_ros.IntrospectionServer('server_pmc', sm_naviD, '/SM_ROOT/NAVI_D')
        sis_naviD.start()
        with sm_naviD:
            smach.StateMachine.add('NaviDelivery', NaviDelivery(),
                                   transitions={'start':'FetchingBox',
                                                'reachGoal':'StackingBox',
                                                'restart':'NaviDelivery',
                                                'home':'NaviDelivery',
                                                'someone_is_here':'PauseNavi'})
            smach.StateMachine.add('FetchingBox', FetchingBox(), transitions={'fetching_done':'NaviDelivery','someone_is_hit':'PauseMoveit'})
            smach.StateMachine.add('StackingBox', StackingBox(), transitions={'stacking_done':'NaviDelivery','someone_is_hit':'PauseMoveit'})
            smach.StateMachine.add('PauseNavi', PauseNavi(), transitions={'resume_navi':'NaviDelivery'})
            smach.StateMachine.add('PauseMoveit', PauseMoveit(), transitions={'resume_1':'FetchingBox','resume_2':'StackingBox'})
        smach.StateMachine.add('NAVI_D', sm_naviD, transitions={'delivering_done':'VoiceCommand'})

    # Execute SMACH plan
    outcome = sm_top.execute()
    sis_root.stop()
    sis_NaviD.stop()
    sis_NaviF.stop()

if __name__ == '__main__':
    main()
