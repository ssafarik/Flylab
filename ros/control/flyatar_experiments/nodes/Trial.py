#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('flyatar_experiments')
import rospy

import smach
import smach_ros
#import stage_action_server.msg
from flystage.msg import *
import time
import RobotMotionProfiles
from save_data.msg import CommandSavedata
import MonitorSystemState
import random
import numpy as N
import copy


class PublisherCommandSavedata:
    def __init__(self):
        self.pub_commandsavedata = rospy.Publisher("CommandSavedata", CommandSavedata)
        self.commandsavedata = CommandSavedata()
        self.trial_count = 1

    def trial_count_increment(self):
        self.trial_count += 1


# Global Variables
SAVE_DATA_CONTROLS_PUB = PublisherCommandSavedata()

# define state ChooseAngularVelocity
class ChooseAngularVelocity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['stateOUT'])

        self.experiment_angular_velocity_max_negative = rospy.get_param("experiment_angular_velocity_max_negative") # rad/s
        self.experiment_angular_velocity_max_positive = rospy.get_param("experiment_angular_velocity_max_positive") # rad/s
        self.experiment_angular_velocity_bin_count = rospy.get_param("experiment_angular_velocity_bin_count")
        self.experiment_angular_velocity_vector_negative_repetition = rospy.get_param("experiment_angular_velocity_vector_negative_repetition","1")
        self.experiment_angular_velocity_vector_positive_repetition = rospy.get_param("experiment_angular_velocity_vector_positive_repetition","1")
        self.experiment_angular_velocity_vector_zero_repetition = rospy.get_param("experiment_angular_velocity_vector_zero_repetition","1")
        self.angular_velocity_list_complete = list(N.linspace(self.experiment_angular_velocity_max_negative,
                                                                  self.experiment_angular_velocity_max_positive,
                                                                  self.experiment_angular_velocity_bin_count,
                                                                  True))
        
        if 1 < self.experiment_angular_velocity_vector_zero_repetition:
            extra_zero_list = list(N.zeros(self.experiment_angular_velocity_vector_zero_repetition - 1))
            self.angular_velocity_list_complete.extend(extra_zero_list)
            
        if 1 < self.experiment_angular_velocity_vector_negative_repetition:
            extra_negative_list = [l for l in self.angular_velocity_list_complete if l < 0]
            extra_negative_list *= int(self.experiment_angular_velocity_vector_negative_repetition - 1)
            self.angular_velocity_list_complete.extend(extra_negative_list)
            
        if 1 < self.experiment_angular_velocity_vector_positive_repetition:
            extra_positive_list = [l for l in self.angular_velocity_list_complete if 0 < l]
            extra_positive_list *= int(self.experiment_angular_velocity_vector_positive_repetition - 1)
            self.angular_velocity_list_complete.extend(extra_positive_list)
            
        random.shuffle(self.angular_velocity_list_complete)
        self.angular_velocity_list = copy.copy(self.angular_velocity_list_complete)


    def execute(self, userdata):
        rospy.logwarn('Executing state CHOOSE_ANGULAR_VELOCITY')
        try:
            angular_velocity = self.angular_velocity_list.pop()
        except (IndexError):
            self.angular_velocity_list = copy.copy(self.angular_velocity_list_complete)
            angular_velocity = self.angular_velocity_list.pop()

        #rospy.loginfo ('asdf1')
        userdata.stateOUT = MsgFrameState()
        userdata.stateOUT.velocity.angular.z = angular_velocity
        #rospy.loginfo ('asdf2')

        return 'succeeded'


class RecordData(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['stateIn'])
        self.protocol = "walking_protocol_type_1"

    def execute(self, userdata):
        rospy.logwarn('Executing state RECORD_DATA')

        rospy.logwarn("RECORD_DATA userdata = %s" % (str(userdata)))

        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.commandsavedata.file_name_base        = time.strftime("%Y_%m_%d_%H_%M_%S")
        SAVE_DATA_CONTROLS_PUB.commandsavedata.protocol              = self.protocol
        SAVE_DATA_CONTROLS_PUB.commandsavedata.trial_number          = int(SAVE_DATA_CONTROLS_PUB.trial_count)
        SAVE_DATA_CONTROLS_PUB.commandsavedata.angular_velocity_goal = userdata.stateIn.velocity.angular.z
        SAVE_DATA_CONTROLS_PUB.commandsavedata.rm_file               = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_arenastate       = True
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_video            = True
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_bag              = True
        SAVE_DATA_CONTROLS_PUB.pub_commandsavedata.publish(SAVE_DATA_CONTROLS_PUB.commandsavedata)
        rospy.logwarn("RecordData succeeded")
        return 'succeeded'

# define state EraseData
class EraseData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.logwarn('Executing state ERASE_DATA')
        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.commandsavedata.rm_file = True
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_arenastate = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_video = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_bag = False
        SAVE_DATA_CONTROLS_PUB.pub_commandsavedata.publish(SAVE_DATA_CONTROLS_PUB.commandsavedata)
        return 'succeeded'

# define state MonitorConditions
class MonitorConditions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fly_left_bounds','preempted'])
        self.in_bounds_sub = MonitorSystemState.InBoundsSubscriber()

    def execute(self, userdata):
        rospy.logwarn('Executing state MONITOR_CONDITIONS')

        while not self.in_bounds_sub.initialized:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        while True:
            if self.preempt_requested():
                rospy.logwarn("MonitorConditions preempted")
                return 'preempted'
            if not self.in_bounds_sub.in_bounds.fly_in_bounds:
                rospy.logwarn("MonitorConditions fly_left_bounds")
                return 'fly_left_bounds'
            time.sleep(0.1)

class LogTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.logwarn('Executing state LOG_TRIAL')
        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.commandsavedata.rm_file = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_arenastate = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_video = False
        SAVE_DATA_CONTROLS_PUB.commandsavedata.save_bag = False
        SAVE_DATA_CONTROLS_PUB.pub_commandsavedata.publish(SAVE_DATA_CONTROLS_PUB.commandsavedata)
        SAVE_DATA_CONTROLS_PUB.trial_count_increment()
        return 'succeeded'

class Trial():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_trial = smach.StateMachine(['succeeded', 'aborted', 'preempted']) #, 
                                           #input_keys = ['stateIn'],
                                           #output_keys = ['stateOUT'])
                                    

        # Open the container
        with self.sm_trial:
            self.sm_trial.userdata.state = MsgFrameState()
            self.robot_motion_profile = RobotMotionProfiles.RobotMotionProfile()
            self.sm_robot_motion_profile = self.robot_motion_profile.sm_robot_motion_profile

            # Create the concurrent sub SMACH state machine...
            self.sm_record_monitor_control = smach.Concurrence(outcomes=['succeeded','fly_left_bounds','aborted','preempted'],
                                                               default_outcome='aborted',
                                                               input_keys=['stateIn'],
                                                               outcome_map={'succeeded':
                                                                             {'RECORD_DATA':'succeeded',
                                                                              'MONITOR_CONDITIONS':'preempted',
                                                                              'CONTROL_ROBOT':'succeeded'
                                                                             },
                                                                            'fly_left_bounds':
                                                                             {'RECORD_DATA':'succeeded',
                                                                             'MONITOR_CONDITIONS':'fly_left_bounds'
                                                                             }
                                                                           },
                                                               child_termination_cb = self.child_termination_callback)
            # ...with states.
            with self.sm_record_monitor_control:
                # Add states to the RMC container
                smach.Concurrence.add('RECORD_DATA', RecordData(),
                                      remapping={'stateIn':'state'})
                smach.Concurrence.add('MONITOR_CONDITIONS', MonitorConditions())
                smach.Concurrence.add('CONTROL_ROBOT', self.sm_robot_motion_profile,
                                      remapping={'stateIn':'state'})


            # Add states to the TRIAL container
            smach.StateMachine.add('CHOOSE_ANGULAR_VELOCITY', ChooseAngularVelocity(),
                                   transitions={'succeeded':'RECORD_MONITOR_CONTROL'},
                                   remapping={'stateOUT':'state'})

            smach.StateMachine.add('RECORD_MONITOR_CONTROL', self.sm_record_monitor_control,
                                   transitions={'succeeded':'LOG_TRIAL',
                                                'fly_left_bounds':'LOG_TRIAL',
                                                # 'skipped_move':'LOG_TRIAL',
                                                'aborted':'ERASE_DATA',
                                                'preempted':'ERASE_DATA'},
                                                # 'aborted':'LOG_TRIAL',
                                                # 'preempted':'LOG_TRIAL'},
                                   remapping={'stateIn':'state'})

            smach.StateMachine.add('ERASE_DATA', EraseData(),
                                   transitions={'succeeded':'succeeded'})

            smach.StateMachine.add('LOG_TRIAL', LogTrial(),
                                   transitions={'succeeded':'succeeded'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_trial',
                                                 self.sm_trial,
                                                 '/SM_EXPERIMENT/SM_TRIAL')
        self.sis.start()

    # gets called when ANY child state terminates
    def child_termination_callback(self,outcome_map):

        # rospy.logwarn("outcome_map = %s" % (str(outcome_map)))

        # terminate all running states if FOO finished with outcome 'outcome3'
        # if outcome_map['FOO'] == 'outcome3':
        #     # just keep running
        #     return False

        # terminate all running states if CONTROL_ROBOT finished
        if outcome_map['CONTROL_ROBOT'] == 'succeeded':
            return True

        # terminate all running states if MONITOR_CONDITIONS finished
        # if outcome_map['MONITOR_CONDITIONS'] == 'aborted':
        #     return True

        # in all other case, just keep running, don't terminate anything
        return False

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_trial.execute()


if __name__ == '__main__':
    rospy.init_node('Trial')
    t = Trial()
    t.execute()
    t.sis.stop()
