#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
from stage_action_server.msg import *
from flystage.msg import *
import time
import Trial
import MonitorSystemState


# Global Variables
IN_BOUNDS_SUB = MonitorSystemState.InBoundsSubscriber()

# define state WaitForFlyToBeInBounds
class WaitForFlyToBeInBounds(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_FLY_TO_BE_IN_BOUNDS')

        while not IN_BOUNDS_SUB.initialized:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        while not IN_BOUNDS_SUB.in_bounds.fly_in_bounds:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        return 'succeeded'

# define state WaitForFlyToBeOutOfBounds
class WaitForFlyToBeOutOfBounds(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_FLY_TO_BE_OUT_OF_BOUNDS')

        while not IN_BOUNDS_SUB.initialized:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        while IN_BOUNDS_SUB.in_bounds.fly_in_bounds:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        return 'succeeded'

# define state CheckCalibration
class CheckCalibration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibration_ok','needs_recalibration'])

    def execute(self, userdata):
        rospy.logwarn('Executing state CHECK_CALIBRATION')
        if self.preempt_requested():
            return 'preempted'

        # Wait for system to settle
        time.sleep(1)

        return 'calibration_ok'

# define state Recalibrate
class Recalibrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state RECALIBRATE')
        # Wait for system to settle
        if self.preempt_requested():
            return 'preempted'
        time.sleep(1)

        return 'succeeded'

class Experiment():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_experiment = smach.StateMachine(['succeeded','aborted','preempted'])
        #self.sm_experiment.userdata.state = MsgFrameState()
        self.start_position_x = 0 #rospy.get_param("start_position_x")
        self.start_position_y = 0 #rospy.get_param("start_position_y")

        # Open the container
        with self.sm_experiment:
            self.goalStart = stage_action_server.msg.ActionStageStateGoal()
            self.goalStart.state.header.frame_id = 'Plate'
            self.goalStart.state.pose.position.x = self.start_position_x
            self.goalStart.state.pose.position.y = self.start_position_y

            self.trial = Trial.Trial()
            self.sm_trial = self.trial.sm_trial

            # Add states to the container
            smach.StateMachine.add('RECALIBRATE', Recalibrate(),
                                   transitions={'succeeded':'GOTO_START',
                                                'aborted':'aborted',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('GOTO_START',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               ActionStageStateAction,
                                                               goal=self.goalStart),
                                   transitions={'succeeded':'WAIT_FOR_FLY_TO_BE_OUT_OF_BOUNDS',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('WAIT_FOR_FLY_TO_BE_OUT_OF_BOUNDS', 
                                   WaitForFlyToBeOutOfBounds(),
                                   transitions={'succeeded':'CHECK_CALIBRATION',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('CHECK_CALIBRATION', 
                                   CheckCalibration(),
                                   transitions={'calibration_ok':'WAIT_FOR_FLY_TO_BE_IN_BOUNDS',
                                                'needs_recalibration':'RECALIBRATE'})

            smach.StateMachine.add('WAIT_FOR_FLY_TO_BE_IN_BOUNDS', 
                                   WaitForFlyToBeInBounds(),
                                   transitions={'succeeded':'RUN_TRIAL',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('RUN_TRIAL',
                                   self.sm_trial,
                                   transitions={'succeeded':'GOTO_START',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_experiment',
                                                 self.sm_experiment,
                                                 '/SM_EXPERIMENT')

    def execute(self):
        self.sis.start()
        # Execute SMACH plan
        self.outcome = self.sm_experiment.execute()


if __name__ == '__main__':
    rospy.init_node('ExperimentWait')
    try:
        e = Experiment()
        e.execute()
    finally:
        e.sis.stop()
