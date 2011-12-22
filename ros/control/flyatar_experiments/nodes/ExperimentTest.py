#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time

# define state Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        time.sleep(5)
        return 'succeeded'

class Experiment():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_experiment = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_experiment:
            stage_goal = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal.x_position = [0]
            stage_goal.y_position = [0]
            stage_goal.speed = [50]

            stage_goal2 = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal2.x_position = [25]
            stage_goal2.y_position = [25]
            stage_goal2.speed = [50]

            # Add states to the container
            smach.StateMachine.add('GOTO_START',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal),
                                   transitions={'succeeded':'WAIT'})

            smach.StateMachine.add('WAIT', Wait(),
                                   transitions={'succeeded':'GOTO_NEWPOSITION'})

            smach.StateMachine.add('GOTO_NEWPOSITION',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal2),
                                   transitions={'succeeded':'succeeded'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_experiment', self.sm_experiment, '/SM_EXPERIMENT')
        self.sis.start()

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_experiment.execute()


if __name__ == '__main__':
    rospy.init_node('ExperimentTest')
    e = Experiment()
    e.execute()
    e.sis.stop()
