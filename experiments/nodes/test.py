#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import smach
import smach_ros

from std_msgs.msg import Empty

class setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])
    def execute(self, userdata):
        rospy.sleep(3.5)
        return 'setup_done'

class calc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calc_succeeded', 'preempted'])
    def execute(self, userdata):
        for idx in range(5):
            if self.preempt_requested():
                print "state calc is being preempted!!!"
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)
        return 'calc_succeeded'


# Gets called when ANY child state terminates.
def child_term_cb(outcome_map):
    if outcome_map['CALC'] == 'calc_succeeded':
        return True
    elif outcome_map['RESET'] == 'invalid':
        return True
    else:
        return False


# Gets called when ALL child states terminate.
def out_cb(outcome_map):
    if outcome_map['RESET'] == 'invalid':
        return 'concurrence_reset'
    elif outcome_map['CALC'] == 'calc_succeeded':
        return 'concurrence_done'
    else:
        return 'concurrence_reset'


def reset_cb(ud, msg):
    return False     


def main():
    rospy.init_node("preemption_example")

    foo_concurrence = smach.Concurrence(outcomes=['concurrence_done', 'concurrence_reset'],
                                        default_outcome='concurrence_done',
                                        child_termination_cb=child_term_cb,
                                        outcome_cb=out_cb)

    with foo_concurrence:
        smach.Concurrence.add('CALC', calc())
        smach.Concurrence.add('RESET', smach_ros.MonitorState("/sm_reset", Empty, reset_cb))

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('SETUP',       setup(),         transitions={'setup_done':'CONCURRENCE'})
        smach.StateMachine.add('CONCURRENCE', foo_concurrence, transitions={'concurrence_done':'BAR', 'concurrence_reset':'SETUP'}) 
        smach.StateMachine.add('BAR',         calc(),          transitions={'calc_succeeded':'CONCURRENCE', 'preempted':'SETUP'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
