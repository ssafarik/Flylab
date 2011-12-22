#! /usr/bin/env python
import roslib; roslib.load_manifest('flyatar_action_client')
import rospy

import actionlib
from std_msgs.msg import Header
from stage_action_server.msg import *


class FlyatarActionClient():
    def __init__(self):
        # Creates the SimpleActionClient, passing the type of the action
        # to the constructor.
        self.client = actionlib.SimpleActionClient('StageActionServer', ActionStageStateAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Create a goal message.
        self.goal = ActionStageStateGoal()
        

    def send_goal(self):
        # Send the goal to the action server.
        rospy.loginfo ('FlyatarActionClient sending goal %s' % self.goal)
        self.client.send_goal(self.goal)
        rospy.loginfo ('FlyatarActionClient wait_for_result %s' % self.goal)
        self.client.wait_for_result()
        rospy.loginfo ('FlyatarActionClient get_result %s' % self.goal)
        self.result = self.client.get_result()
        rospy.loginfo ('FlyatarActionClient result = %s' % self.result)


    def home(self):
        self.goal.state.pose = None
        self.send_goal()


    def square_move(self,trial=0):
        x = [ 25,  25, -25, -25]
        y = [-25,  25,  25, -25]

        self.goal = ActionStageStateGoal()
        self.goal.state.header = Header(frame_id='Plate')
        self.goal.state.pose.position.x = x[trial % 4]
        self.goal.state.pose.position.y = y[trial % 4]
        
#                    { \
#                      'header': {'stamp': rospy.Time.now(), 'frame_id': 'Plate'}, \
#                      'pose': {'x': x[trial % 5], 'y': y[trial % 5] } \
#                      }
        self.send_goal()


    def center_move(self):
        self.goal = ActionStageStateGoal()
        self.goal.state.header = Header(frame_id='Plate')
        self.goal.state.pose.position.x = 0
        self.goal.state.pose.position.y = 0
        self.send_goal()


    def main(self):
        # self.home()
        self.num_trials = 100
        for trial in range(self.num_trials):
            rospy.loginfo('trial=%s' % trial)
            self.square_move(trial)
            #self.center_move()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('FlyatarActionClient')
        fac = FlyatarActionClient()
        fac.main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
