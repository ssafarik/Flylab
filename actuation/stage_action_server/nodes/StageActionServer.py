#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage_action_server')
import rospy
import numpy as N
import threading
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf

from stage_action_server.msg import *
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateRequest
import arena_tf.srv

    

class StageActionServer(object):
    def __init__(self):
        self.initialized = False
        self.isRunning = False
        self.isAtGoal = False
        
        rospy.init_node('StageActionServer') #, anonymous=True)
        self.rosRate = rospy.Rate(100)
        self._action_name = rospy.get_name()
        rospy.loginfo ('%s' % self._action_name)
        self._tfrx = tf.TransformListener() 
        self._as = actionlib.SimpleActionServer(self._action_name, ActionStageStateAction, execute_cb=self.Goal_callback, auto_start=False)
        self._as.start()
        
        # create messages that are used to publish feedback/result.
        self.goal_stage     = ActionStageStateGoal()
        self.goal_arena     = ActionStageStateGoal()
        self.result_stage   = ActionStageStateResult()
        self.result_arena   = ActionStageStateResult()
        self.feedback_stage = ActionStageStateFeedback()
        self.feedback_arena = ActionStageStateFeedback()
        
        self.requestStageState = SrvFrameStateRequest()
            

        try:
            rospy.wait_for_service('get_stage_state')
            self.get_stage_state = rospy.ServiceProxy('get_stage_state', SrvFrameState)
            
            rospy.wait_for_service('set_stage_state')
            self.set_stage_state = rospy.ServiceProxy('set_stage_state', SrvFrameState)

            rospy.wait_for_service('home_stage')
            self.home_stage = rospy.ServiceProxy('home_stage', SrvFrameState)
        except (rospy.exceptions.ROSInterruptException, rospy.ServiceException), e:
            print "Service call failed: %s" % e
        
        
        self.tolerance = 1.0             # mm
        self.initialized = True
    
    
    
    def Goal_callback(self, goal):
        while not self.initialized:
            rospy.sleep(0.5)
        
        #goal.state.pose.position.x = -20
        #goal.state.pose.position.y = -20
        
        # We can eliminate this section, as we are just transforming Arena to Arena.
        poseStamped = PoseStamped(header=goal.state.header, pose=goal.state.pose) 
        #try:
        self._tfrx.waitForTransform("Arena", poseStamped.header.frame_id, poseStamped.header.stamp, rospy.Duration(5.0))
        poseStage = self._tfrx.transformPose('Arena', poseStamped)
        #except:
        #    rospy.sleep(0.1)
        #    poseStage = self._tfrx.transformPose('Arena', poseStamped)

        rospy.loginfo ('SAS goalpose  pretransform %s' % poseStamped)
        rospy.loginfo ('SAS goalpose posttransform %s' % poseStage)
        

        # start executing the action
        self.isAtGoal = False
        while (not self.isAtGoal):
            # Get the current state
            try:
                responseStage = self.get_stage_state(self.requestStageState)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                responseStage = None
                
            if responseStage is not None:             
                # Check if preempt has been requested.
                if self._as.is_preempt_requested():
                    rospy.loginfo('Goal preempted')
                    self._as.set_aborted(result=responseStage)
                    break
        
                # See if we're at the goal
                dx = (responseStage.state.pose.position.x - poseStage.pose.position.x)
                dy = (responseStage.state.pose.position.y - poseStage.pose.position.y)
                dz = (responseStage.state.pose.position.z - poseStage.pose.position.z)
                d = N.linalg.norm([dx,dy,dz]) 
                if d <= self.tolerance:
                    self.isAtGoal = True
                else:
                    self.isAtGoal = False
                
                #rospy.loginfo ('x1x2=%s,%s,\ny1,y2=%s,%s\n' % (responseStage.state.pose.position.x,poseStage.pose.position.x,
                #                                               responseStage.state.pose.position.y,poseStage.pose.position.y))
                rospy.loginfo ('frame=%s,  actual x=%s, y=%s\n' % (responseStage.state.header.frame_id, responseStage.state.pose.position.x, responseStage.state.pose.position.y))
                rospy.loginfo ('frame=%s, desired x=%s, y=%s\n' % (responseStage.state.header.frame_id, poseStage.pose.position.x, poseStage.pose.position.y))
                #rospy.loginfo ('dx=%s, dy=%s, dz=%s, norm=%s' % (dx,dy,dz, d))
            
        
                if self.isAtGoal:
                    self._as.set_succeeded(result=responseStage)
                    self.isRunning = False
                else:
                    if not self.isRunning:
                        self.set_stage_state(SrvFrameStateRequest(state=MsgFrameState(header=poseStage.header, pose=poseStage.pose)))
                        self.isRunning = True
                    else:
                        # We can eliminate this section, as we are just transforming Arena to Arena.
                        poseStamped = PoseStamped(header=Header(frame_id='Arena'), pose=responseStage.state.pose)
                        poseArena = self._tfrx.transformPose('Arena', poseStamped)
                            
                        stateArena = ActionStageStateResult(state=MsgFrameState(header=poseArena.header, pose=poseArena.pose))
                        self._as.publish_feedback(stateArena)
            
            
            self.rosRate.sleep()
                                     
        
    def MainLoop(self):
        rospy.spin()


if __name__ == '__main__':
    node = StageActionServer()
    node.MainLoop()
      

