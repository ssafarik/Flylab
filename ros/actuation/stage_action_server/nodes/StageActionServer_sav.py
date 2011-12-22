#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_action_server')
import rospy
import actionlib
import stage_action_server.msg
import stage.srv
import tf
import plate_tf.srv
import numpy
import threading


class StageUpdate:
    def __init__(self):
        self.initialized = False
        
        self.dt = rospy.get_param("Stage_Update_dt","0.010")
        self.rate = rospy.Rate(1/self.dt)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.reentrant_lock = threading.Lock()
        self.response = None
        self.stage_commands = stage.srv.Stage_StateRequest()
        self.bUpdatePosition = False
        self.bHome = False
        
        rospy.wait_for_service('get_stage_state')
        try:
            self.get_stage_state = rospy.ServiceProxy('get_stage_state', stage.srv.Stage_State)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.wait_for_service('set_stage_position')
        try:
            self.set_stage_position = rospy.ServiceProxy('set_stage_position', stage.srv.Stage_State)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.wait_for_service('home_stage')
        try:
            self.home_stage = rospy.ServiceProxy('home_stage', stage.srv.Stage_State)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.wait_for_service('plate_to_stage')
        try:
            self.plate_to_stage = rospy.ServiceProxy('plate_to_stage', plate_tf.srv.PlateStageConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.wait_for_service('stage_to_plate')
        try:
            self.stage_to_plate = rospy.ServiceProxy('stage_to_plate', plate_tf.srv.PlateStageConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        self.initialized = True

    
    def update(self):
        if self.initialized:
            with self.reentrant_lock:
                try:
                    bUpdatePos = self.bUpdatePosition
                    bHome = self.bHome
                    self.bUpdatePosition = False
                    self.bHome = False
                    
                    if bUpdatePos:
                      rospy.loginfo ('SAS update sending goal stage_commands=%s' % (self.stage_commands))
                      self.response = self.set_stage_position(self.stage_commands)
                    elif bHome:
                      self.response = self.home_stage()
                    else:
                      self.response = self.get_stage_state()
                    
                    x = self.response.x
                    y = self.response.y
                    
                    self.tf_broadcaster.sendTransform((x, y, 0),
                                                      tf.transformations.quaternion_from_euler(0, 0, 0),
                                                      rospy.Time.now(),
                                                      "Magnet",
                                                      "Stage")
                
                except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
                    pass
    

class StageActionServer(object):
    def __init__(self, name):
        self.initialized = False
        
        self.stage_update = StageUpdate()
        self._action_name = name
        self.tf = tf.TransformListener() 
        self._as = actionlib.SimpleActionServer(self._action_name, stage_action_server.msg.UpdateStagePositionAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        # create messages that are used to publish feedback/result
        self.goal_stage     = stage_action_server.msg.UpdateStagePositionGoal()
        self.goal_plate     = stage_action_server.msg.UpdateStagePositionGoal()
        self.result_stage   = stage_action_server.msg.UpdateStagePositionResult()
        self.result_plate   = stage_action_server.msg.UpdateStagePositionResult()
        self.feedback_stage = stage_action_server.msg.UpdateStagePositionFeedback()
        self.feedback_plate = stage_action_server.msg.UpdateStagePositionFeedback()
        
        self.goal_threshold = 1.0             # mm
        self.initialized = True
    
    
    def convert_goal_to_stage(self):
        response = self.stage_update.plate_to_stage(self.goal_plate.x_position,self.goal_plate.y_position) # This is a service call.
        self.goal_stage.x_position = response.Xdst
        self.goal_stage.y_position = response.Ydst
        self.goal_stage.speed = self.goal_plate.speed
    
    def convert_result_to_plate(self):
        response = self.stage_update.stage_to_plate([self.result_stage.x],[self.result_stage.y]) # This is a service call.
        self.result_plate.x = response.Xdst[0]
        self.result_plate.y = response.Ydst[0]
    
    def convert_feedback_to_plate(self):
        response = self.stage_update.stage_to_plate([self.feedback_stage.x],[self.feedback_stage.y]) # This is a service call.
        # rospy.logwarn("self.feedback_stage.x = %s" % (str(self.feedback_stage.x)))
        # rospy.logwarn("self.feedback_stage.y = %s" % (str(self.feedback_stage.y)))
        self.feedback_plate.x = response.Xdst[0]
        self.feedback_plate.y = response.Ydst[0]
        # rospy.logwarn("self.feedback_plate.x = %s" % (str(self.feedback_plate.x)))
        # rospy.logwarn("self.feedback_plate.y = %s" % (str(self.feedback_plate.y)))
        vel_norm_stage = numpy.linalg.norm([self.feedback_stage.x_velocity, self.feedback_stage.y_velocity])
        response = self.stage_update.stage_to_plate([self.feedback_stage.x_velocity], [self.feedback_stage.y_velocity]) # This is a service call.
        self.feedback_plate.x_velocity = response.Xdst[0]
        self.feedback_plate.y_velocity = response.Ydst[0]
        vel_norm_plate = numpy.linalg.norm([self.feedback_plate.x_velocity, self.feedback_plate.y_velocity])
        vel_norm_ratio = vel_norm_stage / vel_norm_plate
        self.feedback_plate.x_velocity = self.feedback_plate.x_velocity * vel_norm_ratio
        self.feedback_plate.y_velocity = self.feedback_plate.y_velocity * vel_norm_ratio
        # rospy.logwarn("self.feedback_stage.x_velocity = %s" % (str(self.feedback_stage.x_velocity)))
        # rospy.logwarn("self.feedback_stage.y_velocity = %s" % (str(self.feedback_stage.y_velocity)))
        # rospy.logwarn("self.feedback_plate.x_velocity = %s" % (str(self.feedback_plate.x_velocity)))
        # rospy.logwarn("self.feedback_plate.y_velocity = %s" % (str(self.feedback_plate.y_velocity)))
    
    
    def execute_cb(self, goal):
        if self.initialized:
            self.goal_plate = goal
            self.convert_goal_to_stage()
            position_list_length = min(len(self.goal_stage.x_position),len(self.goal_stage.y_position))
            if (0 < position_list_length):
                self.stage_update.bUpdatePosition = True
                self.x_goal = self.goal_stage.x_position[-1]
                self.y_goal = self.goal_stage.y_position[-1]
            else:
                self.stage_update.bHome = True
                self.x_goal = None
                self.y_goal = None
            
            rospy.loginfo ('SAS execute_cb setting goal x=%s, y=%s' % (self.goal_stage.x_position, self.goal_stage.y_position))
            self.stage_update.stage_commands.x_position = self.goal_stage.x_position
            self.stage_update.stage_commands.y_position = self.goal_stage.y_position
            self.stage_update.stage_commands.speed = self.goal_stage.speed
            
            # helper variables
            self.success = False
            
            # start executing the action
            while (not self.success):
                self.stage_update.update()
                if self.stage_update.response is None:
                    self._as.set_aborted()
                    break
                # rospy.logwarn("stage_update.response.x = %s" % (str(self.stage_update.response.x)))
                # rospy.logwarn("stage_update.response.y = %s" % (str(self.stage_update.response.y)))
                # rospy.logwarn("self.x_goal = %s" % (str(self.x_goal)))
                # rospy.logwarn("self.y_goal = %s" % (str(self.y_goal)))
                
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    break
                if (self.x_goal is not None) and (self.y_goal is not None):
                    if (not self.stage_update.response.bMoveInProgress) and \
                           self.stage_update.response.bAllJointsInPosition and \
                           self.stage_update.response.bJointsAtHome:
                        if (abs(self.stage_update.response.x - self.x_goal) < self.goal_threshold) and \
                               (abs(self.stage_update.response.y - self.y_goal) < self.goal_threshold):
                            self.success = True
                        else:
                            self._as.set_aborted()
                            break
                    else:
                        self.feedback_stage.x = self.stage_update.response.x
                        self.feedback_stage.y = self.stage_update.response.y
                        self.feedback_stage.x_velocity = self.stage_update.response.x_velocity
                        self.feedback_stage.y_velocity = self.stage_update.response.y_velocity
                else:
                    if (not self.stage_update.response.bMoveInProgress) and \
                            self.stage_update.response.bAllJointsInPosition and \
                            self.stage_update.response.bJointsAtHome:
                        self.success = True
                        # if (abs(self.stage_update.response.x - self.x_goal) < self.goal_threshold) and \
                        #        (abs(self.stage_update.response.y - self.y_goal) < self.goal_threshold):
                        #   self.success = True
                        # else:
                        #   self._as.set_aborted()
                        #   break
                
                self.convert_feedback_to_plate()
                self._as.publish_feedback(self.feedback_plate)
                
                self.stage_update.rate.sleep()

            if self.success:
              self.result_stage.x = self.stage_update.response.x
              self.result_stage.y = self.stage_update.response.y
              self.convert_result_to_plate()
              self._as.set_succeeded(self.result_plate)
        else:
            self._as.set_aborted()
        
    def mainloop(self):
        try:
            while not rospy.is_shutdown():
                self.stage_update.update()
                self.stage_update.rate.sleep()
        except KeyboardInterrupt:
            print "Shutting down"


if __name__ == '__main__':
    rospy.init_node('StageActionServer') #, anonymous=True)
    node = StageActionServer(rospy.get_name())
    node.mainloop()
      

