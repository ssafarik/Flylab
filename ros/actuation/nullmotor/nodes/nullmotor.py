#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('nullmotor')
import rospy
from sensor_msgs.msg import JointState
from flycore.msg import *
from flycore.srv import *
from PatternGen.srv import SrvSignal, SrvSignalResponse



class NullMotor:

    def __init__(self):
        rospy.loginfo("Opening NullMotor device...")
        rospy.loginfo ("NullMotor name=%s", __name__)
        rospy.init_node('NullMotor')
        rospy.on_shutdown(self.OnShutdown_callback)

        
        # Publish & Subscribe
        rospy.Service('set_stage_state',    SrvStageState, self.SetStageState_callback)
        rospy.Service('set_stage_velocity', SrvStageState, self.SetStageVelocity_callback)
        rospy.Service('get_stage_state',    SrvStageState, self.GetStageState_callback)
        rospy.Service('home_stage',         SrvStageState, self.HomeStage_callback)
        rospy.Service('calibrate_stage',    SrvStageState, self.Calibrate_callback)
        rospy.Service('signal_input',       SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.

        self.pubJointState        = rospy.Publisher('joint_states', JointState)
        self.pubEndEffector       = rospy.Publisher('EndEffector', MsgFrameState)
        
        self.rvStageState = SrvStageStateResponse()
        self.rvStageState.state.header.stamp = rospy.Time.now()
        self.rvStageState.state.header.frame_id = '/Stage'
        self.rvStageState.state.pose.position.x = 0.0 
        self.rvStageState.state.pose.position.y = 0.0 
        self.rvStageState.state.pose.position.z = 0.0
        self.rvStageState.state.velocity.linear.x = 0.0
        self.rvStageState.state.velocity.linear.y = 0.0
        self.rvStageState.state.velocity.linear.z = 0.0
        self.rvStageState.state.velocity.angular.x = 0.0
        self.rvStageState.state.velocity.angular.y = 0.0
        self.rvStageState.state.velocity.angular.z = 0.0

        self.js = JointState()
        self.js.header.seq = 0
        self.js.header.stamp.secs = rospy.get_time()
        self.js.header.frame_id = "/Stage"
        self.js.name = ['joint1']
        self.js.velocity = [0.0]
        
        
        self.request = SrvStageStateRequest()
        #self.request.state.header.stamp = rospy.Time.now()
        self.request.state.header.frame_id = '/Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0


    def SignalInput_callback (self, srvSignal):
        rv = SrvSignalResponse()
        rv.success = True
        return rv


    def GetStageState_callback(self, reqStageState):
        return self.rvStageState
    
    
    def GetStageState (self):
        self.rvStageState.state.header.stamp = rospy.Time.now()
        
        return self.rvStageState.state

        
    def SetStageState_callback(self, reqStageState):
        self.rvStageState.state = reqStageState.state

        return self.rvStageState
    

    def SetStageVelocity_callback(self, reqStageState):
        self.rvStageState.state.velocity = reqStageState.state.velocity

        return self.rvStageState


    def HomeStage_callback(self, reqStageState):
        return self.rvStageState
        
        
    def Calibrate_callback(self, req):
        return self.rvStageState
    

    def OnShutdown_callback(self):
        rospy.loginfo("NM Closed NullMotor device.")


    def PublishEndEffector(self):
        state = self.rvStageState.state
        state.header.stamp = rospy.Time.now()
        self.pubEndEffector.publish (state)
        
    def PublishJointState(self):
        self.js.header.seq = self.js.header.seq + 1
        self.js.header.stamp.secs = rospy.get_time()
        self.js.position = [0]
        self.pubJointState.publish(self.js)
        

        
    def MainLoop(self):
        try:
            rosrate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.PublishEndEffector()
                self.PublishJointState()
                rosrate.sleep()
                
        except KeyboardInterrupt:
            print "Shutting down"
    
    

if __name__ == '__main__':
    try:
        nullmotor = NullMotor()
    except rospy.ROSInterruptException: 
        pass
    
    nullmotor.MainLoop()
    
    