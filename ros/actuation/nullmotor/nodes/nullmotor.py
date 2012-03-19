#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('nullmotor')
import rospy
import tf
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateResponse, SrvFrameStateRequest
from patterngen.srv import SrvSignal, SrvSignalResponse



class NullMotor:

    def __init__(self):
        self.initialized = False
        rospy.loginfo("Opening NullMotor device...")
        rospy.loginfo ("NullMotor name=%s", __name__)
        rospy.init_node('NullMotor')
        rospy.on_shutdown(self.OnShutdown_callback)

        
        # Publish & Subscribe
        rospy.Service('set_stage_state',    SrvFrameState, self.SetStageState_callback)
        rospy.Service('get_stage_state',    SrvFrameState, self.GetStageState_callback)
        rospy.Service('home_stage',         SrvFrameState, self.HomeStage_callback)
        rospy.Service('calibrate_stage',    SrvFrameState, self.Calibrate_callback)
        
        rospy.Service('signal_input',       SrvSignal,     self.SignalInput_callback) # For receiving input from a signal generator.


        self.subEndEffectorOffset = rospy.Subscriber('EndEffectorOffset', Point, self.EndEffectorOffset_callback)
        self.pubJointState        = rospy.Publisher('joint_states', JointState)
        self.pubEndEffector       = rospy.Publisher('EndEffector', MsgFrameState)

        self.tfbx = tf.TransformBroadcaster()
        
        self.rvStageState = SrvFrameStateResponse()
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
        
        
        self.request = SrvFrameStateRequest()
        #self.request.state.header.stamp = rospy.Time.now()
        self.request.state.header.frame_id = '/Stage'
        self.request.state.pose.position.x = 0.0
        self.request.state.pose.position.y = 0.0
        self.request.state.pose.position.z = 0.0
        self.initialized = True


    def EndEffectorOffset_callback(self, ptOffset):
        pass
    
        
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
        
    def SendTransforms(self):  
        if self.initialized:
            self.tfbx.sendTransform((-133.35, 246.31423, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link0",     # child
                                    "Stage"      # parent
                                    )

            self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link1",     # child
                                    "link0"      # parent
                                    )
            self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link3",     # child
                                    "link1"      # parent
                                    )
    
            self.tfbx.sendTransform((266.70, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link5",     # child
                                    "link3"      # parent
                                    )

            self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link2",     # child
                                    "link0"      # parent
                                    )
            self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "link4",     # child
                                    "link2"      # parent
                                    )
            # Frame EndEffector
            self.tfbx.sendTransform((0.0, 0.0, 0.0), 
                                    tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "EndEffector",     # child
                                    "Stage"      # parent
                                    )
                # Frame Target
#                if self.ptsToolRefStage is not None:
#                    markerTarget = Marker(header=Header(stamp = rospy.Time.now(),
#                                                        frame_id='Stage'),
#                                          ns='target',
#                                          id=0,
#                                          type=2, #SPHERE,
#                                          action=0,
#                                          pose=Pose(position=Point(x=self.ptsToolRefStage.point.x, 
#                                                                   y=self.ptsToolRefStage.point.y, 
#                                                                   z=self.ptsToolRefStage.point.z)),
#                                          scale=Vector3(x=3.0,
#                                                        y=3.0,
#                                                        z=3.0),
#                                          color=ColorRGBA(a=1.0,
#                                                          r=0.5,
#                                                          g=0.5,
#                                                          b=0.5),
#                                          lifetime=rospy.Duration(0.1))
#                    self.pubMarker.publish(markerTarget)

#                    markerTool   = Marker(header=Header(stamp = rospy.Time.now(),
#                                                        frame_id='Stage'),
#                                          ns='tooloffset',
#                                          id=1,
#                                          type=0, #ARROW,
#                                          action=0,
#                                          scale=Vector3(x=1.0, # Shaft diameter
#                                                        y=2.0, # Head diameter
#                                                        z=0.0),
#                                          color=ColorRGBA(a=0.9,
#                                                          r=1.0,
#                                                          g=1.0,
#                                                          b=1.0),
#                                          lifetime=rospy.Duration(0.1),
#                                          points=[Point(x=state.pose.position.x, 
#                                                        y=state.pose.position.y, 
#                                                        z=state.pose.position.z),
#                                                  Point(x=state.pose.position.x+self.ptOffsetSense.x, 
#                                                        y=state.pose.position.y+self.ptOffsetSense.y, 
#                                                        z=state.pose.position.z+self.ptOffsetSense.z)])
#                    self.pubMarker.publish(markerTool)




        
    def MainLoop(self):
        try:
            rosrate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.PublishEndEffector()
                self.PublishJointState()
                self.SendTransforms()
                rosrate.sleep()
                
        except KeyboardInterrupt:
            print "Shutting down"
    
    

if __name__ == '__main__':
    try:
        nullmotor = NullMotor()
    except rospy.ROSInterruptException: 
        pass
    
    nullmotor.MainLoop()
    
    