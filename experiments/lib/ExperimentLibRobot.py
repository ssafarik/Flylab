#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import copy
import numpy as np
import smach
import tf

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, String
from flycore.msg import MsgFrameState, TrackingCommand
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from patterngen.msg import MsgPattern
from tracking.msg import ArenaState


#######################################################################################################
#######################################################################################################
class Reset (smach.State):
    def __init__(self, tfrx=None):
        self.tfrx = tfrx
        smach.State.__init__(self, 
                             outcomes=['success','disabled','preempt','aborted'],
                             input_keys=['experimentparamsChoicesIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        self.pubSetPattern = rospy.Publisher('SetPattern', MsgPattern, latch=True)
        self.pubTrackingCommand = rospy.Publisher('tracking/command', TrackingCommand, latch=True)

        self.SetStageStateRef = None

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
    
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo('EL State ResetRobot(%s)' % [userdata.experimentparamsChoicesIn.home.enabled, userdata.experimentparamsChoicesIn.home.x, userdata.experimentparamsChoicesIn.home.y])

        rv = 'disabled'
        if (userdata.experimentparamsChoicesIn.trial.robot.enabled or userdata.experimentparamsChoicesIn.pre.robot.enabled) and (userdata.experimentparamsChoicesIn.home.enabled):
            
            self.target = MsgFrameState()
            self.timeStart = rospy.Time.now()
    
            if (self.SetStageStateRef is None):
                stSrv = 'set_stage_state_ref'
                try:
                    rospy.wait_for_service(stSrv)
                    self.SetStageStateRef = rospy.ServiceProxy(stSrv, SrvFrameState, persistent=True)
                except rospy.ServiceException, e:
                    rospy.logwarn ('EL FAILED to connect service %s(): %s' % (stSrv, e))
            
            while self.arenastate is None:
                #if userdata.experimentparamsChoicesIn.home.timeout != -1:
                #    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsChoicesIn.home.timeout:
                #        return 'timeout'
                    
                #if self.preempt_requested():
                #    self.service_preempt()
                #    return 'preempt'
                rospy.sleep(1.0)
    
                if (self.commandExperiment=='exit_now'):
                    return 'aborted'
            # Turn off any prior pattern generation.
            msgPattern = MsgPattern()
            msgPattern.shape = 'constant'
            msgPattern.points = []
            msgPattern.frameidPosition = 'Arena'
            msgPattern.frameidAngle = 'Arena'
            msgPattern.hzPattern = 1.0
            msgPattern.hzPoint = 50
            msgPattern.count = 0
            msgPattern.size = Point(0.0, 0.0, 0.0)
            msgPattern.restart = False
            msgPattern.param = 0.0
            msgPattern.direction = 1
            self.pubSetPattern.publish (msgPattern)
            rospy.sleep(1)
    
            
            # Send the 'home' command.
            self.target.header = self.arenastate.robot.header
            #self.target.header.stamp = rospy.Time.now()
            self.target.pose.position.x = userdata.experimentparamsChoicesIn.home.x
            self.target.pose.position.y = userdata.experimentparamsChoicesIn.home.y
            try:
                self.SetStageStateRef(SrvFrameStateRequest(state=MsgFrameState(header=self.target.header, 
                                                                            pose=self.target.pose,
                                                                            speed = userdata.experimentparamsChoicesIn.home.speed)))
            except rospy.ServiceException, e:
                rospy.logwarn ('EL FAILED service call to set_stage_state_ref().')

            rv = 'aborted'
            while not rospy.is_shutdown():
                # Are we there yet?
                
                ptRobot = np.array([self.arenastate.robot.pose.position.x, 
                                   self.arenastate.robot.pose.position.y])
                ptTarget = np.array([self.target.pose.position.x,
                                    self.target.pose.position.y])                
                r = np.linalg.norm(ptRobot-ptTarget)
                
                rospy.logwarn('Distance to home: %f, tolerance: %f' % (r, userdata.experimentparamsChoicesIn.home.tolerance))
                if (r <= userdata.experimentparamsChoicesIn.home.tolerance):
                    rospy.sleep(0.5) # Allow some settling time.
                    rv = 'success'
                    break
                
                
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: ResetRobot()')
                    self.service_preempt()
                    rv = 'preempt'
                    break
                
                
                #if userdata.experimentparamsChoicesIn.home.timeout != -1:
                #    if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsChoicesIn.home.timeout:
                #        rv = 'timeout'
                #        break
                
                self.rosrate.sleep()

                if (self.commandExperiment=='exit_now'):
                    rv = 'aborted'
                    break
                
        return rv
        

# End class Reset()        

        

#######################################################################################################
#######################################################################################################
class Action (smach.State):
    def __init__(self, mode='trial', tfrx=None):
        self.tfrx = tfrx
        smach.State.__init__(self, 
                             outcomes=['success','disabled','preempt','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        self.mode = mode
        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        self.pubSetPattern = rospy.Publisher('SetPattern', MsgPattern, latch=True)

        self.target = MsgFrameState()
        self.SetStageStateRef = None

        self.radiusMovement = float(rospy.get_param('arena/radius_inner','25.4'))
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


#     # GetAngleFrame()
#     # Get the orientation angle of the given frameid in the Arena frame.
#     #
#     def GetAngleFrame (self, frameidChild):
#         try:
#             stamp = self.tfrx.getLatestCommonTime('Arena', frameidChild)
#             (trans,q) = self.tfrx.lookupTransform('Arena', frameidChild, stamp)
#         except tf.Exception, e:
#             #rospy.logwarn('Exception in GetAngleFrame():  %s' % e)
#             angleOfChild = None
#         else:
#             rpy = tf.transformations.euler_from_quaternion(q)
#             angleOfChild = rpy[2] % (2.0 * np.pi)
#     
#             
#         return angleOfChild
    
    
    # GetAngleFrameToFrame()
    # Get the angle of the child's position in the parent's frame.
    #
    def GetAngleFrameToFrame (self, frameidParent, frameidChild):
        try:
            stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = self.tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception, e:
            #rospy.logwarn('Exception in GetAngleFrameToFrame():  %s' % e)
            angleToChild = None
        else:
            angleToChild = np.arctan2(pointP.point.y, pointP.point.x) % (2.0*np.pi)
            
        return angleToChild
    
    
    # GetPositionFrame()
    # Get the position of the given frameid in the Arena frame.
    #
    def GetPositionFrame (self, arenastate, frameid):
        stamp = arenastate.robot.header.stamp
        if len(arenastate.flies)>0:
            stamp = max(stamp,arenastate.robot.header.stamp)
            
        if self.tfrx.canTransform('Arena', frameid, stamp):        
            (trans,q) = self.tfrx.lookupTransform('Arena', frameid, stamp)
            trans = np.array(trans)
        else:
            trans = None
            
        return trans
    
    
    def ClipXyToRadius(self, x, y, rmax):
        r = np.sqrt(x*x + y*y)
        
        xOut,yOut = x,y
        if (rmax < r):
            angle = np.arctan2(y, x)
            xOut = rmax * np.cos(angle)
            yOut = rmax * np.sin(angle)
            #rospy.loginfo('EL CLIPPING x,y=%s to %s' % ([x,y],[xOut,yOut]))
            
        return [xOut,yOut]
    
    
    # step_time_elapsed()
    # Increment the elapsed time, adjusting for the pause/continue status of the experiment.
    #
    def step_time_elapsed(self):
        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.experimentparams.experiment.timeTrialElapsed += dt.to_sec()


        
    def execute(self, userdata):
        self.experimentparams = copy.deepcopy(userdata.experimentparamsIn)
        self.timePrev = rospy.Time.now()

        if self.mode == 'pre':
            self.paramsIn = self.experimentparams.pre
        if self.mode == 'trial':
            self.paramsIn = self.experimentparams.trial

        rospy.loginfo('EL State MoveRobot()')

        if self.paramsIn.robot.enabled:
            rv = 'aborted'
            
            if (self.SetStageStateRef is None):
                stSrv = 'set_stage_state_ref'
                try:
                    rospy.wait_for_service(stSrv)
                    self.SetStageStateRef = rospy.ServiceProxy(stSrv, SrvFrameState, persistent=True)
                except rospy.ServiceException, e:
                    rospy.logwarn ('EL FAILED to connect service %s(): %s' % (stSrv, e))
            
            self.timeStart = rospy.Time.now()
    
            while self.arenastate is None:
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: MoveRobot()')
                    self.service_preempt()
                    return 'preempt'
                
                if (self.commandExperiment=='exit_now'):
                    return 'aborted'

                rospy.sleep(1.0)
    
    
            if self.paramsIn.robot.move.mode=='relative':
                rv = self.MoveRelative()
            else:
                rv = self.MovePattern()
        else:
            rv = 'disabled'
            
                
        #rospy.loginfo ('EL Exiting MoveRobot()')
        userdata.experimentparamsOut = self.experimentparams
        return rv
            
            
    def MoveRelative (self):            
        ptTarget = None
        rv = 'aborted'

        # Compute the speedMax.
        if (self.paramsIn.robot.move.relative.typeSpeedMax=='constant'):
            speedMax = self.paramsIn.robot.move.relative.speedMax
        else:
            # Choose a random speed in range [0,speed]                     
            speedMax = self.paramsIn.robot.move.relative.speedMax * np.random.random() # Choose a random speed.

        
        # Compute the angleOffset (in the frameidOrigin).
        if (self.paramsIn.robot.move.relative.typeAngleOffset=='constant'):
            angleOffset = self.paramsIn.robot.move.relative.angleOffset
        elif (self.paramsIn.robot.move.relative.typeAngleOffset=='random'):
            angleOffset = self.paramsIn.robot.move.relative.angleOffset * np.random.random()
        elif (self.paramsIn.robot.move.relative.typeAngleOffset=='current'): # Use the robot's current angular position in the origin frame, rather than a given angle.
            angleOffset = self.GetAngleFrameToFrame(self.paramsIn.robot.move.relative.frameidOrigin, 'Robot')         
        else:
            rospy.logwarn ('EL, unknown robot.move.relative.typeAngleOffset: %s' % self.paramsIn.robot.move.relative.typeAngleOffset)
            angleOffset = 0

        
        # Compute the angleVelocity (in the frameidOrigin).
        if (self.paramsIn.robot.move.relative.typeAngleVelocity=='constant'):
            angleVelocity = self.paramsIn.robot.move.relative.angleVelocity
        elif (self.paramsIn.robot.move.relative.typeAngleVelocity=='random'):
            angleVelocity = self.paramsIn.robot.move.relative.angleVelocity * np.random.random()
        else:
            rospy.logwarn ('EL, unknown robot.move.relative.typeAngleVelocity: %s' % self.paramsIn.robot.move.relative.typeAngleVelocity)
            angleVelocity = 0

        
        # Compute the angleOscMag (in the frameidOrigin).
        if (self.paramsIn.robot.move.relative.typeAngleOscMag=='constant'):
            angleOscMag = self.paramsIn.robot.move.relative.angleOscMag
        elif (self.paramsIn.robot.move.relative.typeAngleOscMag=='random'):
            angleOscMag = self.paramsIn.robot.move.relative.angleOscMag * np.random.random()
        else:
            rospy.logwarn ('EL, unknown robot.move.relative.typeAngleOscMag: %s' % self.paramsIn.robot.move.relative.typeAngleOscMag)
            angleOscMag = 0

        
        # Compute the angleOscFreq (in the frameidOrigin).
        if (self.paramsIn.robot.move.relative.typeAngleOscFreq=='constant'):
            angleOscFreq = self.paramsIn.robot.move.relative.angleOscFreq
        elif (self.paramsIn.robot.move.relative.typeAngleOscFreq=='random'):
            angleOscFreq = self.paramsIn.robot.move.relative.angleOscFreq * np.random.random()
        else:
            rospy.logwarn ('EL, unknown robot.move.relative.typeAngleOscFreq: %s' % self.paramsIn.robot.move.relative.typeAngleOscFreq)
            angleOscFreq = 0
            
        t0 = self.experimentparams.experiment.timeTrialElapsed
        angleInitial = self.GetAngleFrameToFrame(self.paramsIn.robot.move.relative.frameidOrigin, 'target')
        while not rospy.is_shutdown():
            self.step_time_elapsed()
            t = self.experimentparams.experiment.timeTrialElapsed - t0
            
            # If we need to update the target position.
            if (ptTarget is None) or (self.paramsIn.robot.move.relative.tracking):
                # Update the angleOffset based on the current robot position.
                if (self.paramsIn.robot.move.relative.typeAngleOffset=='current'): # Use the robot's current angular position in the origin frame, rather than a given angle.
                    angleOffset = self.GetAngleFrameToFrame(self.paramsIn.robot.move.relative.frameidOrigin, 'Robot')
            
                # Angle of target in origin frame.
                angleInOriginFrame = angleInitial + angleOffset + angleVelocity*t + angleOscMag * np.sin(2.0 * np.pi * angleOscFreq * t) 

                # Target point in Origin frame.
                d = self.paramsIn.robot.move.relative.distance
                stamp = self.tfrx.getLatestCommonTime('Arena', self.paramsIn.robot.move.relative.frameidOrigin)
                ptsInOriginFrame = PointStamped(header=Header(frame_id=self.paramsIn.robot.move.relative.frameidOrigin, stamp=stamp),
                                                point=Point(x = d*np.cos(angleInOriginFrame), 
                                                            y = d*np.sin(angleInOriginFrame), 
                                                            z = 0.0))
                
                # Target point in Origin frame.
                ptsInArenaFrame = self.tfrx.transformPoint('Arena', ptsInOriginFrame)
                ptTarget = self.ClipXyToRadius(ptsInArenaFrame.point.x, ptsInArenaFrame.point.y, self.radiusMovement)
            

            # Send the command.
            self.target.header = self.arenastate.robot.header
            self.target.pose.position.x = ptTarget[0]
            self.target.pose.position.y = ptTarget[1]
            #rospy.logwarn (ptTarget)

            #rospy.logwarn((self.target.pose.position.x,self.target.pose.position.y))
            try:
                self.SetStageStateRef(SrvFrameStateRequest(state=MsgFrameState(header=self.target.header, 
                                                                               pose=self.target.pose,
                                                                               speed = speedMax # Max allowed speed.
                                                                               )
                                                           )
                                      )
            except rospy.ServiceException, e:
                stSrv = 'set_stage_state_ref'
                try:
                    rospy.wait_for_service(stSrv)
                    self.SetStageStateRef = rospy.ServiceProxy(stSrv, SrvFrameState, persistent=True)
                except rospy.ServiceException, e:
                    rospy.logwarn ('EL FAILED to reconnect service %s(): %s' % (stSrv, e))
                else:
                    rospy.logwarn ('EL Reconnected service %s()' % stSrv)
                    
                ptTarget = None
                    


                    
            # Check if we're there yet.
            ptRobot = np.array([self.arenastate.robot.pose.position.x, self.arenastate.robot.pose.position.y])
            if (ptTarget is not None):
                r = np.linalg.norm(ptRobot-ptTarget)
                if (r <= self.paramsIn.robot.move.relative.tolerance):
                    rv = 'success'
                    break

            
            if (self.preempt_requested()):
                rospy.loginfo('preempt requested: MoveRelative()')
                self.service_preempt()
                rv = 'preempt'
                break

            
            # Timeout is controlled by the pretrigger or the posttrigger.
            #if self.paramsIn.robot.move.timeout != -1:
            #    if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > self.paramsIn.robot.move.timeout:
            #        rv = 'timeout'
            #        break
            
            self.rosrate.sleep()


            # Handle commands.
            if (self.commandExperiment=='continue'):
                pass
            
            if (self.commandExperiment=='pause_now'):
                while (self.commandExperiment=='pause_now'):
                    rospy.sleep(0.5)
                    self.timePrev = rospy.Time.now()

            if (self.commandExperiment=='pause_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break


        ptTarget = None
        
        return rv

        
    def MovePattern (self):
                    
        msgPattern = MsgPattern()

        # Choose one from each set of choices at random.
        msgPattern.shape            = self.paramsIn.robot.move.pattern.shape
        msgPattern.points           = []
        msgPattern.frameidPosition  = self.paramsIn.robot.move.pattern.frameidPosition
        msgPattern.frameidAngle     = self.paramsIn.robot.move.pattern.frameidAngle
        msgPattern.hzPattern        = self.paramsIn.robot.move.pattern.hzPattern
        msgPattern.hzPoint          = self.paramsIn.robot.move.pattern.hzPoint
        msgPattern.count            = self.paramsIn.robot.move.pattern.count
        msgPattern.size             = self.paramsIn.robot.move.pattern.size
        msgPattern.restart          = self.paramsIn.robot.move.pattern.restart
        msgPattern.param            = 0.0#self.paramsIn.robot.move.pattern.param
        msgPattern.direction        = self.paramsIn.robot.move.pattern.direction
        self.pubSetPattern.publish (msgPattern)
                

        rv = 'aborted'
        while not rospy.is_shutdown():
            self.step_time_elapsed()
            
            if self.preempt_requested():
                rospy.loginfo('preempt requested: MovePattern()')
                self.service_preempt()
                rv = 'preempt'
                break

            # Handle commands.
            if (self.commandExperiment=='continue'):
                pass
            
            if (self.commandExperiment=='pause_now'):
                while (self.commandExperiment=='pause_now'):
                    rospy.sleep(0.5)
                    self.timePrev = rospy.Time.now()

            if (self.commandExperiment=='pause_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break

            self.rosrate.sleep()


        # Turn off the pattern
        msgPattern.count = 0
        self.pubSetPattern.publish (msgPattern)

        return rv
# End class Action()        



