#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import copy
import numpy as N
import smach
import tf

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, String
from flycore.msg import MsgFrameState
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
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('broadcast/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo("EL State ResetRobot(%s)" % [userdata.experimentparamsIn.trial.robot.home.enabled, userdata.experimentparamsIn.trial.robot.home.x, userdata.experimentparamsIn.trial.robot.home.y])

        rv = 'disabled'
        if (userdata.experimentparamsIn.trial.robot.enabled) and (userdata.experimentparamsIn.trial.robot.home.enabled):
            self.target = MsgFrameState()
            self.SetStageState = None
            self.timeStart = rospy.Time.now()
    
            rospy.wait_for_service('set_stage_state')
            try:
                self.SetStageState = rospy.ServiceProxy('set_stage_state', SrvFrameState)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            

            while self.arenastate is None:
                #if userdata.experimentparamsIn.trial.robot.home.timeout != -1:
                #    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsIn.trial.robot.home.timeout:
                #        return 'timeout'
                    
                #if self.preempt_requested():
                #    self.service_preempt()
                #    return 'preempt'
                rospy.sleep(1.0)
    
                if (self.commandExperiment=='exit_now'):
                    return 'aborted'
    
    
            # Send the command.
            self.target.header = self.arenastate.robot.header
            #self.target.header.stamp = rospy.Time.now()
            self.target.pose.position.x = userdata.experimentparamsIn.trial.robot.home.x
            self.target.pose.position.y = userdata.experimentparamsIn.trial.robot.home.y
            self.SetStageState(SrvFrameStateRequest(state=MsgFrameState(header=self.target.header, 
                                                                        pose=self.target.pose),
                                                    speed = userdata.experimentparamsIn.trial.robot.home.speed))

            rv = 'aborted'
            while not rospy.is_shutdown():
                # Are we there yet?
                
                ptRobot = N.array([self.arenastate.robot.pose.position.x, 
                                   self.arenastate.robot.pose.position.y])
                ptTarget = N.array([self.target.pose.position.x,
                                    self.target.pose.position.y])                
                r = N.linalg.norm(ptRobot-ptTarget)
                rospy.loginfo ('EL ResetHardware() ptTarget=%s, ptRobot=%s, r=%s' % (ptTarget, ptRobot, r))
                
                
                if (r <= userdata.experimentparamsIn.trial.robot.home.tolerance):
                    rospy.sleep(0.5) # Allow some settling time.
                    rv = 'success'
                    break
                
                
                if self.preempt_requested():
                    rospy.loginfo('preempt requested: ResetRobot()')
                    self.service_preempt()
                    rv = 'preempt'
                    break
                
                
                #if userdata.experimentparamsIn.trial.robot.home.timeout != -1:
                #    if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.trial.robot.home.timeout:
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
                             input_keys=['experimentparamsIn'])

        self.mode = mode
        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        self.pubPatternGen = rospy.Publisher('SetPattern', MsgPattern, latch=True)

        self.target = MsgFrameState()
        self.SetStageState = None

        self.radiusMovement = float(rospy.get_param("arena/radius_inner","25.4"))
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('broadcast/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def GetAngleFrame (self, arenastate, frameid):
        stamp = arenastate.robot.header.stamp
        if len(arenastate.flies)>0:
            stamp = max(stamp,arenastate.flies[0].header.stamp)
    
        if self.tfrx.canTransform('Arena', frameid, stamp):        
            (trans,q) = self.tfrx.lookupTransform('Arena', frameid, stamp)
            rpy = tf.transformations.euler_from_quaternion(q)
            angle = rpy[2] % (2.0 * N.pi)
        else:
            angle = 0.0
    
            
        return angle
    
    
    def GetAngleFrameToFrame (self, frameidParent, frameidChild):
        angleToChild = None
        try:
            stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = self.tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception, e:
            #rospy.logwarn('Exception in GetAngleFrameToFrame():  %s' % e)
            pass
        else:
            angleToChild = N.arctan2(pointP.point.y, pointP.point.x) % (2.0*N.pi)
            
        return angleToChild
    
    
    def GetPositionFrame (self, arenastate, frameid):
        stamp = arenastate.robot.header.stamp
        if len(arenastate.flies)>0:
            stamp = max(stamp,arenastate.robot.header.stamp)
            
        if self.tfrx.canTransform('Arena', frameid, stamp):        
            (trans,q) = self.tfrx.lookupTransform('Arena', frameid, stamp)
            trans = N.array(trans)
        else:
            trans = None
            
        return trans
    
    
    def ClipXyToRadius(self, x, y, rmax):
        r = N.sqrt(x*x + y*y)
        
        xOut,yOut = x,y
        if (rmax < r):
            angle = N.arctan2(y, x)
            xOut = rmax * N.cos(angle)
            yOut = rmax * N.sin(angle)
            #rospy.loginfo('EL CLIPPING x,y=%s to %s' % ([x,y],[xOut,yOut]))
            
        return [xOut,yOut]
    
    
    def execute(self, userdata):
        if self.mode == 'pre':
            self.paramsIn = userdata.experimentparamsIn.pre
        if self.mode == 'trial':
            self.paramsIn = userdata.experimentparamsIn.trial

        rospy.loginfo("EL State MoveRobot(%s)" % [self.paramsIn.robot.move.mode,
                                                  self.paramsIn.robot.move.relative.distance, 
                                                  self.paramsIn.robot.move.relative.angle, 
                                                  self.paramsIn.robot.move.relative.speed])

        if self.paramsIn.robot.enabled:
            rv = 'aborted'
            rospy.wait_for_service('set_stage_state')
            try:
                self.SetStageState = rospy.ServiceProxy('set_stage_state', SrvFrameState)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
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
        return rv
            
            
    def MoveRelative (self):            
        self.ptTarget = None
        rv = 'aborted'

        while not rospy.is_shutdown():
            posRobot = self.arenastate.robot.pose.position # Assumed in the "Arena" frame.
            ptRobot = N.array([posRobot.x, posRobot.y])
            
            # Get a random speed once per move, non-random speed always.
            angleSpeed = 0.0
            if (self.paramsIn.robot.move.relative.speedType=='random'):
                if (self.ptTarget is None):
                    # Choose a random speed in range [-speed,+speed]                     
                    speedTarget = self.paramsIn.robot.move.relative.speed * (2.0*N.random.random() - 1.0) # Choose a random speed, plus or minus.
                    # Convert speed to +speed and angle.                     
                    if speedTarget < 0:
                        speedTarget = -speedTarget
                        angleSpeed = N.pi
            else:
                speedTarget = self.paramsIn.robot.move.relative.speed

            
            
            if (self.paramsIn.robot.move.relative.angleType=='random'):     
                if (self.ptTarget is None):                                 # Get a random angle once per move.
                    angleBase = 2.0*N.pi * N.random.random()
                    angleRel = 0.0
                    angle = (angleBase + angleRel + angleSpeed) % (2.0*N.pi)
                # else we already computed the angle.
                    
            elif (self.paramsIn.robot.move.relative.angleType=='constant'):
                if (self.ptTarget is None) or (self.paramsIn.robot.move.relative.tracking):
                    angleBase = self.GetAngleFrame(self.arenastate, self.paramsIn.robot.move.relative.frameidOriginAngle)
                    angleRel = self.paramsIn.robot.move.relative.angle
                # else we already computed the angle.

            elif (self.paramsIn.robot.move.relative.angleType=='current'):
                if (self.ptTarget is None) or (self.paramsIn.robot.move.relative.tracking):
                    angleBase = self.GetAngleFrame(self.arenastate, self.paramsIn.robot.move.relative.frameidOriginAngle)
                    angleRel = self.GetAngleFrameToFrame(self.paramsIn.robot.move.relative.frameidOriginAngle, 'Robot')
                    if (angleRel is None):
                        angleRel = 0.0
                # else we already computed the angle.
                
            else:
                rospy.logwarn ('EL, unknown robot.move.relative.angleType: %s' % self.paramsIn.robot.move.relative.angleType)
                angleBase = 0.0
                angleRel = 0.0    
                
            angle = (angleBase + angleRel + angleSpeed) % (2.0*N.pi)

                                                   
            # Move a distance relative to whose position?
            #if self.paramsIn.robot.move.relative.frameidOriginPosition=="Fly1" and (len(self.arenastate.flies)>0):
            #    posOrigin = posFly
            #    doMove = True
            #elif self.paramsIn.robot.move.relative.frameidOriginPosition=="Robot":
            #    posOrigin = posRobot
            #    doMove = True
            #else:
            #    posOrigin = Point(x=0, y=0, z=0) # Relative to the origin of the Arena frame
            #    doMove = False
            

            # If we need to calculate a target.
            if (self.ptTarget is None) or (self.paramsIn.robot.move.relative.tracking):
                doMove = True
                # Compute target point in workspace (i.e. Arena) coordinates.
                #ptOrigin = N.array([posOrigin.x, posOrigin.y])
                ptOrigin = self.GetPositionFrame(self.arenastate, self.paramsIn.robot.move.relative.frameidOriginPosition)
                if (ptOrigin is not None):
                    d = self.paramsIn.robot.move.relative.distance
                    ptRelative = d * N.array([N.cos(angle), N.sin(angle)])
                    ptTarget = ptOrigin[0:2] + ptRelative
                    self.ptTarget = self.ClipXyToRadius(ptTarget[0], ptTarget[1], self.radiusMovement)

                    # Send the command.
                    self.target.header = self.arenastate.robot.header
                    self.target.pose.position.x = self.ptTarget[0]
                    self.target.pose.position.y = self.ptTarget[1]
                    #rospy.logwarn (self.ptTarget)
                    try:
                        if (doMove):
                            self.SetStageState(SrvFrameStateRequest(state=MsgFrameState(header=self.target.header, 
                                                                                        pose=self.target.pose),
                                                                    speed = speedTarget))
                    except (rospy.ServiceException, rospy.exceptions.ROSInterruptException), e:
                        rospy.logwarn ('EL Exception calling set_stage_state(): %s' % e)
                        self.ptTarget = None


                    
            # Check if we're there yet.
            if self.ptTarget is not None:
                r = N.linalg.norm(ptRobot-self.ptTarget)
                if (r <= self.paramsIn.robot.move.relative.tolerance):
                    rv = 'success'
                    break

            
            if self.preempt_requested():
                rospy.loginfo('preempt requested: MoveRelative()')
                self.service_preempt()
                rv = 'preempt'
                break

            
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

            if (self.commandExperiment=='pause_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break


        self.ptTarget = None
        
        return rv

        
    def MovePattern (self):
                    
        msgPattern = MsgPattern()

        # Publish the pattern message.
        msgPattern.shape = self.paramsIn.robot.move.pattern.shape
        msgPattern.points = []
        msgPattern.frameidPosition = self.paramsIn.robot.move.pattern.frameidPosition
        msgPattern.frameidAngle = self.paramsIn.robot.move.pattern.frameidAngle
        msgPattern.hzPattern = self.paramsIn.robot.move.pattern.hzPattern
        msgPattern.hzPoint = self.paramsIn.robot.move.pattern.hzPoint
        msgPattern.count = self.paramsIn.robot.move.pattern.count
        msgPattern.size = self.paramsIn.robot.move.pattern.size
        msgPattern.preempt = True
        msgPattern.param = 0.0#self.paramsIn.robot.move.pattern.param
        msgPattern.direction = self.paramsIn.robot.move.pattern.direction
        self.pubPatternGen.publish (msgPattern)
                

        rv = 'aborted'
        while not rospy.is_shutdown():
            if self.preempt_requested():
                rospy.loginfo('preempt requested: MovePattern()')
                self.service_preempt()
                rv = 'preempt'
                break

            
            #if self.paramsIn.robot.move.timeout != -1:
            #    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > self.paramsIn.robot.move.timeout:
            #        rv = 'timeout'
            #        break
            
            self.rosrate.sleep()


            # Handle commands.
            if (self.commandExperiment=='continue'):
                pass
            
            if (self.commandExperiment=='pause_now'):
                while (self.commandExperiment=='pause_now'):
                    rospy.sleep(0.5)

            if (self.commandExperiment=='pause_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_after_trial'):
                pass
            
            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break


        # Turn off the pattern
        msgPattern.shape = self.paramsIn.robot.move.pattern.shape
        msgPattern.points = []
        msgPattern.frameidPosition = self.paramsIn.robot.move.pattern.frameidPosition
        msgPattern.frameidAngle = self.paramsIn.robot.move.pattern.frameidAngle
        msgPattern.hzPattern = self.paramsIn.robot.move.pattern.hzPattern
        msgPattern.hzPoint = self.paramsIn.robot.move.pattern.hzPoint
        msgPattern.count = 0
        msgPattern.size = self.paramsIn.robot.move.pattern.size
        msgPattern.preempt = True
        msgPattern.param = 0.0#self.paramsIn.robot.move.pattern.param
        msgPattern.direction = self.paramsIn.robot.move.pattern.direction
        self.pubPatternGen.publish (msgPattern)

        return rv
# End class Action()        



