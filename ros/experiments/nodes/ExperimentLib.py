#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import actionlib
import numpy as N
import smach
import smach_ros
import tf

from geometry_msgs.msg import Pose, Point, Quaternion
from stage_action_server.msg import *
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from tracking.msg import ArenaState
from experiments.srv import Trigger, ExperimentParams
from patterngen.msg import MsgPatternGen


#######################################################################################################
#######################################################################################################
def GetNearestFly (arenastate):
    iBest = None
    if len(arenastate.flies)>0:
        # Find the nearest fly.
        rBest = N.inf
        vFlyBest = None
        for iFly in range(len(arenastate.flies)):
            vRobot = N.array([arenastate.robot.pose.position.x, 
                              arenastate.robot.pose.position.y])
            vFly   = N.array([arenastate.flies[iFly].pose.position.x, 
                              arenastate.flies[iFly].pose.position.y])
            r = N.linalg.norm(vRobot-vFly)
            #rospy.loginfo ('vRobot=%s, vFly=%s, r=%s, iFly=%s' % (vRobot, vFly, r, iFly))
            if r<rBest:
                rBest = r
                iBest = iFly
                vFlyBest = vFly


        #rospy.loginfo ('EL vRobot=%s, vFlyBest=%s, rBest=%s, iBest=%s' % (vRobot, vFlyBest, rBest, iBest))
                
    return 0 #iBest
                    

def GetOrientationRobot (arenastate):
    q = arenastate.robot.pose.orientation
    rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    angle = rpy[2] % (2.0 * N.pi)
        
    return angle


def GetOrientationFly (arenastate, iFly):
    angle = None
    if len(arenastate.flies)-1 >= iFly:
        q = arenastate.flies[iFly].pose.orientation
        rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        angle = rpy[2] % (2.0 * N.pi)
        
    return angle


def GetAngleToRobotInFlyView (arenastate, iFly):
    angle = None
    if len(arenastate.flies)-1 >= iFly:
        dx = arenastate.robot.pose.position.x - arenastate.flies[iFly].pose.position.x
        dy = arenastate.robot.pose.position.y - arenastate.flies[iFly].pose.position.y
        angleToRobot = N.arctan2(dy,dx)
        
        qFly = arenastate.flies[iFly].pose.orientation
        rpy = tf.transformations.euler_from_quaternion((qFly.x, qFly.y, qFly.z, qFly.w))
        angleOfFly = rpy[2]
        angle = (angleToRobot - angleOfFly) % (2.0 * N.pi)
        
    #rospy.logwarn('EL GetAngleToRobotInFlyView()=%s' % angle)
    return angle


def GetSpeedFly (arenastate, iFly):
    speed = None
    if len(arenastate.flies)-1 >= iFly:
        speed = N.linalg.norm(N.array([arenastate.flies[iFly].velocity.linear.x,
                                       arenastate.flies[iFly].velocity.linear.y,
                                       arenastate.flies[iFly].velocity.linear.z]))
        
    #rospy.logwarn ('EL GetSpeedFly()=%s' % speed)
    return speed


def GetDistanceFlyToRobot (arenastate, iFly):
    distance = None
    if len(arenastate.flies)-1 >= iFly:
        dx = arenastate.robot.pose.position.x - arenastate.flies[iFly].pose.position.x
        dy = arenastate.robot.pose.position.y - arenastate.flies[iFly].pose.position.y
        distance = N.linalg.norm([dx,dy])
        
    #rospy.logwarn('EL GetDistanceFlyToRobot()=%s' % distance)
    return distance


def ClipXyToRadius(x, y, rmax):
    r = N.sqrt(x*x + y*y)
    
    xOut,yOut = x,y
    if (rmax*0.8) < r:
        angle = N.arctan2(y, x)
        xOut = (rmax*0.8) * N.cos(angle)
        yOut = (rmax*0.8) * N.sin(angle)
        rospy.logwarn('EL CLIPPING x,y=%s to %s' % ([x,y],[xOut,yOut]))
        
    return [xOut,yOut]


def TriggerServiceAttach():
    trigger = None
    stSrv = "trigger"
    rospy.wait_for_service(stSrv)
    try:
        trigger = rospy.ServiceProxy(stSrv, Trigger)
    except rospy.ServiceException, e:
        rospy.logwarn ("FAILED %s: %s"%(stSrv,e))
        
    return trigger


def NewTrialServiceAttach():
    NewTrial = None    
    stSrv = "new_trial"
    rospy.wait_for_service(stSrv)
    try:
        NewTrial = rospy.ServiceProxy(stSrv, ExperimentParams)
    except rospy.ServiceException, e:
        rospy.logwarn ("FAILED %s: %s"%(stSrv,e))
        
    return NewTrial
    

#######################################################################################################
#######################################################################################################
class TemplateState (smach.State):
    def __init__(self, type='entry', timeout=None):
        self.type = type
        self.timeout = timeout
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])

        rospy.on_shutdown(self.OnShutdown_callback)
        self.arenastate = None
        self.rosrate = rospy.Rate(100)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)

        self.TriggerNotify = TriggerServiceAttach()
        
    
    def OnShutdown_callback(self):
        pass
        

    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def execute(self, userdata):
        rospy.logwarn('Starting state (template)')
        self.timeStart = rospy.Time.now()
    
        while self.arenastate is None:
            if self.preempt_requested():
                self.TriggerNotify(False)
                return 'preempted'
            rospy.sleep(1.0)

        rospy.logwarn("EL State state (template)...")
        rv = 'aborted'
        while True:
            try:
                #if something:
                #    rv = 'succeeded'

                #if something:
                #    rv = 'aborted'
                
                if self.preempt_requested():
                    rv = 'preempted'
                    break
                
                if self.timeout is not None:
                    if rospy.Time.now()-self.timeStart > self.timeout:
                        rv = 'preempted'
                        break
            except:
                pass
            
            self.rosrate.sleep()


         # Send trigger status.
        if rv=='succeeded' and self.type=='entry':
            self.TriggerNotify(True)
        else:
            self.TriggerNotify(False)

        
                 
        return rv

    
#######################################################################################################
#######################################################################################################
class NewExperiment (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        
        
    def execute(self, userdata):
        es = userdata.experimentparamsIn
        rospy.logwarn("EL State NewExperiment(%s)" % es)

        es.experiment.trial = es.experiment.trial-1 
        userdata.experimentparamsOut = es
        
        #rospy.logwarn ('EL Exiting NewExperiment()')
        return 'succeeded'


#######################################################################################################
#######################################################################################################
# NewTrial() - Increments the trial number, untriggers, and calls the 
#              new_trial service (which begins recording).
#
class NewTrial (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        #self.pub_experimentparams = rospy.Publisher('ExperimentParams', ExperimentParams)
        self.TriggerNotify = TriggerServiceAttach()
        self.NewTrial = NewTrialServiceAttach()

        
    def execute(self, userdata):
        rv = 'aborted'
        experimentparams = userdata.experimentparamsIn
        experimentparams.experiment.trial = userdata.experimentparamsIn.experiment.trial+1
        if (experimentparams.experiment.maxTrials != -1):
            if (experimentparams.experiment.maxTrials < experimentparams.experiment.trial):
                return rv

        rospy.logwarn ('EL State NewTrial(%s)' % experimentparams.experiment.trial)

        self.TriggerNotify(False)
        userdata.experimentparamsOut = experimentparams
        #self.pub_experimentparams.publish(experimentparams)
        try:
            self.NewTrial (experimentparams)
            rv = 'succeeded'
        except rospy.ServiceException:
            rv = 'aborted'

        #rospy.logwarn ('EL Exiting NewTrial()')
        return rv



#######################################################################################################
#######################################################################################################
class TriggerOnStates (smach.State):
    def __init__(self, type='entry'):
        
        self.type               = type
        
        self.isTriggered        = False
        self.timeTriggered      = None
        
        
        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        rospy.on_shutdown(self.OnShutdown_callback)
        self.arenastate = None
        self.rosrate = rospy.Rate(100)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)

        self.TriggerNotify = TriggerServiceAttach()
    
    
    def OnShutdown_callback(self):
        pass
        

    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def execute(self, userdata):
        rospy.logwarn("EL State TriggerOnStates(%s)" % (self.type))

        if self.type == 'entry':
            trigger = userdata.experimentparamsIn.triggerEntry
        else:
            trigger = userdata.experimentparamsIn.triggerExit

        rv = 'succeeded'
        try:
            if trigger.enabled:
                self.timeStart = rospy.Time.now()
                self.isTriggered = False
                self.timeTriggered  = None
                
                
                while self.arenastate is None:
                    if self.preempt_requested():
                        self.TriggerNotify(False)
                        return 'preempted'
                    rospy.sleep(1.0)
        
        
                rv = 'aborted'
                while not rospy.is_shutdown():
                    if len(self.arenastate.flies)>0:
                        iFly = GetNearestFly(self.arenastate)
                        if iFly is None:
                            continue
        
                        # Test for distance.
                        isDistanceInRange = True
                        dMin = trigger.distanceMin
                        dMax = trigger.distanceMax
                        distance = None
                        if (dMin is not None) and (dMax is not None):
                            distance = GetDistanceFlyToRobot(self.arenastate, iFly)
                            isDistanceInRange = False
                            if (dMin <= distance <= dMax):
                                isDistanceInRange = True
                                
                        # Test for angle.
                        isAngleInRange = True
                        angleMin = trigger.angleMin
                        angleMax = trigger.angleMax
                        angleTest = trigger.angleTest
                        angleTestBilateral = trigger.angleTestBilateral
                        if (angleMin is not None) and (angleMax is not None):
                            angle = GetAngleToRobotInFlyView(self.arenastate, iFly)
                            
                            angleA1 = angleMin % (2.0*N.pi)
                            angleA2 = angleMax % (2.0*N.pi)
                            angleB1 = (2.0*N.pi - angleA2) # % (2.0*N.pi)
                            angleB2 = (2.0*N.pi - angleA1) # % (2.0*N.pi)
            
                            if angle is not None:
                                # Test for angle meeting the angle criteria.
                                isAngleInRange = False
                                if angleTestBilateral:
                                    if angleTest=='inclusive':
                                        #rospy.logwarn('EL angles %s' % [angleA1, angleA2, angleB1, angleB2, angle])
                                        if (angleA1 <= angle <= angleA2) or (angleB1 <= angle <= angleB2):
                                            isAngleInRange = True
                                            
                                    elif angleTest=='exclusive':
                                        if (0.0 <= angle < angleA1) or (angleA2 < angle < angleB1) or (angleB2 < angle <= (2.0*N.pi)):
                                            isAngleInRange = True
                                else:
                                    if angleTest=='inclusive':
                                        if (angleA1 <= angle <= angleA2):
                                            isAngleInRange = True
                                            
                                    elif angleTest=='exclusive':
                                        if (angle < angleA1) or (angleA2 < angle):
                                            isAngleInRange = True
                            
                        
                        # Test for speed.
                        isSpeedInRange = True
                        speedMin = trigger.speedMin
                        speedMax = trigger.speedMax
                        if (speedMin is not None) and (speedMax is not None):
                            speed = GetSpeedFly(self.arenastate, iFly)
                            #rospy.loginfo ('EL speed=%s' % speed)
                            isSpeedInRange = False
                            if speed is not None:
                                if (speedMin <= speed <= speedMax):
                                    isSpeedInRange = True
        
                        # Test all the trigger criteria.
                        if isDistanceInRange and isAngleInRange and isSpeedInRange:
                            
                            # Set the pending trigger start time.
                            if not self.isTriggered:
                                self.isTriggered = True
                                self.timeTriggered = rospy.Time.now()
                        else:
                            # Cancel a pending trigger.
                            self.isTriggered = False
                            self.timeTriggered = None
        
                        #if (distance is not None) and (angle is not None) and (speed is not None):
                        #    rospy.logwarn ('EL triggers=distance=%0.3f, speed=%0.3f, angle=%0.3f, bools=%s' % (distance, speed, angle, [isDistanceInRange, isSpeedInRange, isAngleInRange]))
        
                        # If pending trigger has lasted longer than requested duration, then set trigger.
                        if (self.isTriggered):
                            duration = rospy.Time.now().to_sec() - self.timeTriggered.to_sec()
                            
                            if duration >= trigger.timeHold:
                                rv = 'succeeded'
                                break
                            
                            #rospy.logwarn('EL duration=%s' % duration)
                            
                        
                    if self.preempt_requested():
                        rv = 'preempted'
                        break
                    
                    if trigger.timeout != -1:
                        if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > trigger.timeout:
                            rv = 'preempted'
                            break
                    
                    self.rosrate.sleep()
    
            if rv=='succeeded' and self.type=='entry':
                self.TriggerNotify(True)
            else:
                self.TriggerNotify(False)
        except rospy.ServiceException:
            rv = 'aborted'
            
        #rospy.logwarn ('EL Exiting TriggerOnStates()')
        return rv

    

#######################################################################################################
#######################################################################################################
class TriggerOnTime (smach.State):
    def __init__(self, type='entry'):
        self.type = type
        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        self.TriggerNotify = TriggerServiceAttach()
        

    def execute(self, userdata):
        rospy.logwarn("EL State TriggerOnTime(%s)" % (userdata.experimentparamsIn.waitEntry))
        rv = 'succeeded'
        rospy.sleep(userdata.experimentparamsIn.waitEntry)

    
        if rv=='succeeded' and self.type=='entry':
            self.TriggerNotify(True)
        else:
            self.TriggerNotify(False)

        #rospy.logwarn ('EL Exiting TriggerOnTime()')
        return rv



#######################################################################################################
#######################################################################################################
class GotoHome (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        self.arenastate = None
        self.rosrate = rospy.Rate(100)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)

        self.action = actionlib.SimpleActionClient('StageActionServer', ActionStageStateAction)
        self.action.wait_for_server()
        self.goal = ActionStageStateGoal()

        rospy.wait_for_service('set_stage_state')
        try:
            self.set_stage_state = rospy.ServiceProxy('set_stage_state', SrvFrameState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.logwarn("EL State GotoHome(%s)" % [userdata.experimentparamsIn.home.enabled, userdata.experimentparamsIn.home.x, userdata.experimentparamsIn.home.y])

        rv = 'succeeded'
        if userdata.experimentparamsIn.home.enabled:
            self.timeStart = rospy.Time.now()
    
            while self.arenastate is None:
                if self.preempt_requested():
                    return 'preempted'
                rospy.sleep(1.0)
    
    
    
            # Send the command.
            self.goal.state.header = self.arenastate.robot.header
            self.goal.state.header.stamp = rospy.Time.now()
            self.goal.state.pose.position.x = userdata.experimentparamsIn.home.x
            self.goal.state.pose.position.y = userdata.experimentparamsIn.home.y
            self.set_stage_state(SrvFrameStateRequest(state=MsgFrameState(header=self.goal.state.header, 
                                                                          pose=self.goal.state.pose),
                                                      speed = userdata.experimentparamsIn.move.relative.speed))

            rv = 'aborted'
            while True:
                # Are we there yet?
                
                ptRobot = N.array([self.arenastate.robot.pose.position.x, 
                                   self.arenastate.robot.pose.position.y])
                ptTarget = N.array([self.goal.state.pose.position.x,
                                    self.goal.state.pose.position.y])                
                r = N.linalg.norm(ptRobot-ptTarget)
                rospy.logwarn ('EL GotoHome() ptTarget=%s, ptRobot=%s, r=%s' % (ptTarget, ptRobot, r))
                
                
                if (r <= userdata.experimentparamsIn.home.tolerance):
                    rospy.sleep(0.5) # Allow some settling time.
                    rv = 'succeeded'
                    break
                
                
                if self.preempt_requested():
                    rv = 'preempted'
                    break
                
                
                if userdata.experimentparamsIn.home.timeout != -1:
                    if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.home.timeout:
                        rv = 'preempted'
                        break
                
                self.rosrate.sleep()


        #rospy.logwarn ('EL Exiting MoveRobot()')
        return rv
        


#######################################################################################################
#######################################################################################################
class MoveRobot (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['succeeded','preempted','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        self.arenastate = None
        self.rosrate = rospy.Rate(100)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)
        self.pubPatternGen = rospy.Publisher('PatternGen', MsgPatternGen, latch=True)

        self.action = actionlib.SimpleActionClient('StageActionServer', ActionStageStateAction)
        self.action.wait_for_server()
        self.goal = ActionStageStateGoal()

        rospy.wait_for_service('set_stage_state')
        try:
            self.set_stage_state = rospy.ServiceProxy('set_stage_state', SrvFrameState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.radiusMovement = float(rospy.get_param("arena/radius_movement","25.4"))
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.logwarn("EL State MoveRobot(%s)" % [userdata.experimentparamsIn.move.relative.distance, userdata.experimentparamsIn.move.relative.angle, userdata.experimentparamsIn.move.relative.speed])

        rv = 'succeeded'
        if userdata.experimentparamsIn.move.enabled:
            self.timeStart = rospy.Time.now()
    
            while self.arenastate is None:
                if self.preempt_requested():
                    return 'preempted'
                rospy.sleep(1.0)
    
            if userdata.experimentparamsIn.move.mode=='relative':
                rv = self.MoveRelative(userdata)
            else:
                rv = self.MovePattern(userdata)
                
        #rospy.logwarn ('EL Exiting MoveRobot()')
        return rv
            
            
    def MoveRelative (self, userdata):            
        self.ptTarget = None
        rv = 'aborted'

        while not rospy.is_shutdown():
            posRobot = self.arenastate.robot.pose.position # Assumed in the "Plate" frame.
            ptRobot = N.array([posRobot.x, posRobot.y])
            
            # Fly data.                        
            if (len(self.arenastate.flies)>0):
                iFly = GetNearestFly(self.arenastate)
                posFly = self.arenastate.flies[iFly].pose.position # Assumed in the "Plate" frame

            
            # Get a random velocity once per move, non-random velocity always.
            if (userdata.experimentparamsIn.move.relative.speedType=='random'):
                if (self.ptTarget is None):
                    # Choose a random velocity forward or backward.                        
                    speedTarget = userdata.experimentparamsIn.move.relative.speed * (2.0*N.random.random() - 1.0) # Choose a random vel, plus or minus.
                    # Choose a random velocity forward or backward.                        
                    if speedTarget < 0:
                        speedTarget = -speedTarget
                        angleDirection = N.pi
                    else:
                        angleDirection = 0.0
            else:
                speedTarget = userdata.experimentparamsIn.move.relative.speed
                angleDirection = 0.0

            
            # Get a random angle once per move, non-random angle always.
            if (userdata.experimentparamsIn.move.relative.angleType=='random'):
                if (self.ptTarget is None):
                    angle = 2.0*N.pi*N.random.random()
            else:
                # Move at an angle relative to whose orientation?
                if (userdata.experimentparamsIn.move.relative.frameidOriginAngle=="Fly1") and (len(self.arenastate.flies)>0):
                    angle = GetOrientationFly(self.arenastate, iFly) + userdata.experimentparamsIn.move.relative.angle
                elif userdata.experimentparamsIn.move.relative.frameidOriginAngle=="Robot":
                    angle = GetOrientationRobot(self.arenastate) + userdata.experimentparamsIn.move.relative.angle
                else:
                    angle = userdata.experimentparamsIn.move.relative.angle # Relative to orientation of the Plate frame.

                # Correct the angle for the velocity sign.
                angle = (angle + angleDirection) % (2.0*N.pi)    
                
                                                   
            # Move a distance relative to whose position?
            if userdata.experimentparamsIn.move.relative.frameidOriginPosition=="Fly1" and (len(self.arenastate.flies)>0):
                posOrigin = posFly
            elif userdata.experimentparamsIn.move.relative.frameidOriginPosition=="Robot":
                posOrigin = posRobot
            else:
                posOrigin = Point(x=0, y=0, z=0) # Relative to the origin of the Plate frame


            # If we need to calculate a target.
            if (self.ptTarget is None) or (userdata.experimentparamsIn.move.relative.tracking):
                # Compute target point in workspace (i.e. Plate) coordinates.
                ptOrigin = N.array([posOrigin.x, posOrigin.y])
                d = userdata.experimentparamsIn.move.relative.distance
                ptRelative = d * N.array([N.cos(angle), N.sin(angle)])
                ptTarget = ptOrigin + ptRelative
                self.ptTarget = ClipXyToRadius(ptTarget[0], ptTarget[1], self.radiusMovement)
                #self.ptTarget = ptTarget

                #rospy.logwarn ('EL self.ptTarget=%s, ptOrigin=%s, ptRelative=%s, angle=%s, frameAngle=%s' % (self.ptTarget, ptOrigin, ptRelative, angle,userdata.experimentparamsIn.move.relative.frameidOriginAngle))
                #rospy.logwarn('EL Robot/Fly frame_id=%s' % [self.arenastate.robot.header.frame_id,self.arenastate.flies[iFly].header.frame_id])

                # Send the command.
                #self.goal.state.header = self.arenastate.flies[iFly].header
                self.goal.state.header = self.arenastate.robot.header
                self.goal.state.header.stamp = rospy.Time.now()
                self.goal.state.pose.position.x = self.ptTarget[0]
                self.goal.state.pose.position.y = self.ptTarget[1]
                
                
                #rospy.logwarn('EL calling set_stage_state(%s) pre' % [self.goal.state.pose.position.x,self.goal.state.pose.position.y])
                try:
                    self.set_stage_state(SrvFrameStateRequest(state=MsgFrameState(header=self.goal.state.header, 
                                                                                  pose=self.goal.state.pose),
                                                              speed = speedTarget))
                except (rospy.ServiceException, rospy.ROSInterruptException):
                    self.ptTarget = None
                #rospy.logwarn('EL calling set_stage_state(%s) post' % [self.goal.state.pose.position.x,self.goal.state.pose.position.y])


            # If no flies and no target, then preempt.
            #if (len(self.arenastate.flies)==0) and (self.ptTarget is None):
            #    rv = 'preempted'
            #    break

                    
            # Check if we're there yet.
            if self.ptTarget is not None:
                r = N.linalg.norm(ptRobot-self.ptTarget)
                #rospy.logwarn ('EL ptTarget=%s, ptRobot=%s, r=%s' % (self.ptTarget, ptRobot, r))
                if (r <= userdata.experimentparamsIn.move.relative.tolerance):
                    rv = 'succeeded'
                    break

            
            if self.preempt_requested():
                rv = 'preempted'
                break

            
            if userdata.experimentparamsIn.move.timeout != -1:
                if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.move.timeout:
                    rv = 'preempted'
                    break
            
            self.rosrate.sleep()


        self.ptTarget = None
        
        return rv

        
    def MovePattern (self, userdata):
                    
        msgPattern = MsgPatternGen()

        # Publish the spiral pattern message.
        msgPattern.mode = 'byshape'
        msgPattern.shape = userdata.experimentparamsIn.move.pattern.shape
        msgPattern.points = []
        msgPattern.hz = userdata.experimentparamsIn.move.pattern.hz
        msgPattern.count = userdata.experimentparamsIn.move.pattern.count
        msgPattern.radius = userdata.experimentparamsIn.move.pattern.radius
        msgPattern.preempt = True
        self.pubPatternGen.publish (msgPattern)
                

        rv = 'succeeded'
        while not rospy.is_shutdown():
            if self.preempt_requested():
                rv = 'preempted'
                break

            
            if userdata.experimentparamsIn.move.timeout != -1:
                if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.move.timeout:
                    rv = 'preempted'
                    break
            
            self.rosrate.sleep()

        # Turn off the pattern
        msgPattern.mode = 'byshape'
        msgPattern.shape = userdata.experimentparamsIn.move.pattern.shape
        msgPattern.points = []
        msgPattern.hz = userdata.experimentparamsIn.move.pattern.hz
        msgPattern.count = 0
        msgPattern.radius = userdata.experimentparamsIn.move.pattern.radius
        msgPattern.preempt = True
        self.pubPatternGen.publish (msgPattern)

        return rv
        


#######################################################################################################
#######################################################################################################
class Experiment():
    def __init__(self, experimentparams=None):
        self.sm = smach.StateMachine(['succeeded','aborted','preempted'])
        self.sm.userdata.experimentparams = experimentparams
        
        self.xHome = 0
        self.yHome = 0
        
        
        with self.sm:
            #self.goalStart = stage_action_server.msg.ActionStageStateGoal()
            #self.goalStart.state.header.frame_id = 'Plate'
            #self.goalStart.state.pose.position.x = self.xHome
            #self.goalStart.state.pose.position.y = self.yHome

            # Add states.
            smach.StateMachine.add('NEW_EXPERIMENT',
                                   NewExperiment(),
                                   transitions={'succeeded':'GOTO_HOME_PRE',
                                                'aborted':'aborted',
                                                'preempted':'NEW_EXPERIMENT'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('GOTO_HOME_PRE',
                                   GotoHome(),
                                   #smach_ros.SimpleActionState('StageActionServer',
                                   #                            ActionStageStateAction,
                                   #                            goal=self.goalStart),
                                   transitions={'succeeded':'NEW_TRIAL',
                                                'aborted':'aborted',
                                                'preempted':'GOTO_HOME_PRE'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('NEW_TRIAL',
                                   NewTrial(),
                                   transitions={'succeeded':'ENTRYWAIT',
                                                'aborted':'aborted',
                                                'preempted':'NEW_TRIAL'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('ENTRYWAIT', 
                                   TriggerOnTime(type='none'),
                                   transitions={'succeeded':'ENTRYTRIGGER',
                                                'aborted':'aborted',
                                                'preempted':'NEW_TRIAL'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('ENTRYTRIGGER', 
                                   TriggerOnStates(type='entry'),
                                   transitions={'succeeded':'MOVE',
                                                'aborted':'aborted',
                                                'preempted':'NEW_TRIAL'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('MOVE', 
                                   MoveRobot (),
                                   transitions={'succeeded':'EXITTRIGGER',
                                                'aborted':'aborted',
                                                'preempted':'NEW_TRIAL'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('EXITTRIGGER', 
                                   TriggerOnStates(type='exit'),
                                   transitions={'succeeded':'GOTO_HOME_POST',
                                                'aborted':'aborted',
                                                'preempted':'GOTO_HOME_POST'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('GOTO_HOME_POST',
                                   GotoHome(),
                                   #smach_ros.SimpleActionState('StageActionServer',
                                   #                            ActionStageStateAction,
                                   #                            goal=self.goalStart),
                                   transitions={'succeeded':'NEW_TRIAL',
                                                'aborted':'aborted',
                                                'preempted':'NEW_TRIAL'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})



        self.sis = smach_ros.IntrospectionServer('sis_experiment',
                                                 self.sm,
                                                 '/EXPERIMENT')

    def Run(self):
        self.sis.start()
        try:
            outcome = self.sm.execute()
            rospy.logwarn ('Experiment returned with: %s' % outcome)
        except smach.exceptions.InvalidUserCodeError:
            pass
        
        self.sis.stop()



