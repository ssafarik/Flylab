#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import actionlib
import numpy as N
import smach
import smach_ros
import tf

from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from std_msgs.msg import Header
from stage_action_server.msg import *
from flycore.msg import MsgFrameState
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from experiments.srv import Trigger, ExperimentParams
from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from patterngen.msg import MsgPattern

gRate = 100
g_tfrx = None

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
        
    #rospy.loginfo('EL GetAngleToRobotInFlyView()=%s' % angle)
    return angle


def GetSpeedFly (arenastate, iFly):
    speed = None
    if len(arenastate.flies)-1 >= iFly:
        speed = N.linalg.norm(N.array([arenastate.flies[iFly].velocity.linear.x,
                                       arenastate.flies[iFly].velocity.linear.y,
                                       arenastate.flies[iFly].velocity.linear.z]))
        
    #rospy.loginfo ('EL GetSpeedFly()=%s' % speed)
    return speed


def GetDistanceFlyToRobot (arenastate, iFly):
    distance = None
    if len(arenastate.flies)-1 >= iFly:
        dx = arenastate.robot.pose.position.x - arenastate.flies[iFly].pose.position.x
        dy = arenastate.robot.pose.position.y - arenastate.flies[iFly].pose.position.y
        distance = N.linalg.norm([dx,dy])
        
    return distance


def ClipXyToRadius(x, y, rmax):
    r = N.sqrt(x*x + y*y)
    
    xOut,yOut = x,y
    if (rmax*0.8) < r:
        angle = N.arctan2(y, x)
        xOut = (rmax*0.8) * N.cos(angle)
        yOut = (rmax*0.8) * N.sin(angle)
        #rospy.loginfo('EL CLIPPING x,y=%s to %s' % ([x,y],[xOut,yOut]))
        
    return [xOut,yOut]


# TriggerService()
# Sends trigger commands to a list of services.
# When you call TriggerService.notify(), then for each in the list, we call the ".../trigger" services with the given "status" parameter.
#
class TriggerService():
    def __init__(self):
        self.services = {"save/arenastate/trigger": None, 
                         "save/video/trigger": None}
        
    def attach(self):
        for key in self.services:
            #rospy.logwarn('Waiting for service: %s' % key)
            rospy.wait_for_service(key)
            try:
                self.services[key] = rospy.ServiceProxy(key, Trigger)
            except rospy.ServiceException, e:
                rospy.logwarn ("FAILED %s: %s" % (key,e))
	    #rospy.logwarn('Attached service: %s' % key)
            
        
    def notify(self, status):
	#rospy.logwarn('Triggering: %s' % self.services)
        for key,callback in self.services.iteritems():
            if callback is not None:
                callback(status)
            
        

# NewTrialService()
# Sends new_trial commands to a list of services.
# When you call NewTrialService.notify(), then for each in the list, we call the ".../new_trial" services with the given "experimentparams" parameter.
#
class NewTrialService():
    def __init__(self):
        self.services = {"save/arenastate/new_trial": None, 
                         "save/video/new_trial": None}
    
    def attach(self):
        for key in self.services:
            #rospy.logwarn('Waiting for service: %s' % key)
            rospy.wait_for_service(key)
            try:
                self.services[key] = rospy.ServiceProxy(key, ExperimentParams)
            except rospy.ServiceException, e:
                rospy.logwarn ("FAILED %s: %s" % (key,e))
        
    def notify(self, experimentparams):
        for key,callback in self.services.iteritems():
            if callback is not None:
                callback(experimentparams)

    

#######################################################################################################
#######################################################################################################
class NewExperiment (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','abort'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        
        
    def execute(self, userdata):
        experimentparams = userdata.experimentparamsIn
        rospy.loginfo("EL State NewExperiment(%s)" % experimentparams)

        experimentparams.experiment.trial = experimentparams.experiment.trial-1 
        userdata.experimentparamsOut = experimentparams
        
        #rospy.loginfo ('EL Exiting NewExperiment()')
        return 'success'


#######################################################################################################
#######################################################################################################
# NewTrial() - Increments the trial number, untriggers, and calls the 
#              new_trial service (which begins recording).
#
class NewTrial (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['continue','stop','abort'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        #self.pub_experimentparams = rospy.Publisher('ExperimentParams', ExperimentParams)
        self.Trigger = TriggerService()
        self.Trigger.attach()
        
        self.NewTrial = NewTrialService()
        self.NewTrial.attach()

        
    def execute(self, userdata):
        rv = 'stop'
        experimentparams = userdata.experimentparamsIn
        experimentparams.experiment.trial = userdata.experimentparamsIn.experiment.trial+1
        if (experimentparams.experiment.maxTrials != -1):
            if (experimentparams.experiment.maxTrials < experimentparams.experiment.trial):
                return rv

        rospy.loginfo ('EL State NewTrial(%s)' % experimentparams.experiment.trial)

        self.Trigger.notify(False)
        userdata.experimentparamsOut = experimentparams
        #self.pub_experimentparams.publish(experimentparams)
        try:
            self.NewTrial.notify(experimentparams)
            rv = 'continue'
        except rospy.ServiceException:
            rv = 'abort'

        #rospy.loginfo ('EL Exiting NewTrial()')
        return rv



#######################################################################################################
#######################################################################################################
class TriggerOnStates (smach.State):
    def __init__(self, type='entry'):
        global g_tfrx
        
        self.type               = type
        
        self.isTriggered        = False
        self.timeTriggered      = None
        
        
        smach.State.__init__(self, 
                             outcomes=['success','disabled','timeout','abort'],
                             input_keys=['experimentparamsIn'])
        rospy.on_shutdown(self.OnShutdown_callback)
        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=2)
        if g_tfrx is None:
            g_tfrx = tf.TransformListener()

        self.Trigger = TriggerService()
        self.Trigger.attach()
    
        self.nRobots = rospy.get_param('nRobots', 0)
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.
        
    
    def OnShutdown_callback(self):
        pass
        

    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def GetDistanceFrameToFrame (self, frameidParent, frameidChild):
        distance = None
        try:
            stamp = g_tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = g_tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception:
            pass
        else:
            distance = N.linalg.norm([pointP.point.x, pointP.point.y, pointP.point.z])
            
        return distance


    def GetAngleFrameToFrame (self, frameidParent, frameidChild):
        angleToChild = None
        try:
            stamp = g_tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = g_tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception:
            pass
        else:
            angleToChild = N.arctan2(pointP.point.y, pointP.point.x) % (2.0*N.pi)
            
        return angleToChild


    def GetSpeedFrameToFrame (self, frameidParent, frameidChild):
        speed = None
        try:
            stamp = g_tfrx.getLatestCommonTime(frameidParent, frameidChild)
            ((vx,vy,vz),(wx,wy,wz)) = g_tfrx.lookupTwist(frameidChild, frameidParent, stamp-self.dtVelocity, self.dtVelocity)
        except (tf.Exception, AttributeError), e:
            ((vx,vy,vz),(wx,wy,wz)) = ((0,0,0),(0,0,0))

        speed = N.linalg.norm(N.array([vx,vy,vz]))
            
        return speed


    def execute(self, userdata):
        rospy.loginfo("EL State TriggerOnStates(%s)" % (self.type))

        if self.type == 'entry':
            trigger = userdata.experimentparamsIn.triggerEntry
        else:
            trigger = userdata.experimentparamsIn.triggerExit

        rv = 'disabled'
        try:
            if trigger.enabled:
                self.timeStart = rospy.Time.now()
                self.isTriggered = False
                self.timeTriggered  = None
                
                # Wait for an arenastate.
                while self.arenastate is None:
                    if trigger.timeout != -1:
                        if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > trigger.timeout:
                            return 'timeout'
                    #if self.preempt_requested():
                    #    self.Trigger.notify(False)
                    #    return 'preempt'
                    rospy.sleep(1.0)
        
        
                rv = 'abort'
                while not rospy.is_shutdown():
                    if True: #(len(self.arenastate.flies)>0) and (self.nRobots>0):
                        #iFly = GetNearestFly(self.arenastate)
                        #if iFly is None:
                        #    continue
        
                        # Test for distance.
                        isDistanceInRange = True
                        distance = None
                        if (trigger.distanceMin is not None) and (trigger.distanceMax is not None):
                            #distance = GetDistanceFlyToRobot(self.arenastate, iFly)
                            distance = self.GetDistanceFrameToFrame(trigger.frameidParent, trigger.frameidChild)
                            isDistanceInRange = False
                            if (distance is not None) and (trigger.distanceMin <= distance <= trigger.distanceMax):
                                isDistanceInRange = True
                                
                        # Test for angle.
                        isAngleInRange = True
                        if (trigger.angleMin is not None) and (trigger.angleMax is not None):
                            #angle = GetAngleToRobotInFlyView(self.arenastate, iFly)
                            angle = self.GetAngleFrameToFrame(trigger.frameidParent, trigger.frameidChild)
                            
                            angleA1 = trigger.angleMin % (2.0*N.pi)
                            angleA2 = trigger.angleMax % (2.0*N.pi)
                            angleB1 = (2.0*N.pi - angleA2) # % (2.0*N.pi)
                            angleB2 = (2.0*N.pi - angleA1) # % (2.0*N.pi)
            
                            if angle is not None:
                                # Test for angle meeting the angle criteria.
                                isAngleInRange = False
                                if trigger.angleTestBilateral:
                                    if trigger.angleTest=='inclusive':
                                        #rospy.loginfo('EL angles %s' % [angleA1, angleA2, angleB1, angleB2, angle])
                                        if (angleA1 <= angle <= angleA2) or (angleB1 <= angle <= angleB2):
                                            isAngleInRange = True
                                            
                                    elif trigger.angleTest=='exclusive':
                                        if (0.0 <= angle < angleA1) or (angleA2 < angle < angleB1) or (angleB2 < angle <= (2.0*N.pi)):
                                            isAngleInRange = True
                                else:
                                    if trigger.angleTest=='inclusive':
                                        if (angleA1 <= angle <= angleA2):
                                            isAngleInRange = True
                                            
                                    elif trigger.angleTest=='exclusive':
                                        if (angle < angleA1) or (angleA2 < angle):
                                            isAngleInRange = True
                            
                        
                        # Test for speed of parent.
                        isSpeedParentInRange = True
                        if (trigger.speedParentMin is not None) and (trigger.speedParentMax is not None):
                            isSpeedParentInRange = False
                            speedParent = self.GetSpeedFrameToFrame('Plate', trigger.frameidParent)# Absolute speed of the parent frame.
                            #rospy.loginfo ('EL speed=%s' % speed)
                            if speedParent is not None:
                                if (trigger.speedParentMin <= speedParent <= trigger.speedParentMax):
                                    isSpeedParentInRange = True
        
                        # Test for speed of child.
                        isSpeedChildInRange = True
                        if (trigger.speedChildMin is not None) and (trigger.speedChildMax is not None):
                            isSpeedChildInRange = False
                            speedChild = self.GetSpeedFrameToFrame('Plate', trigger.frameidChild)# Absolute speed of the child frame.
                            #rospy.loginfo ('EL speed=%s' % speed)
                            if speedChild is not None:
                                if (trigger.speedChildMin <= speedChild <= trigger.speedChildMax):
                                    isSpeedChildInRange = True
        
                        # Test all the trigger criteria.
                        if isDistanceInRange and isAngleInRange and isSpeedParentInRange and isSpeedChildInRange:
                            
                            # Set the pending trigger start time.
                            if not self.isTriggered:
                                self.isTriggered = True
                                self.timeTriggered = rospy.Time.now()
                        else:
                            # Cancel a pending trigger.
                            self.isTriggered = False
                            self.timeTriggered = None
        
                        if (distance is not None) and (angle is not None) and (speedParent is not None) and (speedChild is not None):
                            rospy.loginfo ('EL triggers=distance=%0.3f, speed=%0.3f,%0.3f, angle=%0.3f, bools=%s' % (distance, speedParent, speedChild, angle, [isDistanceInRange, isSpeedParentInRange, isSpeedChildInRange, isAngleInRange]))
                        else:
                            rospy.loginfo ('EL triggers=distance=%s, speed=%s,%s, angle=%s, bools=%s' % (distance, speedParent, speedChild, angle, [isDistanceInRange, isSpeedParentInRange, isSpeedChildInRange, isAngleInRange]))
        
                        # If pending trigger has lasted longer than requested duration, then set trigger.
                        if (self.isTriggered):
                            duration = rospy.Time.now().to_sec() - self.timeTriggered.to_sec()
                            
                            if duration >= trigger.timeHold:
                                rv = 'success'
                                break
                            
                            #rospy.loginfo('EL duration=%s' % duration)
                            
                        
                    #if self.preempt_requested():
                    #    rv = 'preempt'
                    #    break
                    
                    if trigger.timeout != -1:
                        if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > trigger.timeout:
                            rv = 'timeout'
                            break
                    
                    self.rosrate.sleep()

            #rospy.logwarn ('rv=%s', rv)
            #rospy.logwarn ('self.type=%s', self.type)
            if rv!='abort' and self.type=='entry':
                self.Trigger.notify(True)
            #else:
            #    self.Trigger.notify(False)
                
        except rospy.ServiceException:
            rv = 'abort'
            
        #rospy.loginfo ('EL Exiting TriggerOnStates()')
        return rv

    

#######################################################################################################
#######################################################################################################
class TriggerOnTime (smach.State):
    def __init__(self, type='entry'):
        self.type = type
        smach.State.__init__(self, 
                             outcomes=['success','abort'],
                             input_keys=['experimentparamsIn'])

        self.Trigger = TriggerService()
        self.Trigger.attach()
        

    def execute(self, userdata):
        rospy.loginfo("EL State TriggerOnTime(%s)" % (userdata.experimentparamsIn.waitEntry))
        rv = 'success'
        try:
            rospy.sleep(userdata.experimentparamsIn.waitEntry)
        except rospy.ServiceException:
            rv = 'abort'

        if rv!='abort' and self.type=='entry':
            self.Trigger.notify(True)
        #else:
        #    self.Trigger.notify(False)

        #rospy.loginfo ('EL Exiting TriggerOnTime()')
        return rv



#######################################################################################################
#######################################################################################################
class ResetRobot (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['success','disabled','timeout','abort'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=2)

        self.action = actionlib.SimpleActionClient('StageActionServer', ActionStageStateAction)
        self.action.wait_for_server()
        self.goal = ActionStageStateGoal()
        self.set_stage_state = None
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo("EL State ResetRobot(%s)" % [userdata.experimentparamsIn.home.enabled, userdata.experimentparamsIn.home.x, userdata.experimentparamsIn.home.y])

        rv = 'disabled'
        if userdata.experimentparamsIn.home.enabled:
            self.timeStart = rospy.Time.now()
    
            rospy.wait_for_service('set_stage_state')
            try:
                self.set_stage_state = rospy.ServiceProxy('set_stage_state', SrvFrameState)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            

            while self.arenastate is None:
                if userdata.experimentparamsIn.home.timeout != -1:
                    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsIn.home.timeout:
                        return 'timeout'
                #if self.preempt_requested():
                #    return 'preempt'
                rospy.sleep(1.0)
    
    
    
            # Send the command.
            self.goal.state.header = self.arenastate.robot.header
            #self.goal.state.header.stamp = rospy.Time.now()
            self.goal.state.pose.position.x = userdata.experimentparamsIn.home.x
            self.goal.state.pose.position.y = userdata.experimentparamsIn.home.y
            self.set_stage_state(SrvFrameStateRequest(state=MsgFrameState(header=self.goal.state.header, 
                                                                          pose=self.goal.state.pose),
                                                      speed = userdata.experimentparamsIn.home.speed))

            rv = 'abort'
            while not rospy.is_shutdown():
                # Are we there yet?
                
                ptRobot = N.array([self.arenastate.robot.pose.position.x, 
                                   self.arenastate.robot.pose.position.y])
                ptTarget = N.array([self.goal.state.pose.position.x,
                                    self.goal.state.pose.position.y])                
                r = N.linalg.norm(ptRobot-ptTarget)
                rospy.loginfo ('EL ResetRobot() ptTarget=%s, ptRobot=%s, r=%s' % (ptTarget, ptRobot, r))
                
                
                if (r <= userdata.experimentparamsIn.home.tolerance):
                    rospy.sleep(0.5) # Allow some settling time.
                    rv = 'success'
                    break
                
                
                #if self.preempt_requested():
                #    rv = 'preempt'
                #    break
                
                
                if userdata.experimentparamsIn.home.timeout != -1:
                    if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.home.timeout:
                        rv = 'timeout'
                        break
                
                self.rosrate.sleep()


        #rospy.loginfo ('EL Exiting ResetRobot()')
        return rv
        


#######################################################################################################
#######################################################################################################
class MoveRobot (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['success','disabled','timeout','preempt','abort'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=2)
        self.pubPatternGen = rospy.Publisher('SetSignalGen', MsgPattern, latch=True)

        self.action = actionlib.SimpleActionClient('StageActionServer', ActionStageStateAction)
        self.action.wait_for_server()
        self.goal = ActionStageStateGoal()
        self.set_stage_state = None

        self.radiusMovement = float(rospy.get_param("arena/radius_inner","25.4"))
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate



    def execute(self, userdata):
        rospy.loginfo("EL State MoveRobot(%s)" % [userdata.experimentparamsIn.move.mode,
                                                  userdata.experimentparamsIn.move.relative.distance, 
                                                  userdata.experimentparamsIn.move.relative.angle, 
                                                  userdata.experimentparamsIn.move.relative.speed])

        rv = 'disabled'
        if userdata.experimentparamsIn.move.enabled:
            rospy.wait_for_service('set_stage_state')
            try:
                self.set_stage_state = rospy.ServiceProxy('set_stage_state', SrvFrameState)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            self.timeStart = rospy.Time.now()
    
            while self.arenastate is None:
                if userdata.experimentparamsIn.move.timeout != -1:
                    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsIn.move.timeout:
                        return 'timeout'
                
                if self.preempt_requested():
                    return 'preempt'
                
                rospy.sleep(1.0)
    
    
            if userdata.experimentparamsIn.move.mode=='relative':
                rv = self.MoveRelative(userdata)
            else:
                rv = self.MovePattern(userdata)
                
        #rospy.loginfo ('EL Exiting MoveRobot()')
        return rv
            
            
    def MoveRelative (self, userdata):            
        self.ptTarget = None
        rv = 'abort'

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
                doMove = True
            elif userdata.experimentparamsIn.move.relative.frameidOriginPosition=="Robot":
                posOrigin = posRobot
                doMove = True
            else:
                posOrigin = Point(x=0, y=0, z=0) # Relative to the origin of the Plate frame
                doMove = False
            
            #rospy.loginfo('EL len(ArenaState)=%d, posOrigin=[%0.2f,%0.2f]'%(len(self.arenastate.flies),posOrigin.x,posOrigin.y))

            # If we need to calculate a target.
            if (self.ptTarget is None) or (userdata.experimentparamsIn.move.relative.tracking):
                # Compute target point in workspace (i.e. Plate) coordinates.
                ptOrigin = N.array([posOrigin.x, posOrigin.y])
                d = userdata.experimentparamsIn.move.relative.distance
                ptRelative = d * N.array([N.cos(angle), N.sin(angle)])
                ptTarget = ptOrigin + ptRelative
                self.ptTarget = ClipXyToRadius(ptTarget[0], ptTarget[1], self.radiusMovement)
                #self.ptTarget = ptTarget

                #rospy.loginfo ('EL self.ptTarget=%s, ptOrigin=%s, ptRelative=%s, angle=%s, frameAngle=%s' % (self.ptTarget, ptOrigin, ptRelative, angle,userdata.experimentparamsIn.move.relative.frameidOriginAngle))
                #rospy.loginfo('EL Robot/Fly frame_id=%s' % [self.arenastate.robot.header.frame_id,self.arenastate.flies[iFly].header.frame_id])

                # Send the command.
                #self.goal.state.header = self.arenastate.flies[iFly].header
                self.goal.state.header = self.arenastate.robot.header
                #self.goal.state.header.stamp = rospy.Time.now()
                self.goal.state.pose.position.x = self.ptTarget[0]
                self.goal.state.pose.position.y = self.ptTarget[1]
                
                
                
                try:
                    if (doMove):
                        #rospy.loginfo('EL set_stage_state(%s)' % [self.goal.state.pose.position.x,self.goal.state.pose.position.y])
                        self.set_stage_state(SrvFrameStateRequest(state=MsgFrameState(header=self.goal.state.header, 
                                                                                      pose=self.goal.state.pose),
                                                                  speed = speedTarget))
                except (rospy.ServiceException, rospy.exceptions.ROSInterruptException), e:
                    rospy.logwarn ('EL Exception calling set_stage_state(): %s' % e)
                    self.ptTarget = None
                #rospy.loginfo('EL calling set_stage_state(%s) post' % [self.goal.state.pose.position.x,self.goal.state.pose.position.y])


            # If no flies and no target, then abort.
            #if (len(self.arenastate.flies)==0) and (self.ptTarget is None):
            #    rv = 'abort'
            #    rospy.logwarn ('No flies and no target.')
            #    break

                    
            # Check if we're there yet.
            if self.ptTarget is not None:
                r = N.linalg.norm(ptRobot-self.ptTarget)
                #rospy.loginfo ('EL ptTarget=%s, ptRobot=%s, r=%s' % (self.ptTarget, ptRobot, r))
                if (r <= userdata.experimentparamsIn.move.relative.tolerance):
                    rv = 'success'
                    break

            
            if self.preempt_requested():
                rv = 'preempt'
                break

            
            if userdata.experimentparamsIn.move.timeout != -1:
                if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > userdata.experimentparamsIn.move.timeout:
                    rv = 'timeout'
                    break
            
            self.rosrate.sleep()


        self.ptTarget = None
        
        return rv

        
    def MovePattern (self, userdata):
                    
        msgPattern = MsgPattern()

        # Publish the pattern message.
        msgPattern.mode = 'byshape'
        msgPattern.shape = userdata.experimentparamsIn.move.pattern.shape
        msgPattern.points = []
        msgPattern.frame_id = 'Plate'
        msgPattern.hzPattern = userdata.experimentparamsIn.move.pattern.hzPattern
        msgPattern.hzPoint = userdata.experimentparamsIn.move.pattern.hzPoint
        msgPattern.count = userdata.experimentparamsIn.move.pattern.count
        msgPattern.size = userdata.experimentparamsIn.move.pattern.size
        msgPattern.preempt = True
        msgPattern.param = 0.0#userdata.experimentparamsIn.move.pattern.param
        self.pubPatternGen.publish (msgPattern)
                

        rv = 'abort'
        while not rospy.is_shutdown():
            if self.preempt_requested():
                rv = 'preempt'
                break

            
            if userdata.experimentparamsIn.move.timeout != -1:
                if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsIn.move.timeout:
                    rv = 'timeout'
                    break
            
            self.rosrate.sleep()

        # Turn off the pattern
        msgPattern.mode = 'byshape'
        msgPattern.shape = userdata.experimentparamsIn.move.pattern.shape
        msgPattern.points = []
        msgPattern.frame_id = 'Plate'
        msgPattern.hzPattern = userdata.experimentparamsIn.move.pattern.hzPattern
        msgPattern.hzPoint = userdata.experimentparamsIn.move.pattern.hzPoint
        msgPattern.count = 0
        msgPattern.size = userdata.experimentparamsIn.move.pattern.size
        msgPattern.preempt = True
        msgPattern.param = 0.0#userdata.experimentparamsIn.move.pattern.param
        self.pubPatternGen.publish (msgPattern)

        return rv
        


#######################################################################################################
#######################################################################################################
# Lasertrack()
# 
# Control the laser according to the experimentparams.  Turn off when done.
#
class Lasertrack (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['disabled','timeout','preempt','abort'],
                             input_keys=['experimentparamsIn'])

        self.rosrate = rospy.Rate(gRate)
        self.pubGalvoCommand = rospy.Publisher('GalvoDirector/command', MsgGalvoCommand, latch=True)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass
        
        
    def execute(self, userdata):
        for pattern in userdata.experimentparamsIn.lasertrack.pattern_list:
            rospy.loginfo("EL State Lasertrack(%s)" % pattern)

        rv = 'disabled'
        if userdata.experimentparamsIn.lasertrack.enabled:
            self.timeStart = rospy.Time.now()
    
            # Send the tracking command to the galvo director.
            command = MsgGalvoCommand()
            command.enable_laser = True
            command.pattern_list = userdata.experimentparamsIn.lasertrack.pattern_list
            command.units = 'millimeters' # 'millimeters' or 'volts'
            self.pubGalvoCommand.publish(command)
    
            # Move galvos until preempt or timeout.        
            while not rospy.is_shutdown():
                if self.preempt_requested():
                    self.recall_preempt()
                    rv = 'preempt'
                    break
    
                if userdata.experimentparamsIn.lasertrack.timeout != -1:
                    if (rospy.Time.now().to_sec()-self.timeStart.to_sec()) > userdata.experimentparamsIn.lasertrack.timeout:
                        rv = 'timeout'
                        break
                
                self.rosrate.sleep()

                
        # Turn off the laser.
        command = MsgGalvoCommand()
        command.enable_laser = False
        self.pubGalvoCommand.publish(command)
        
                
        return rv

    

            
            
#######################################################################################################
#######################################################################################################
class Experiment():
    def __init__(self, experimentparams=None):
        self.xHome = 0
        self.yHome = 0

        self.Trigger = TriggerService()
        self.Trigger.attach()
        
        # Create the state machine.
        self.stateTop = smach.StateMachine(['success','abort'])
        self.stateTop.userdata.experimentparams = experimentparams
        
        # Create the "action" concurrency state.
        stateActions = smach.Concurrence(outcomes=['success','disabled','abort'],
                                         default_outcome='abort',
                                         child_termination_cb=self.child_term_callback,
                                         outcome_cb=self.all_term_callback,
                                         input_keys=['experimentparamsIn'])

        with stateActions:
            smach.Concurrence.add('MOVEROBOT', 
                                  MoveRobot ())
            smach.Concurrence.add('LASERTRACK', 
                                  Lasertrack ())
            smach.Concurrence.add('EXITTRIGGER', 
                                  TriggerOnStates(type='exit'))

        
        with self.stateTop:
            smach.StateMachine.add('NEWEXPERIMENT',
                                   NewExperiment(),
                                   transitions={'success':'RESETHARDWARE',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('RESETHARDWARE',
                                   ResetRobot(),
                                   transitions={'success':'NEWTRIAL',
                                                'disabled':'NEWTRIAL',
                                                'timeout':'NEWTRIAL',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams'})

            smach.StateMachine.add('NEWTRIAL',
                                   NewTrial(),
                                   transitions={'continue':'ENTRYWAIT',
                                                'stop':'success',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})

            smach.StateMachine.add('ENTRYWAIT', 
                                   TriggerOnTime(type='none'),
                                   transitions={'success':'ENTRYTRIGGER',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams'})

            smach.StateMachine.add('ENTRYTRIGGER', 
                                   TriggerOnStates(type='entry'),
                                   transitions={'success':'ACTIONS',
                                                'disabled':'ACTIONS',
                                                'timeout':'ACTIONS',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams'})

            smach.StateMachine.add('ACTIONS', stateActions,
                                   transitions={'success':'RESETHARDWARE',
                                                'disabled':'RESETHARDWARE',
                                                'abort':'abort'},
                                   remapping={'experimentparamsIn':'experimentparams'})


        self.sis = smach_ros.IntrospectionServer('sis_experiment',
                                                 self.stateTop,
                                                 '/EXPERIMENT')
        
    # Gets called upon ANY child state termination.
    def child_term_callback(self, outcome_map):
        rv = False

        if ('EXITTRIGGER' in outcome_map) and ('MOVEROBOT' in outcome_map) and ('LASERTRACK' in outcome_map):
            # If any of the states either succeeds or times-out, then we're done (i.e. preempt the other states).
            if (outcome_map['EXITTRIGGER']=='timeout') or (outcome_map['EXITTRIGGER']=='success'):
                rv = True 
            
            if (outcome_map['MOVEROBOT']=='timeout') or (outcome_map['MOVEROBOT']=='success'):
                rv = True 
            
            if (outcome_map['LASERTRACK']=='timeout') or (outcome_map['LASERTRACK']=='success'):
                rv = True 
        
            
        #rospy.logwarn('child_term_callback outcome_map=%s, rv=%s' % (outcome_map,rv))
        # True:  Preempt remaining states.
        # False: Let remaining states keep going.
        return rv
    

    # Gets called after all child states are terminated.
    def all_term_callback(self, outcome_map):
        rv = 'abort'
        if ('EXITTRIGGER' in outcome_map) and ('MOVEROBOT' in outcome_map) and ('LASERTRACK' in outcome_map):
            if (outcome_map['EXITTRIGGER']=='timeout') or \
               (outcome_map['EXITTRIGGER']=='success') or \
               (outcome_map['MOVEROBOT']=='timeout') or \
               (outcome_map['MOVEROBOT']=='success') or \
               (outcome_map['LASERTRACK']=='timeout') or \
               (outcome_map['LASERTRACK']=='success'):
                rv = 'success'
                
            elif (outcome_map['EXITTRIGGER']=='preempt') and (outcome_map['MOVEROBOT']=='preempt') and (outcome_map['LASERTRACK']=='preempt'):
                rv = 'preempt'
                rospy.logwarn ('All experiment actions preempted.')
                 
            elif (outcome_map['EXITTRIGGER']=='disabled') and (outcome_map['MOVEROBOT']=='disabled') and (outcome_map['LASERTRACK']=='disabled'):
                rv = 'disabled'
                rospy.logwarn ('All experiment actions set as disabled.')
                
            else: # Some preempted, some disabled.
                rv = 'disabled'
                rospy.logwarn ('Unexpected state outcomes.  Please check this.') 


        self.Trigger.notify(False)

        return rv

        

    def Run(self):
        self.sis.start()
        try:
            outcome = self.stateTop.execute()
        except smach.exceptions.InvalidUserCodeError, e:
            rospy.logwarn('Exception InvalidUserCodeError executing state machine: %s' % e)
        
        rospy.logwarn ('Experiment state machine finished with outcome=%s' % outcome)
        self.sis.stop()



