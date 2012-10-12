#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import actionlib
import copy
import numpy as N
import smach
import smach_ros
import tf

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from stage_action_server.msg import *
from pythonmodules import filters
from flycore.msg import MsgFrameState, TrackingCommand
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from experiments.srv import Trigger, ExperimentParams
from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from patterngen.msg import MsgPattern
from visualization_msgs.msg import Marker

gRate = 100  # This is the loop rate at which the experiment states run.
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
        self.services = {"save/trigger": None}
        
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
        self.services = {"save/new_trial": None}
    
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
                             outcomes=['success','aborted'],
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
# Experiment may be paused & restarted via the commandlines:
# rostopic pub -1 experiment/command std_msgs/String pause
# rostopic pub -1 experiment/command std_msgs/String run
#
class NewTrial (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['continue','stop','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        self.pubTrackingCommand = rospy.Publisher('TrackingCommand', TrackingCommand, latch=True)
        
        # Command messages.
        self.command = 'continue'
        self.command_list = ['continue','pause', 'stage/calibrate', 'exit']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.Command_callback)

        
        self.Trigger = TriggerService()
        self.Trigger.attach()
        
        self.NewTrial = NewTrialService()
        self.NewTrial.attach()

    def Command_callback(self, msgString):
        if msgString.data in self.command_list:
            self.command = msgString.data
            rospy.logwarn ('Experiment command received: %s' % self.command)
        else:
            rospy.logwarn ('Unknown experiment command: %s, valid commands are %s' % (msgString.data, self.command_list))
            
        
    def execute(self, userdata):
        rv = 'stop'
        experimentparams = userdata.experimentparamsIn
        experimentparams.experiment.trial = userdata.experimentparamsIn.experiment.trial+1
        if (experimentparams.experiment.maxTrials != -1):
            if (experimentparams.experiment.maxTrials < experimentparams.experiment.trial):
                return rv

        rospy.loginfo ('EL State NewTrial(%s)' % experimentparams.experiment.trial)

        self.Trigger.notify(False)
        
        # Handle various commands sent in via messages.
        if (self.command=='pause'):
            rospy.logwarn ('**************************************** Experiment paused at NewTrial...')
            while (self.command != 'continue'):
                rospy.sleep(1)
            rospy.logwarn ('**************************************** Experiment continuing.')

        if (self.command=='stage/calibrate'):
            rospy.logwarn ('**************************************** Calibrating...')
            rospy.wait_for_service('calibrate_stage')
            try:
                Calibrate = rospy.ServiceProxy('calibrate_stage', SrvFrameState)
            except rospy.ServiceException, e:
                rospy.logwarn ("FAILED to attach to calibrate_stage service: %s" % e)
            else:
                Calibrate(SrvFrameStateRequest())
            self.command = 'continue'
            rospy.logwarn ('**************************************** Experiment continuing.')
            
        if (self.command=='exit'):
            return rv

            
        userdata.experimentparamsOut = experimentparams
        self.pubTrackingCommand.publish(experimentparams.tracking)
        try:
            self.NewTrial.notify(experimentparams)
            rv = 'continue'
        except rospy.ServiceException:
            rv = 'aborted'

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
                             outcomes=['success','disabled','timeout','aborted'],
                             input_keys=['experimentparamsIn'])
        rospy.on_shutdown(self.OnShutdown_callback)
        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
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
                    #    self.recall_preempt()
                    #    self.Trigger.notify(False)
                    #    return 'preempt'
                    rospy.sleep(1.0)
        
        
                rv = 'aborted'
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
                            
                        
                        # Test for absolute speed of parent.
                        isSpeedAbsParentInRange = True
                        if (trigger.speedAbsParentMin is not None) and (trigger.speedAbsParentMax is not None):
                            isSpeedAbsParentInRange = False
                            speedAbsParent = self.GetSpeedFrameToFrame('Plate', trigger.frameidParent)# Absolute speed of the parent frame.
                            #rospy.loginfo ('EL speed=%s' % speed)
                            if speedAbsParent is not None:
                                if (trigger.speedAbsParentMin <= speedAbsParent <= trigger.speedAbsParentMax):
                                    isSpeedAbsParentInRange = True
        
                        # Test for absolute speed of child.
                        isSpeedAbsChildInRange = True
                        if (trigger.speedAbsChildMin is not None) and (trigger.speedAbsChildMax is not None):
                            isSpeedAbsChildInRange = False
                            speedAbsChild = self.GetSpeedFrameToFrame('Plate', trigger.frameidChild)# Absolute speed of the child frame.
                            #rospy.loginfo ('EL speed=%s' % speed)
                            if speedAbsChild is not None:
                                if (trigger.speedAbsChildMin <= speedAbsChild <= trigger.speedAbsChildMax):
                                    isSpeedAbsChildInRange = True
        
                        # Test for relative speed between parent & child.
                        isSpeedRelInRange = True
                        if (trigger.speedRelMin is not None) and (trigger.speedRelMax is not None):
                            isSpeedRelInRange = False
                            speedRel = self.GetSpeedFrameToFrame(trigger.frameidParent, trigger.frameidChild)# Relative speed parent to child.
                            #rospy.loginfo ('EL speed=%s' % speed)
                            if speedRel is not None:
                                if (trigger.speedRelMin <= speedRel <= trigger.speedRelMax):
                                    isSpeedRelInRange = True
        
                        # Test all the trigger criteria.
                        if isDistanceInRange and isAngleInRange and isSpeedAbsParentInRange and isSpeedAbsChildInRange and isSpeedRelInRange:
                            
                            # Set the pending trigger start time.
                            if not self.isTriggered:
                                self.isTriggered = True
                                self.timeTriggered = rospy.Time.now()
                        else:
                            # Cancel a pending trigger.
                            self.isTriggered = False
                            self.timeTriggered = None
        
                        #if (distance is not None) and (angle is not None) and (speedAbsParent is not None) and (speedAbsChild is not None) and (speedRel is not None):
                        #    rospy.loginfo ('EL triggers=distance=%0.3f, speed=%0.3f,%0.3f, angle=%0.3f, bools=%s' % (distance, speedAbsParent, speedAbsChild, angle, [isDistanceInRange, isSpeedAbsParentInRange, isSpeedAbsChildInRange, isSpeedRelInRange, isAngleInRange]))
                        #else:
                        #    rospy.loginfo ('EL triggers=distance=%s, speed=%s,%s, angle=%s, bools=%s' % (distance, speedAbsParent, speedAbsChild, angle, [isDistanceInRange, isSpeedAbsParentInRange, isSpeedAbsChildInRange, isSpeecRelInRange, isAngleInRange]))
        
                        # If pending trigger has lasted longer than requested duration, then set trigger.
                        if (self.isTriggered):
                            duration = rospy.Time.now().to_sec() - self.timeTriggered.to_sec()
                            
                            if duration >= trigger.timeHold:
                                rv = 'success'
                                break
                            
                            #rospy.loginfo('EL duration=%s' % duration)
                            
                        
                    #if self.preempt_requested():
                    #    self.recall_preempt()
                    #    rv = 'preempt'
                    #    break
                    
                    if trigger.timeout != -1:
                        if (rospy.Time.now().to_sec() - self.timeStart.to_sec()) > trigger.timeout:
                            rv = 'timeout'
                            break
                    
                    self.rosrate.sleep()

            #rospy.logwarn ('rv=%s', rv)
            #rospy.logwarn ('self.type=%s', self.type)
            if rv!='aborted' and self.type=='entry':
                self.Trigger.notify(True)
            #else:
            #    self.Trigger.notify(False)
                
        except rospy.ServiceException:
            rv = 'aborted'
            
        #rospy.loginfo ('EL Exiting TriggerOnStates()')
        return rv

    

#######################################################################################################
#######################################################################################################
class TriggerOnTime (smach.State):
    def __init__(self, type='entry'):
        self.type = type
        smach.State.__init__(self, 
                             outcomes=['success','aborted'],
                             input_keys=['experimentparamsIn'])

        self.Trigger = TriggerService()
        self.Trigger.attach()
        

    def execute(self, userdata):
        if self.type=='entry':
            duration = userdata.experimentparamsIn.waitEntry
        else:
            duration = userdata.experimentparamsIn.waitExit

        rospy.loginfo("EL State TriggerOnTime(%s, %s)" % (self.type, duration))
            
        rv = 'success'
        try:
            rospy.sleep(duration)
        except rospy.ServiceException:
            rv = 'aborted'

        if rv!='aborted' and self.type=='entry':
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
                             outcomes=['success','disabled','timeout','aborted'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

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
                #    self.recall_preempt()
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

            rv = 'aborted'
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
                #    self.recall_preempt()
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
                             outcomes=['success','disabled','timeout','preempt','aborted'],
                             input_keys=['experimentparamsIn'])

        self.arenastate = None
        self.rosrate = rospy.Rate(gRate)

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
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
                    self.recall_preempt()
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
            #    rv = 'aborted'
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
                self.recall_preempt()
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
                

        rv = 'aborted'
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.recall_preempt()
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
# Control the laser according to the experimentparams.  Turns off when done.
# This state allows enabling the laser only when the given object's state (i.e. Fly state) is 
# in a restricted domain of states.
#
class Lasertrack (smach.State):
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['disabled','timeout','preempt','aborted'],
                             input_keys=['experimentparamsIn'])
        
        self.rosrate = rospy.Rate(gRate)
        self.arenastate = None
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.

        self.pubMarker          = rospy.Publisher('visualization_marker', Marker)
        self.pubGalvoCommand    = rospy.Publisher('GalvoDirector/command', MsgGalvoCommand, latch=True)
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState      = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        rospy.on_shutdown(self.OnShutdown_callback)
        
        
    def OnShutdown_callback(self):
        pass


    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate
        
    
    # PublishStatefilterMarkers()
    #  
    def PublishStatefilterMarkers(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                positionLo_dict = poseLo_dict['position']
                positionHi_dict = poseHi_dict['position']
                (xLo,xHi) = (-9999,+9999)
                (yLo,yHi) = (-9999,+9999)
                (zLo,zHi) = (0,0.1)
                if ('x' in positionLo_dict):
                    xLo = positionLo_dict['x'] 
                    xHi = positionHi_dict['x']
                if ('y' in positionLo_dict):
                    yLo = positionLo_dict['y'] 
                    yHi = positionHi_dict['y']
                if ('z' in positionLo_dict):
                    zLo = positionLo_dict['z'] 
                    zHi = positionHi_dict['z']

                if ('x' in positionLo_dict) or ('y' in positionLo_dict) or ('z' in positionLo_dict):
                    markerTarget = Marker(header=Header(stamp = state.header.stamp,
                                                        frame_id='Plate'),
                                          ns='statefilter',
                                          id=1,
                                          type=Marker.CUBE,
                                          action=0,
#                                          type=Marker.LINE_STRIP
#                                          points=[Point(x=xLo,y=yLo,z=zLo),Point(x=xHi,y=yLo,z=zLo),Point(x=xHi,y=yHi,z=zLo),Point(x=xLo,y=yHi,z=zLo),Point(x=xLo,y=yLo,z=zLo)],
#                                          scale=Vector3(x=0.2, y=0.0, z=0.0),
                                          pose=Pose(position=Point(x=(xLo+xHi)/2, 
                                                                   y=(yLo+yHi)/2, 
                                                                   z=(zLo+zHi)/2)),
                                          scale=Vector3(x=xHi-xLo,
                                                        y=yHi-yLo,
                                                        z=zHi-zLo),
                                          color=ColorRGBA(a=0.1,
                                                          r=1.0,
                                                          g=1.0,
                                                          b=1.0),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)
        
        
    
    # InStatefilterRange()
    # Check if the given state falls in the given region.
    # For statefilterCriteria=="inclusive", if the given state is within all the terms, then the filter returns True.
    # For statefilterCriteria=="exclusive", if the given state is within all the terms, then the filter returns False.
    #
    # We have to manually go through each of the entries in the dict, rather than
    # using a MsgFrameState, since we need the dict to only contain the entries
    # we care about, and the MsgFrameState always contains them all.
    #  
    def InStatefilterRange(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        rv = True
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                 positionLo_dict = poseLo_dict['position']
                 positionHi_dict = poseHi_dict['position']
                 if 'x' in positionLo_dict:
                     xLo = positionLo_dict['x'] 
                     xHi = positionHi_dict['x']
                     if (state.pose.position.x < xLo) or (xHi < state.pose.position.x):
                         rv = False   
                 if 'y' in positionLo_dict:
                     yLo = positionLo_dict['y'] 
                     yHi = positionHi_dict['y']
                     if (state.pose.position.y < yLo) or (yHi < state.pose.position.y):
                         rv = False   
                 if 'z' in positionLo_dict:
                     zLo = positionLo_dict['z'] 
                     zHi = positionHi_dict['z']
                     if (state.pose.position.z < zLo) or (zHi < state.pose.position.z):
                         rv = False   
                         
            if 'orientation' in poseLo_dict:
                 orientationLo_dict = poseLo_dict['orientation']
                 orientationHi_dict = poseHi_dict['orientation']
                 if 'x' in orientationLo_dict:
                     xLo = orientationLo_dict['x'] 
                     xHi = orientationHi_dict['x']
                     if (state.pose.orientation.x < xLo) or (xHi < state.pose.orientation.x):
                         rv = False   
                 if 'y' in orientationLo_dict:
                     yLo = orientationLo_dict['y'] 
                     yHi = orientationHi_dict['y']
                     if (state.pose.orientation.y < yLo) or (yHi < state.pose.orientation.y):
                         rv = False   
                 if 'z' in orientationLo_dict:
                     zLo = orientationLo_dict['z'] 
                     zHi = orientationHi_dict['z']
                     if (state.pose.orientation.z < zLo) or (zHi < state.pose.orientation.z):
                         rv = False   
                 if 'w' in orientationLo_dict:
                     wLo = orientationLo_dict['w'] 
                     wHi = orientationHi_dict['w']
                     if (state.pose.orientation.w < wLo) or (wHi < state.pose.orientation.w):
                         rv = False   

        if 'velocity' in statefilterLo_dict:
            velocityLo_dict = statefilterLo_dict['velocity'] 
            velocityHi_dict = statefilterHi_dict['velocity']
             
            if 'linear' in velocityLo_dict:
                 linearLo_dict = velocityLo_dict['linear']
                 linearHi_dict = velocityHi_dict['linear']
                 if 'x' in linearLo_dict:
                     xLo = linearLo_dict['x'] 
                     xHi = linearHi_dict['x']
                     if (state.velocity.linear.x < xLo) or (xHi < state.velocity.linear.x):
                         rv = False   
                 if 'y' in linearLo_dict:
                     yLo = linearLo_dict['y'] 
                     yHi = linearHi_dict['y']
                     if (state.velocity.linear.y < yLo) or (yHi < state.velocity.linear.y):
                         rv = False   
                 if 'z' in linearLo_dict:
                     zLo = linearLo_dict['z'] 
                     zHi = linearHi_dict['z']
                     if (state.velocity.linear.z < zLo) or (zHi < state.velocity.linear.z):
                         rv = False   
                         
            if 'angular' in velocityLo_dict:
                 angularLo_dict = velocityLo_dict['angular']
                 angularHi_dict = velocityHi_dict['angular']
                 if 'x' in angularLo_dict:
                     xLo = angularLo_dict['x'] 
                     xHi = angularHi_dict['x']
                     if (state.velocity.angular.x < xLo) or (xHi < state.velocity.angular.x):
                         rv = False   
                 if 'y' in angularLo_dict:
                     yLo = angularLo_dict['y'] 
                     yHi = angularHi_dict['y']
                     if (state.velocity.angular.y < yLo) or (yHi < state.velocity.angular.y):
                         rv = False   
                 if 'z' in angularLo_dict:
                     zLo = angularLo_dict['z'] 
                     zHi = angularHi_dict['z']
                     if (state.velocity.angular.z < zLo) or (zHi < state.velocity.angular.z):
                         rv = False
                     
        if 'speed' in statefilterLo_dict:
            speedLo = statefilterLo_dict['speed'] 
            speedHi = statefilterHi_dict['speed']
            #speed = N.linalg.norm([state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z])
            if (state.speed < speedLo) or (speedHi < state.speed):
                rv = False   

        if (statefilterCriteria=="exclusive"):
            rv = not rv
            
        return rv
    
    
        
    def execute(self, userdata):
        for pattern in userdata.experimentparamsIn.lasertrack.pattern_list:
            rospy.loginfo("EL State Lasertrack(%s)" % pattern)

        rv = 'disabled'
        if userdata.experimentparamsIn.lasertrack.enabled:
            self.timeStart = rospy.Time.now()
    
            # Create the tracking command for the galvo director.
            command = MsgGalvoCommand()
            command.enable_laser = True
            command.units = 'millimeters' # 'millimeters' or 'volts'
            command.pattern_list = userdata.experimentparamsIn.lasertrack.pattern_list

            # Determine if we're showing patterns only for certain states.            
            nPatterns = len(userdata.experimentparamsIn.lasertrack.pattern_list)
            if len(userdata.experimentparamsIn.lasertrack.statefilterLo_list) == nPatterns and \
               len(userdata.experimentparamsIn.lasertrack.statefilterHi_list) == nPatterns:
                isStatefiltered = True
            else:
                isStatefiltered = False

            # Initialize statefilter vars.                
            bInStatefilterRangePrev = [False for i in range(nPatterns)]
            bInStatefilterRange     = [False for i in range(nPatterns)]

                             
            # If unfiltered, publish the command.
            if not isStatefiltered:
                self.pubGalvoCommand.publish(command)
    
            # Move galvos until preempt or timeout.        
            while not rospy.is_shutdown():
                
                # If state is used to determine when pattern is shown.
                if isStatefiltered:
                    # Check if any filterstates have changed.
                    bFilterStateChanged = False
                    for iPattern in range(nPatterns):
                        # Get the pattern.
                        pattern = userdata.experimentparamsIn.lasertrack.pattern_list[iPattern]

                        # Convert strings to dicts.
                        statefilterLo_dict = eval(userdata.experimentparamsIn.lasertrack.statefilterLo_list[iPattern])
                        statefilterHi_dict = eval(userdata.experimentparamsIn.lasertrack.statefilterHi_list[iPattern])
                        statefilterCriteria = userdata.experimentparamsIn.lasertrack.statefilterCriteria_list[iPattern]
                
                        # If possible, take the pose &/or velocity &/or speed from arenastate, else use transform via ROS.
                        pose = None
                        velocity = None     
                        speed = None   
                        if self.arenastate is not None:
                            if ('Robot' in pattern.frame_id):
                                #pose = self.arenastate.robot.pose  # For consistency w/ galvodirector, we'll get pose via transform.
                                velocity = self.arenastate.robot.velocity
                                speed = self.arenastate.robot.speed
                                
                            elif ('Fly' in pattern.frame_id):
                                for iFly in range(len(self.arenastate.flies)):
                                    if pattern.frame_id == self.arenastate.flies[iFly].name:
                                        #pose = self.arenastate.flies[iFly].pose  # For consistency w/ galvodirector, we'll get pose via transform.
                                        velocity = self.arenastate.flies[iFly].velocity
                                        speed = self.arenastate.flies[iFly].speed
                                        break

                        # Get the timestamp for transforms.
                        stamp=None
                        if (pose is None) or (velocity is None):
                            try:
                                stamp = g_tfrx.getLatestCommonTime('Plate', pattern.frame_id)
                            except tf.Exception:
                                pass

                            
                        # If we still need the pose (i.e. the frame wasn't in arenastate), then get it from ROS.
                        if (pose is None) and (stamp is not None) and g_tfrx.canTransform('Plate', pattern.frame_id, stamp):
                            try:
                                poseStamped = g_tfrx.transformPose('Plate', PoseStamped(header=Header(stamp=stamp,
                                                                                                      frame_id=pattern.frame_id),
                                                                                        pose=Pose(position=Point(0,0,0),
                                                                                                  orientation=Quaternion(0,0,0,1)
                                                                                                  )
                                                                                        )
                                                                   )
                                pose = poseStamped.pose
                            except tf.Exception:
                                pose = None

                                
                        # If we still need the velocity, then get it from ROS.
                        if (velocity is None) and (stamp is not None) and g_tfrx.canTransform('Plate', pattern.frame_id, stamp):
                            try:
                                velocity_tuple = g_tfrx.lookupTwist('Plate', pattern.frame_id, stamp, self.dtVelocity)
                            except tf.Exception:
                                velocity = None
                            else:
                                velocity = Twist(linear=Point(x=velocity_tuple[0][0],
                                                              y=velocity_tuple[0][1],
                                                              z=velocity_tuple[0][2]), 
                                                 angular=Point(x=velocity_tuple[1][0],
                                                               y=velocity_tuple[1][1],
                                                               z=velocity_tuple[1][2]))

                        # If we still need the speed, then get it from velocity.
                        if (speed is None) and (velocity is not None):
                            speed = N.linalg.norm([velocity.linear.x, velocity.linear.y, velocity.linear.z])

                                                        
                        # See if any of the states are in range of the filter.
                        if (pose is not None) and (velocity is not None) and (speed is not None):
                            state = MsgFrameState(pose = pose, 
                                                  velocity = velocity,
                                                  speed = speed)
    
                            bInStatefilterRangePrev[iPattern] = bInStatefilterRange[iPattern]
                            bInStatefilterRange[iPattern] = self.InStatefilterRange(state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            self.PublishStatefilterMarkers (state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria)
                            
                            # If any of the filter states have changed, then we need to update them all.
                            if bInStatefilterRangePrev[iPattern] != bInStatefilterRange[iPattern]:
                                bFilterStateChanged = True
                    # end for iPattern in range(nPatterns)
                    
                    #rospy.logwarn ('%s: %s' % (bFilterStateChanged, bInStatefilterRange))
                    
                    # If filter state has changed, then publish the new command                    
                    if bFilterStateChanged:
                        command.pattern_list = []
                        for iPattern in range(nPatterns):
                            if bInStatefilterRange[iPattern]:
                                pattern = userdata.experimentparamsIn.lasertrack.pattern_list[iPattern]
                                command.pattern_list.append(pattern)
                    
                        if len(command.pattern_list)>0:
                            command.enable_laser = True
                        else:
                            command.enable_laser = False
                            
                        self.pubGalvoCommand.publish(command)

                # else command.pattern_list contains all patterns, and has already been published.
                
                
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
class ExperimentLib():
    def __init__(self, experimentparams=None, newexperiment_callback=None, newtrial_callback=None, endtrial_callback=None):
        self.xHome = 0
        self.yHome = 0

        self.Trigger = TriggerService()
        self.Trigger.attach()
        
        # Create the state machine.
        self.smachTop = smach.StateMachine(outcomes = ['success','aborted'])
        self.smachTop.userdata.experimentparams = experimentparams
        
        # Create the "actions" concurrency state.
        smachActions = smach.Concurrence(outcomes = ['success','disabled','aborted'],
                                         default_outcome = 'aborted',
                                         child_termination_cb = self.any_action_term_callback,
                                         outcome_cb = self.all_actions_term_callback,
                                         input_keys = ['experimentparamsIn'])

        with smachActions:
            smach.Concurrence.add('MOVEROBOT', 
                                  MoveRobot ())
            smach.Concurrence.add('LASERTRACK', 
                                  Lasertrack ())
            smach.Concurrence.add('EXITTRIGGER', 
                                  TriggerOnStates(type='exit'))

        
        with self.smachTop:
            ################################################################################### NEWEXPERIMENT ->  NEWEXPERIMENTCALLBACK or RESETHARDWARE
            if newexperiment_callback is not None:
                stateAfterNewExperiment = 'NEWEXPERIMENTCALLBACK'
            else:
                stateAfterNewExperiment = 'RESETHARDWARE'

            smach.StateMachine.add('NEWEXPERIMENT',
                                   NewExperiment(),
                                   transitions={'success':stateAfterNewExperiment,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


            ################################################################################### NEWEXPERIMENTCALLBACK -> RESETHARDWARE
            if newexperiment_callback is not None:
                smach.StateMachine.add('NEWEXPERIMENTCALLBACK', 
                                       smach.CBState(newexperiment_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'RESETHARDWARE',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})



            ################################################################################### RESETHARDWARE -> NEWTRIALCALLBACK or NEWTRIAL
            if newtrial_callback is not None:
                stateAfterResetHardware = 'NEWTRIALCALLBACK'
            else:
                stateAfterResetHardware = 'NEWTRIAL'

            smach.StateMachine.add('RESETHARDWARE',
                                   ResetRobot(),
                                   transitions={'success':stateAfterResetHardware,
                                                'disabled':stateAfterResetHardware,
                                                'timeout':stateAfterResetHardware,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### NEWTRIALCALLBACK -> NEWTRIAL
            if newtrial_callback is not None:
                smach.StateMachine.add('NEWTRIALCALLBACK', 
                                       smach.CBState(newtrial_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'NEWTRIAL',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})


            ################################################################################### NEWTRIAL -> WAITENTRY
            smach.StateMachine.add('NEWTRIAL',                         
                                   NewTrial(),
                                   transitions={'continue':'WAITENTRY',     # Trigger service signal goes False.
                                                'stop':'success',           # Trigger service signal goes False.
                                                'aborted':'aborted'},       # Trigger service signal goes False.
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


            ################################################################################### WAITENTRY -> ENTRYTRIGGER
            smach.StateMachine.add('WAITENTRY', 
                                   TriggerOnTime(type='entry'),
                                   transitions={'success':'ENTRYTRIGGER',   # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### ENTRYTRIGGER -> ACTIONS
            smach.StateMachine.add('ENTRYTRIGGER', 
                                   TriggerOnStates(type='entry'),
                                   transitions={'success':'ACTIONS',        # Trigger service signal goes True.
                                                'disabled':'ACTIONS',       # Trigger service signal goes True.
                                                'timeout':'ACTIONS',        # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### ACTIONS -> WAITEXIT
            smach.StateMachine.add('ACTIONS', 
                                   smachActions,
                                   transitions={'success':'WAITEXIT',
                                                'disabled':'WAITEXIT',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### WAITEXIT -> ENDTRIALCALLBACK or RESETHARDWARE
            if endtrial_callback is not None:
                stateAfterWaitExit = 'ENDTRIALCALLBACK'
            else:
                stateAfterWaitExit = 'RESETHARDWARE'

            smach.StateMachine.add('WAITEXIT', 
                                   TriggerOnTime(type='exit'),
                                   transitions={'success':stateAfterWaitExit,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### ENDTRIALCALLBACK -> RESETHARDWARE
            if endtrial_callback is not None:
                smach.StateMachine.add('ENDTRIALCALLBACK', 
                                       smach.CBState(endtrial_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'RESETHARDWARE',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})

        self.sis = smach_ros.IntrospectionServer('sis_experiment',
                                                 self.smachTop,
                                                 '/EXPERIMENT')
        
        
    # Gets called upon ANY child state termination.
    def any_action_term_callback(self, outcome_map):
        rv = False

        if ('EXITTRIGGER' in outcome_map) and ('MOVEROBOT' in outcome_map) and ('LASERTRACK' in outcome_map):
            # If any of the states either succeeds or times-out, then we're done (i.e. preempt the other states).
            if (outcome_map['EXITTRIGGER']=='timeout') or (outcome_map['EXITTRIGGER']=='success'):
                rv = True 
            
            if (outcome_map['MOVEROBOT']=='timeout') or (outcome_map['MOVEROBOT']=='success'):
                rv = True 
            
            if (outcome_map['LASERTRACK']=='timeout') or (outcome_map['LASERTRACK']=='success'):
                rv = True 
        
            
        return rv
    

    # Gets called after all child states are terminated.
    def all_actions_term_callback(self, outcome_map):
        rv = 'aborted'
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
            outcome = self.smachTop.execute()
        except Exception, e: #smach.exceptions.InvalidUserCodeError, e:
            rospy.logwarn('Exception executing state machine: %s' % e)
        
        rospy.logwarn ('Experiment state machine finished with outcome=%s' % outcome)
        self.sis.stop()



