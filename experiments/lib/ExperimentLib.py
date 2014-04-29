#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import copy
import numpy as N
import smach
import smach_ros
import tf
import time

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from std_srvs.srv import Empty

from experiment_srvs.srv import Trigger, ExperimentParams
from flycore.msg import MsgFrameState, TrackingCommand
from flycore.srv import SrvFrameState, SrvFrameStateRequest
from tracking.msg import ArenaState
from visualization_msgs.msg import Marker


################################################################
# To add a new hardware component, do the following:
# 1. Create a file ExperimentLib_______.py (see the others for examples)
# 2. Import it into this file, as below.
# 3. Add it to the g_actions_dict, as below.
# 4. Add a section for the new stuff in the experimentparams structure, i.e. files in the experiments/msg directory.
# 5. Modify save/nodes/SaveArenastate.py to save the new stuff as appropriate.
#

import ExperimentLibRobot
import ExperimentLibLaserGalvos
import ExperimentLibLEDPanels

g_actions_dict = {'ROBOT' : ExperimentLibRobot, 
                  'LASERGALVOS' : ExperimentLibLaserGalvos,
                  'LEDPANELS' : ExperimentLibLEDPanels}
################################################################

# The notify_list contains those service basenames to call for each experiment: .../init, .../trial_start, .../trigger, .../trial_end, .../wait_until_done, 
g_notify_list = ['savearenastate', 
                 'saveimages', 
                 'savebag', 
                 'tracking', 
                 'transformserverarenacamera', 
                 'transformserverarenastage']

#######################################################################################################
#######################################################################################################
# NotifyServices()
# Sends notification commands to a list of services, each taking an argument of the given type.
# When you call NotifyServices.notify(), then for each in the list, we call the given services with the given parameter.
#
class NotifyServices():
    def __init__(self, services_dict={}, type=None):
        self.services_dict = services_dict
        self.type = type
        
    def attach(self):
        for key in self.services_dict:
            #rospy.logwarn('Waiting for service: %s' % key)
            rospy.wait_for_service(key)
            try:
                self.services_dict[key] = rospy.ServiceProxy(key, self.type)
            except rospy.ServiceException, e:
                rospy.logwarn ('FAILED %s: %s' % (key,e))
            #rospy.logwarn('Attached service: %s' % key)
            
        
    def notify(self, param):
        for key,callback in self.services_dict.iteritems():
            if callback is not None:
                callback(param)
# End class NotifyServices()
            
        

#######################################################################################################
#######################################################################################################
class StartExperiment (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        self.pubTrackingCommand = rospy.Publisher('tracking/command', TrackingCommand, latch=True)

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/init'] = None
             
        self.ExperimentStartServices = NotifyServices(services_dict=services_dict, type=ExperimentParams) #ExperimentStartServices()
        self.ExperimentStartServices.attach()

        self.arenastate = None
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def execute(self, userdata):
        experimentparams = userdata.experimentparamsIn
        rospy.loginfo('EL State StartExperiment(%s)' % experimentparams)

        experimentparams.experiment.trial = experimentparams.experiment.trial-1 
        userdata.experimentparamsOut = experimentparams
        
        self.ExperimentStartServices.notify(experimentparams)

        # Set up the tracking.    
        userdata.experimentparamsOut = experimentparams
        msgTrackingCommand                    = TrackingCommand()
        msgTrackingCommand.command            = 'initialize'
        msgTrackingCommand.exclusionzones     = experimentparams.tracking.exclusionzones
        msgTrackingCommand.nRobots            = experimentparams.robotspec.nRobots
        msgTrackingCommand.nFlies             = experimentparams.flyspec.nFlies
        msgTrackingCommand.bUseVisualServoing = experimentparams.robotspec.isPresent
        self.pubTrackingCommand.publish(msgTrackingCommand)
        
        # Wait for the Arenastate to get published.
        while self.arenastate is None:
            rospy.logwarn('Waiting for tracking to deliver an Arenastate.')
            rospy.sleep(0.5)
            
        #rospy.loginfo ('EL Exiting StartExperiment()')
        return 'success'
# End class StartExperiment()



#######################################################################################################
#######################################################################################################
class EndExperiment (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/wait_until_done'] = None
            
        self.ExperimentEndServices = NotifyServices(services_dict=services_dict, type=ExperimentParams) #ExperimentEndServices()
        self.ExperimentEndServices.attach()
        
        

    def execute(self, userdata):
        experimentparams = userdata.experimentparamsIn
        rospy.loginfo('EL State EndExperiment(%s)' % experimentparams)

        userdata.experimentparamsOut = experimentparams
        
        self.ExperimentEndServices.notify(experimentparams)
            
        #rospy.loginfo ('EL Exiting EndExperiment()')
        return 'success'
# End class EndExperiment()



#######################################################################################################
#######################################################################################################
# StartTrial() - Increments the trial number, untriggers, and calls the 
#              new_trial service (which begins recording).
#
# Experiment may be paused & restarted via the commandlines:
# rostopic pub -1 experiment/command std_msgs/String pause_now
# rostopic pub -1 experiment/command std_msgs/String pause_after_trial 
# rostopic pub -1 experiment/command std_msgs/String continue
# rostopic pub -1 experiment/command std_msgs/String exit_now
# rostopic pub -1 experiment/command std_msgs/String exit_after_trial
#
class StartTrial (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['continue','exit','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        self.pubTrackingCommand = rospy.Publisher('tracking/command', TrackingCommand, latch=True)
        
        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trigger'] = None

        self.TriggerServices = NotifyServices(services_dict=services_dict, type=Trigger) #TriggerServices()
        self.TriggerServices.attach()
        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trial_start'] = None

        self.TrialStartServices = NotifyServices(services_dict=services_dict, type=ExperimentParams) #TrialStartServices()
        self.TrialStartServices.attach()


    def execute(self, userdata):
        rv = 'exit'
        experimentparams = userdata.experimentparamsIn
        experimentparams.experiment.trial = userdata.experimentparamsIn.experiment.trial+1
        if (experimentparams.experiment.maxTrials != -1):
            if (experimentparams.experiment.maxTrials < experimentparams.experiment.trial):
                return rv

        now = rospy.Time.now().to_sec()
        experimentparams.save.timestamp = '%04d%02d%02d%02d%02d%02d' % (time.localtime(now).tm_year,
                                                                        time.localtime(now).tm_mon,
                                                                        time.localtime(now).tm_mday,
                                                                        time.localtime(now).tm_hour,
                                                                        time.localtime(now).tm_min,
                                                                        time.localtime(now).tm_sec)
        

        rospy.loginfo ('EL State StartTrial(%s)' % experimentparams.experiment.trial)


        # Set up the tracking.    
        userdata.experimentparamsOut = experimentparams
        msgTrackingCommand                    = TrackingCommand()
        msgTrackingCommand.command            = 'initialize'
        msgTrackingCommand.exclusionzones     = experimentparams.tracking.exclusionzones
        msgTrackingCommand.nRobots            = experimentparams.robotspec.nRobots
        msgTrackingCommand.nFlies             = experimentparams.flyspec.nFlies
        msgTrackingCommand.bUseVisualServoing = experimentparams.robotspec.isPresent
        
        self.pubTrackingCommand.publish(msgTrackingCommand)
        

        # Tell everyone that the trial is starting.
        try:
            self.TrialStartServices.notify(experimentparams)
            rv = 'continue'
        except rospy.ServiceException:
            rv = 'aborted'


        #rospy.loginfo ('EL Exiting StartTrial()')
        return rv
# End class StartTrial()



#######################################################################################################
#######################################################################################################
# EndTrial() - Calls the trial_end service (which finishes recording).
#
# Experiment may be paused & restarted via the commandlines:
# rostopic pub -1 experiment/command std_msgs/String pause
# rostopic pub -1 experiment/command std_msgs/String run
#
class EndTrial (smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['continue','exit','aborted'],
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
        
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.pubCommand = rospy.Publisher('experiment/command', String)
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)
        self.bSelfPublished = False

        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trial_end'] = None

        self.TrialEndServices = NotifyServices(services_dict=services_dict, type=ExperimentParams) #TrialEndServices()
        self.TrialEndServices.attach()


    def CommandExperiment_callback(self, msgString):
        if (not self.bSelfPublished):
            if msgString.data in self.commandExperiment_list:
                self.commandExperiment = msgString.data
                if '_after_trial' in self.commandExperiment:
                    rospy.logwarn ('Experiment received command: "%s".  Will take effect after this trial.' % self.commandExperiment)
                else:
                    rospy.logwarn ('Experiment received command: "%s".' % self.commandExperiment)
            else:
                rospy.logwarn ('Experiment received unknown command: "%s".  Valid commands are %s' % (msgString.data, self.commandExperiment_list))
        else:
            self.bSelfPublished = False
        
            
        
    def execute(self, userdata):
        rv = 'exit'
        experimentparams = userdata.experimentparamsIn

        rospy.loginfo ('EL State EndTrial(%s)' % experimentparams.experiment.trial)

        # Tell everyone that the trial is ending.
        try:
            self.TrialEndServices.notify(experimentparams)
            rv = 'continue'
        except rospy.ServiceException:
            rv = 'aborted'


        # Handle various user commands.
        if (self.commandExperiment=='pause_after_trial'):
            self.bSelfPublished = True
            self.pubCommand.publish('pause_now')
            rospy.logwarn ('**************************************** Experiment paused at EndTrial...')
            while (self.commandExperiment != 'continue'):
                rospy.sleep(1)
            rospy.logwarn ('**************************************** Experiment continuing.')

        
        # Tell everyone else to exit now.
        if (self.commandExperiment=='exit_after_trial'):
            self.bSelfPublished = True
            self.pubCommand.publish('exit_now')

            
        if (self.commandExperiment=='exit_after_trial') or (self.commandExperiment=='exit_now'):
            rv = 'exit'

        #rospy.loginfo ('EL Exiting EndTrial()')
        return rv
# End class EndTrial()



#######################################################################################################
#######################################################################################################
class TriggerOnStates (smach.State):
    def __init__(self, mode='pre', tfrx=None):
        self.tfrx = tfrx
        self.mode               = mode
        
        self.isTriggered        = False
        self.timeTriggered      = None
        
        
        smach.State.__init__(self, 
                             outcomes=['success','disabled','timeout','preempt','aborted'],
                             input_keys=['experimentparamsIn'])
        rospy.on_shutdown(self.OnShutdown_callback)
        self.arenastate = None
        self.rosrate = rospy.Rate(rospy.get_param('experiment/looprate', 50))

        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trigger'] = None

        self.TriggerServices = NotifyServices(services_dict=services_dict, type=Trigger) #TriggerServices()
        self.TriggerServices.attach()
    
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.
    
        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        #rospy.logwarn('TriggerOnStates received: %s' % msgString.data)
        self.commandExperiment = msgString.data
            
        
    def OnShutdown_callback(self):
        pass
        

    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def GetDistanceFrameToFrame (self, frameidParent, frameidChild):
        distance = None
        try:
            stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = self.tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception:
            pass
        else:
            distance = N.linalg.norm([pointP.point.x, pointP.point.y, pointP.point.z])
            
        return distance


    def GetSpeedFrameToFrame (self, frameidParent, frameidChild):
        speed = None
        
        if (frameidParent!=frameidChild):
            # If absolute speed (i.e. in the Arena frame), then try to use ArenaState speed.
            if (frameidParent=='Arena') and (self.arenastate is not None):
    
                # Check the robot.
                if (self.nRobots>0) and (frameidChild==self.arenastate.robot.name):
                    speed = self.arenastate.robot.speed
    
                # Check the flies.
                if (speed is None):
                    for state in self.arenastate.flies:
                        if (frameidChild==state.name):
                            speed = state.speed
                            break
    
            else:    # Get the speed via transforms.
                try:
                    stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild)
                    ((vx,vy,vz),(wx,wy,wz)) = self.tfrx.lookupTwist(frameidChild, frameidParent, stamp-self.dtVelocity, self.dtVelocity)
                except (tf.Exception, AttributeError), e:
                    ((vx,vy,vz),(wx,wy,wz)) = ((0,0,0),(0,0,0))
    
                speed = N.linalg.norm(N.array([vx,vy,vz]))
        else:
            speed = 0.0
            
        return speed


    def GetAngleFrameToFrame (self, frameidParent, frameidChild):
        angleToChild = None
        try:
            stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild)
            pointC = PointStamped(header=Header(frame_id=frameidChild, stamp=stamp),
                                  point=Point(x=0.0, y=0.0, z=0.0))
            pointP = self.tfrx.transformPoint(frameidParent, pointC)
        except tf.Exception, e:
            #rospy.logwarn('Exception in GetFrameToFrame():  %s' % e)
            pass
        else:
            angleToChild = N.arctan2(pointP.point.y, pointP.point.x) % (2.0*N.pi)
            
        return angleToChild



    def step_time_elapsed(self):
        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.timeElapsed = self.timeElapsed + dt

        
        
    def execute(self, userdata):
        rospy.loginfo('EL State TriggerOnStates(%s)' % (self.mode))


        self.nRobots = userdata.experimentparamsIn.robotspec.nRobots
        
        if self.mode == 'pre':
            trigger = userdata.experimentparamsIn.pre.trigger
        else:
            trigger = userdata.experimentparamsIn.post.trigger

        rv = 'disabled'
        if (trigger.enabled):
            self.timePrev = rospy.Time.now()
            self.timeElapsed = rospy.Time(0)
            
            self.isTriggered = False
            self.timeTriggered  = None
            
            # Wait for an arenastate.
            while (self.arenastate is None):
                self.step_time_elapsed()
                if (trigger.timeout != -1):
                    if ((self.timeElapsed.to_sec()) > trigger.timeout):
                        return 'timeout'
                #if self.preempt_requested():
                #    self.service_preempt()
                #    self.TriggerServices.notify(False)
                #    return 'preempt'
                rospy.sleep(1.0)
    
            rv = 'aborted'
            while (not rospy.is_shutdown()):
                self.step_time_elapsed()
                
                # Test for distance.
                isDistanceInRange = True
                distance = None
                if (trigger.distanceMin is not None) and (trigger.distanceMax is not None):
                    distance = self.GetDistanceFrameToFrame(trigger.frameidParent, trigger.frameidChild)
                    isDistanceInRange = False
                    if (distance is not None) and (trigger.distanceMin <= distance <= trigger.distanceMax):
                        isDistanceInRange = True
                        
                # Test for angle.
                isAngleInRange = True
                if (trigger.angleMin is not None) and (trigger.angleMax is not None):
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
                    speedAbsParent = self.GetSpeedFrameToFrame('Arena', trigger.frameidParent)# Absolute speed of the parent frame.
                    #rospy.loginfo ('EL parent speed=%s' % speedAbsParent)
                    if speedAbsParent is not None:
                        if (trigger.speedAbsParentMin <= speedAbsParent <= trigger.speedAbsParentMax):
                            isSpeedAbsParentInRange = True

                # Test for absolute speed of child.
                isSpeedAbsChildInRange = True
                if (trigger.speedAbsChildMin is not None) and (trigger.speedAbsChildMax is not None):
                    isSpeedAbsChildInRange = False
                    speedAbsChild = self.GetSpeedFrameToFrame('Arena', trigger.frameidChild)# Absolute speed of the child frame.
                    #rospy.loginfo ('EL child speed=%s' % speedAbsChild)
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
                        self.timeTriggered = self.timeElapsed
                else:
                    # Cancel a pending trigger.
                    self.isTriggered = False
                    self.timeTriggered = None

                #if (distance is not None) and (angle is not None) and (speedAbsParent is not None) and (speedAbsChild is not None) and (speedRel is not None):
                #    rospy.logwarn ('EL triggers=distance=%0.3f, speed=%0.3f,%0.3f, angle=%0.3f, bools=%s' % (distance, speedAbsParent, speedAbsChild, angle, [isDistanceInRange, isSpeedAbsParentInRange, isSpeedAbsChildInRange, isSpeedRelInRange, isAngleInRange]))
                #else:
                #    rospy.logwarn ('EL triggers=distance=%s, speed=%s,%s, angle=%s, bools=%s' % (distance, speedAbsParent, speedAbsChild, angle, [isDistanceInRange, isSpeedAbsParentInRange, isSpeedAbsChildInRange, isSpeedRelInRange, isAngleInRange]))

                # If pending trigger has lasted longer than requested duration, then set trigger.
                if (self.isTriggered):
                    duration = self.timeElapsed.to_sec() - self.timeTriggered.to_sec()
                    
                    if (trigger.timeHold <= duration):
                        rv = 'success'
                        break
                    
                    #rospy.loginfo('EL duration=%s' % duration)
                if (self.commandExperiment=='exit_now'):
                    #rospy.logwarn('TriggerOnStates: aborted.')
                    rv = 'aborted'
                    break

                    
                if (self.preempt_requested()):
                    rospy.loginfo('preempt requested: TriggerOnStates()')
                    self.service_preempt()
                    rv = 'preempt'
                    break
                
                if (trigger.timeout != -1):
                    if (trigger.timeout < self.timeElapsed.to_sec()):
                        rv = 'timeout'
                        break
                
                self.rosrate.sleep()

        #rospy.logwarn ('rv=%s', rv)
        #rospy.logwarn ('self.mode=%s', self.mode)
        if self.mode=='pre':
            self.TriggerServices.notify(True)
        if self.mode=='post':
            self.TriggerServices.notify(False)
                
        #rospy.loginfo ('EL Exiting TriggerOnStates()')
        return rv
# End class TriggerOnStates()

    

#######################################################################################################
#######################################################################################################
class TriggerOnTime (smach.State):
    def __init__(self, mode='pre1', tfrx=None):
        self.tfrx = tfrx
        self.mode = mode
        smach.State.__init__(self, 
                             outcomes=['success','aborted'],
                             input_keys=['experimentparamsIn'])

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trigger'] = None

        self.TriggerServices = NotifyServices(services_dict=services_dict, type=Trigger) #TriggerServices()
        self.TriggerServices.attach()
        

        # Command messages.
        self.commandExperiment = 'continue'
        self.commandExperiment_list = ['continue','pause_now','pause_after_trial', 'exit_after_trial', 'exit_now']
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)


    def CommandExperiment_callback(self, msgString):
        #rospy.logwarn('TriggerOnTime received: %s' % msgString.data)
        self.commandExperiment = msgString.data
            
        
    def step_time_elapsed(self):
        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.timeElapsed = self.timeElapsed + dt

        
        
    def execute(self, userdata):
        if self.mode=='pre1':
            elapsedMax = userdata.experimentparamsIn.pre.wait1
        elif self.mode=='pre2':
            elapsedMax = userdata.experimentparamsIn.pre.wait2
        elif self.mode=='post':
            elapsedMax = userdata.experimentparamsIn.post.wait
        else:
            rospy.logwarn ('TriggerOnTime mode must be one of: pre1, pre2, post.')
            

        rospy.loginfo('EL State TriggerOnTime(%s, %s)' % (self.mode, elapsedMax))

        self.timePrev = rospy.Time.now()
        self.timeElapsed = rospy.Time(0)
            
        rv = 'success'
        while not rospy.is_shutdown():
            self.step_time_elapsed()

            if (elapsedMax <= self.timeElapsed.to_sec()):
                rv = 'success'
                break

            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break

            rospy.sleep(0.1)

        #try:
        #    if rv!='aborted' and ('pre' in self.mode):
        #        self.TriggerServices.notify(True)
        #    if rv!='aborted' and ('post' in self.mode):
        #        self.TriggerServices.notify(False)
        #except rospy.ServiceException:
        #    rv = 'aborted'

        #rospy.loginfo ('EL Exiting TriggerOnTime()')
        return rv
# End class TriggerOnTime()



#######################################################################################################
#######################################################################################################
class ExperimentLib():
    def __init__(self, experimentparams=None, startexperiment_callback=None, starttrial_callback=None, endtrial_callback=None):
        
        self.actions_dict = g_actions_dict
        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trigger'] = None

        self.TriggerServices = NotifyServices(services_dict=services_dict, type=Trigger) #TriggerServices()
        self.TriggerServices.attach()
        self.tfrx = tf.TransformListener()
        
        # Create the state machine.
        ################################################################################### Top level
        self.smachTop = smach.StateMachine(outcomes = ['success','aborted'])
        self.smachTop.userdata.experimentparams = experimentparams


        ################################################################################### Pre Qualifier:  wait -> trigger -> wait.
        self.smachPreQualifier = smach.StateMachine(outcomes = ['success','preempt','aborted'])
        self.smachPreQualifier.userdata.experimentparams = experimentparams
        with self.smachPreQualifier:
            smach.StateMachine.add('PREWAIT1', 
                                   TriggerOnTime(mode='pre1', tfrx=self.tfrx),
                                   transitions={'success':'PRETRIGGER',   # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            smach.StateMachine.add('PRETRIGGER', 
                                   TriggerOnStates(mode='pre', tfrx=self.tfrx),
                                   transitions={'success':'PREWAIT2',        # Trigger service signal goes True.
                                                'disabled':'PREWAIT2',       # Trigger service signal goes True.
                                                'timeout':'PREWAIT2',        # Trigger service signal goes True.
                                                'preempt':'preempt',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            smach.StateMachine.add('PREWAIT2', 
                                   TriggerOnTime(mode='pre2', tfrx=self.tfrx),
                                   transitions={'success':'success',   # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})
        
        
        # Create the 'Pre' concurrency state.  This is where the pre qualifier runs alongside the pre actions.
        ################################################################################### Pre qualifiers & trials.
        smachPre = smach.Concurrence(outcomes = ['success','disabled','timeout','aborted'],
                                       default_outcome = 'aborted',
                                       child_termination_cb = self.AnyPreTerm_callback,
                                       outcome_cb = self.AllPreTerm_callback,
                                       input_keys = ['experimentparamsIn'])
        with smachPre:
            smach.Concurrence.add('PREQUALIFIER', 
                                  self.smachPreQualifier)
            for (name,classHardware) in self.actions_dict.iteritems():   # Add a state for each action class in the dict.
                smach.Concurrence.add(name, 
                                      classHardware.Action (mode='pre', tfrx=self.tfrx))

        
        # Create the main 'Trial' concurrency state.
        ################################################################################### TRIAL
        smachTrial = smach.Concurrence(outcomes = ['success','disabled','aborted'],
                                         default_outcome = 'aborted',
                                         child_termination_cb = self.AnyTrialTerm_callback,
                                         outcome_cb = self.AllTrialTerm_callback,
                                         input_keys = ['experimentparamsIn'])
        with smachTrial:
            for (name,classHardware) in self.actions_dict.iteritems():   # Add a state for each action class in the dict.
                smach.Concurrence.add(name, 
                                      classHardware.Action (mode='trial', tfrx=self.tfrx))
            smach.Concurrence.add('POSTTRIGGER', 
                                  TriggerOnStates(mode='post', tfrx=self.tfrx))


        # Create the 'RESET' concurrency state.
        ################################################################################### RESET
        smachReset = smach.Concurrence(outcomes = ['success','aborted'],
                                         default_outcome = 'aborted',
                                         child_termination_cb = self.AnyResetTerm_callback,
                                         outcome_cb = self.AllResetTerm_callback,
                                         input_keys = ['experimentparamsIn'])
        with smachReset:
            for (name,classHardware) in self.actions_dict.iteritems():
                smach.Concurrence.add(name, 
                                      classHardware.Reset (tfrx=self.tfrx))


        ##############################################
        # Now create the main state machine.
        ##############################################
        with self.smachTop:
            ################################################################################### STARTEXPERIMENT ->  STARTEXPERIMENTCALLBACK or RESETHARDWARE
            if startexperiment_callback is not None:
                stateAfterStartExperiment = 'STARTEXPERIMENTCALLBACK'
            else:
                stateAfterStartExperiment = 'RESETHARDWARE'

            smach.StateMachine.add('STARTEXPERIMENT',
                                   StartExperiment(),
                                   transitions={'success':stateAfterStartExperiment,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


            ################################################################################### STARTEXPERIMENTCALLBACK -> RESETHARDWARE
            if startexperiment_callback is not None:
                smach.StateMachine.add('STARTEXPERIMENTCALLBACK', 
                                       smach.CBState(startexperiment_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'RESETHARDWARE',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})



            ################################################################################### RESETHARDWARE -> (STARTTRIALCALLBACK or STARTTRIAL)
            if starttrial_callback is not None:
                stateAfterResetHardware = 'STARTTRIALCALLBACK'
            else:
                stateAfterResetHardware = 'STARTTRIAL'

            smach.StateMachine.add('RESETHARDWARE',                         
                                   smachReset,
                                   transitions={'success':stateAfterResetHardware,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### STARTTRIALCALLBACK -> STARTTRIAL
            if starttrial_callback is not None:
                smach.StateMachine.add('STARTTRIALCALLBACK', 
                                       smach.CBState(starttrial_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'STARTTRIAL',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})


            ################################################################################### STARTTRIAL -> PRE
            smach.StateMachine.add('STARTTRIAL',                         
                                   StartTrial(),
                                   transitions={'continue':'PRE',         # Trigger service signal goes False.
                                                'exit':'ENDEXPERIMENT',           # Trigger service signal goes False.
                                                'aborted':'aborted'},       # Trigger service signal goes False.
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})




            ################################################################################### PRE -> TRIAL
            smach.StateMachine.add('PRE',                         
                                   smachPre,
                                   transitions={'success':'TRIAL', 
                                                'disabled':'TRIAL',
                                                'timeout':'TRIAL',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})




            ################################################################################### TRIAL -> POSTWAIT
            smach.StateMachine.add('TRIAL', 
                                   smachTrial,
                                   transitions={'success':'POSTWAIT',
                                                'disabled':'POSTWAIT',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### POSTWAIT -> (ENDTRIALCALLBACK or RESETHARDWARE)
            if endtrial_callback is not None:
                stateAfterPostWait = 'ENDTRIALCALLBACK'
            else:
                stateAfterPostWait = 'RESETHARDWARE'

            smach.StateMachine.add('POSTWAIT', 
                                   TriggerOnTime(mode='post', tfrx=self.tfrx),
                                   transitions={'success':stateAfterPostWait,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams'})


            ################################################################################### ENDTRIALCALLBACK -> ENDTRIAL
            if endtrial_callback is not None:
                smach.StateMachine.add('ENDTRIALCALLBACK', 
                                       smach.CBState(endtrial_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsIn'],
                                                     output_keys = ['experimentparamsOut']),
                                       transitions={'success':'ENDTRIAL',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsIn':'experimentparams',
                                                  'experimentparamsOut':'experimentparams'})

            ################################################################################### ENDTRIAL -> RESETHARDWARE or ENDEXPERIMENT
            smach.StateMachine.add('ENDTRIAL',                         
                                   EndTrial(),
                                   transitions={'continue':'RESETHARDWARE',         
                                                'exit':'ENDEXPERIMENT',           
                                                'aborted':'aborted'},       
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


            ################################################################################### ENDEXPERIMENT ->  end
            smach.StateMachine.add('ENDEXPERIMENT',
                                   EndExperiment(),
                                   transitions={'success':'success',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


        self.sis = smach_ros.IntrospectionServer('sis_experiment',
                                                 self.smachTop,
                                                 '/EXPERIMENT')
        
        rospy.on_shutdown(self.OnShutdown_callback)


    def OnShutdown_callback(self):
        #rospy.logwarn ('smach shutdown')
        self.smachTop.request_preempt()
        
        
    # Gets called upon ANY 'reset' state termination.
    def AnyResetTerm_callback(self, outcome_map):
        #rospy.logwarn('AnyResetTerm_callback(%s)' % repr(outcome_map))
        rv = False
        
        # rv==True:  preempt all remaining states.
        # rv==False: keep running.
        return rv

    
    # Gets called after all 'reset' states are terminated.
    # If any states aborted, then abort.
    def AllResetTerm_callback(self, outcome_map):
        #rospy.logwarn('AllResetTerm_callback(%s)' % repr(outcome_map))
        #rospy.logwarn('actions_dict=%s' % repr(self.actions_dict))
        
        rv = 'success'
        for (name,classHardware) in self.actions_dict.iteritems():
            if (name in outcome_map):
                if outcome_map[name] == 'aborted':
                    rv = 'aborted'
                    
        return rv

        

    # Gets called upon ANY 'pre' state termination.
    # If any of the states either succeeds or times-out, then we're done (i.e. preempt the other states).
    def AnyPreTerm_callback(self, outcome_map):
        #rospy.logwarn('AnyPreTerm_callback(%s)' % repr(outcome_map))
        rv = False


        # Make sure all the entries are in the outcome_map.
        bAllPresent = True
        if ('PREQUALIFIER' not in outcome_map):
            bAllPresent = False
        for (name,classHardware) in self.actions_dict.iteritems():
            if (name not in outcome_map):
                bAllPresent = False
                break


        # If any of the states either succeeds or times-out, then we're done (i.e. preempt the other states).
        if (bAllPresent):
            if (outcome_map['PREQUALIFIER'] in ['timeout','success']):
                rv = True 
            for (name,classHardware) in self.actions_dict.iteritems():
                if (outcome_map[name] in ['timeout','success']):
                    rv = True 

        
        # rv==True:  preempt all remaining states.
        # rv==False: keep running.
        return rv
    

    # Gets called after all 'pre' states are terminated.
    def AllPreTerm_callback(self, outcome_map):
        #rospy.logwarn('AllPreTerm_callback(%s)' % repr(outcome_map))
        rv = 'aborted'
        
        # Check if all are present.
        bAllPresent = True
        if ('PREQUALIFIER' not in outcome_map):
            bAllPresent = False
        for (name,classHardware) in self.actions_dict.iteritems():
            if (name not in outcome_map):
                bAllPresent = False
                break
            
        if (bAllPresent):
            # Check if any timed-out or succeeded.
            b = False
            if (outcome_map['PREQUALIFIER'] in ['timeout','success']):
                b = True
            for (name,classHardware) in self.actions_dict.iteritems():
               if (outcome_map[name] in ['timeout','success']):
                   b = True
                   break
            if (b):
                rv = 'success'
                
                
#            if (rv == 'aborted'):
#                # Check if all == 'preempt'
#                b = True 
#                for (name,classHardware) in self.actions_dict.iteritems():
#                    if (outcome_map[name] != 'preempt'):
#                        b = False
#                        break
#                if (b):
#                    rv = 'preempt'


            if (rv == 'aborted'):
                # Check if all == 'disabled' 
                b = True 
                if (outcome_map['PREQUALIFIER'] != 'disabled'):
                    b = False
                for (name,classHardware) in self.actions_dict.iteritems():
                    if (outcome_map[name] != 'disabled'):
                        b = False
                        break
                if (b):
                    rv = 'disabled'
                
                
            if (rv == 'aborted'):
                # Check if any == 'aborted' 
                b = False 
                if (outcome_map['PREQUALIFIER'] == 'aborted'):
                    b = True
                for (name,classHardware) in self.actions_dict.iteritems():
                    if (outcome_map[name] == 'aborted'):
                        b = True
                        break
                if (b):
                    rv = 'aborted'


        return rv

        

    # Gets called upon ANY 'trial' state termination.
    def AnyTrialTerm_callback(self, outcome_map):
        #rospy.logwarn('AnyTrialTerm_callback(%s)' % repr(outcome_map))
        rv = False

        # Check if all are present.
        bAllPresent = True
        if ('POSTTRIGGER' not in outcome_map):
            bAllPresent = False
        for (name,classHardware) in self.actions_dict.iteritems():
            if (name not in outcome_map):
                bAllPresent = False
                break

        if (bAllPresent):
            # Check if any in ['timeout','success'], then we're done (i.e. preempt the other states).
            b = False 
            if (outcome_map['POSTTRIGGER'] in ['timeout','success']):
                b = True
            for (name,classHardware) in self.actions_dict.iteritems():
                if (outcome_map[name] in ['timeout','success']):
                    b = True
                    break
            if (b):
                rv = True
        
        
        # rv==True:  preempt all remaining states.
        # rv==False: keep running.
        return rv
    

    # Gets called after all 'trial' states are terminated.
    def AllTrialTerm_callback(self, outcome_map):
        #rospy.logwarn('AllTrialTerm_callback(%s)' % repr(outcome_map))
        rv = 'aborted'

        # Check if all are present.
        bAllPresent = True
        if ('POSTTRIGGER' not in outcome_map):
            bAllPresent = False
        for (name,classHardware) in self.actions_dict.iteritems():
            if (name not in outcome_map):
                bAllPresent = False
                break

        if (bAllPresent):
            
            # Check if any in ['timeout','success'], then we're done (i.e. preempt the other states).
            b = False 
            if (outcome_map['POSTTRIGGER'] in ['timeout','success']):
                b = True
            for (name,classHardware) in self.actions_dict.iteritems():
                if (outcome_map[name] in ['timeout','success']):
                    b = True
                    break
            if (b):
                rv = 'success'

                
#            if (rv == 'aborted'):
#                # Check if all == 'preempt'
#                b = True 
#                if (outcome_map['POSTTRIGGER'] != 'preempt'):
#                    b = False
#                for (name,classHardware) in self.actions_dict.iteritems():
#                    if (outcome_map[name] != 'preempt'):
#                        b = False
#                        break
#                if (b):
#                    rv = 'preempt'
                 
            if (rv == 'aborted'):
                # Check if all == 'disabled' 
                b = True 
                if (outcome_map['POSTTRIGGER'] != 'disabled'):
                    b = False
                for (name,classHardware) in self.actions_dict.iteritems():
                    if (outcome_map[name] != 'disabled'):
                        b = False
                        break
                if (b):
                    rv = 'disabled'
                
                
            if (rv == 'aborted'):
                # Check if any == 'aborted' 
                b = False 
                if (outcome_map['POSTTRIGGER'] == 'aborted'):
                    b = True
                for (name,classHardware) in self.actions_dict.iteritems():
                    if (outcome_map[name] == 'aborted'):
                        b = True
                        break
                if (b):
                    rv = 'aborted'



        return rv

        

    def Run(self):
        self.sis.start()
        try:
            outcome = self.smachTop.execute()
        except Exception, e: #smach.exceptions.InvalidUserCodeError, e:
            rospy.logwarn('Exception executing experiment: %s' % e)
        
        rospy.logwarn ('Experiment finished with outcome=%s' % outcome)
        self.sis.stop()


# End class ExperimentLib()
    
