#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
import copy
import numpy as np
import smach
import smach_ros
import tf
import time

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from std_srvs.srv import Empty

from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsRequest, ExperimentParamsChoices
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

# The time that the experiment started is t=0.  We use elapsed time rather than
# absolute ROS time so that it only progresses when the experiment is running,
# i.e. when the user pressed <Pause> in the FlylabGUI then elapsed time pauses too.
g_timeExperimentElapsed = 0.0


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
                             input_keys=['experimentparamsChoicesIn'],
                             output_keys=['experimentparamsChoicesOut'])

        self.pubTrackingCommand = rospy.Publisher('tracking/command', TrackingCommand, latch=True)

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/init'] = None
             
        self.ExperimentStartServices = NotifyServices(services_dict=services_dict, type=ExperimentParamsChoices) #ExperimentStartServices()
        self.ExperimentStartServices.attach()

        self.arenastate = None
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        
        
    def ArenaState_callback(self, arenastate):
        self.arenastate = arenastate


    def execute(self, userdata):
        global g_timeExperimentElapsed
        
        experimentparamsChoices = userdata.experimentparamsChoicesIn
        rospy.loginfo('EL State StartExperiment(%s)' % experimentparamsChoices)

        experimentparamsChoices.experiment.trial = experimentparamsChoices.experiment.trial-1 
        userdata.experimentparamsChoicesOut = experimentparamsChoices
        g_timeExperimentElapsed = 0.0
        
        self.ExperimentStartServices.notify(experimentparamsChoices)

        # Set up the tracking.    
        userdata.experimentparamsChoicesOut = experimentparamsChoices
        msgTrackingCommand                    = TrackingCommand()
        msgTrackingCommand.command            = 'initialize'
        msgTrackingCommand.exclusionzones     = experimentparamsChoices.tracking.exclusionzones
        msgTrackingCommand.nRobots            = experimentparamsChoices.robotspec.nRobots
        msgTrackingCommand.nFlies             = experimentparamsChoices.flyspec.nFlies
        msgTrackingCommand.bUseVisualServoing = experimentparamsChoices.robotspec.isPresent
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
                             input_keys=['experimentparamsChoicesIn'],
                             output_keys=['experimentparamsChoicesOut'])

        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/wait_until_done'] = None
            
        self.ExperimentEndServices = NotifyServices(services_dict=services_dict, type=ExperimentParamsChoices) #ExperimentEndServices()
        self.ExperimentEndServices.attach()
        
        

    def execute(self, userdata):
        experimentparamsChoices = userdata.experimentparamsChoicesIn
        rospy.loginfo('EL State EndExperiment(%s)' % experimentparamsChoices)

        userdata.experimentparamsChoicesOut = experimentparamsChoices
        
        self.ExperimentEndServices.notify(experimentparamsChoices)
            
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
                             input_keys=['experimentparamsChoicesIn'],
                             output_keys=['experimentparamsOut'])
        self.pubTrackingCommand = rospy.Publisher('tracking/command', TrackingCommand, latch=True)
        
        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trigger'] = None

        self.TriggerServices = NotifyServices(services_dict=services_dict, type=Trigger)
        self.TriggerServices.attach()
        
        services_dict = {}
        for name in g_notify_list:
            services_dict[name+'/trial_start'] = None

        self.TrialStartServices = NotifyServices(services_dict=services_dict, type=ExperimentParams)
        self.TrialStartServices.attach()


    # Choose()
    # Return one random entry from the given list.
    #
    def Choose(self, choices):
        if (len(choices)>0):
            choice = choices[np.random.randint(len(choices))]
        else:
            choice = None
        
        return choice
    
    
    def IterFields(self, obj):
        for objsub,val in obj.__dict__.iteritems():
            if (not callable(objsub)):
                if ('.msg.' in str(type(objsub))):
                    self.IterFields(name+'.'+namesub)
                else:
                    #rospy.logwarn('%s: %s' % (name+'.'+namesub, type(name+'.'+namesub)))
                    rospy.logwarn(objsub)
#         for d in dir(experimentparamsChoices):
#             if (d[0]!='_'):
#                 obj = eval('experimentparamsChoices.'+d)
#                 if  ('instancemethod' not in str(type(obj))):
#                     rospy.logwarn('%s: %s' % ('experimentparamsChoices.'+d, type(obj)))
#                     if ('.msg.' in str(type(obj))):
#                         for d2 in dir(obj):
#                             if (d2[0]!='_'):
#                                 obj2 = eval('experimentparamsChoices.'+d+'.'+d2)
#                                 if  ('instancemethod' not in str(type(obj2))):
#                                     rospy.logwarn('%s: %s' % ('experimentparamsChoices.'+d+'.'+d2, type(obj2)))
                        
    
    def GetChoice(self, experimentparamsChoices):
        # Make a choice for the experimentparams.        
        experimentparamsTemp = ExperimentParamsRequest()
        experimentparams     = ExperimentParamsRequest()
        
        
        #########################################
        # Make a temporary random choice, but it might have None values.
        experimentparamsTemp.experiment.description                             =             experimentparamsChoices.experiment.description
        experimentparamsTemp.experiment.maxTrials                               =             experimentparamsChoices.experiment.maxTrials
        experimentparamsTemp.experiment.timeout                                 =             experimentparamsChoices.experiment.timeout
        
        experimentparamsTemp.save.filenamebase                                  =             experimentparamsChoices.save.filenamebase
        experimentparamsTemp.save.csv                                           =             experimentparamsChoices.save.csv
        experimentparamsTemp.save.bag                                           =             experimentparamsChoices.save.bag
        experimentparamsTemp.save.mov                                           =             experimentparamsChoices.save.mov
        experimentparamsTemp.save.fmf                                           =             experimentparamsChoices.save.fmf
        experimentparamsTemp.save.imagetopic_list                               =             experimentparamsChoices.save.imagetopic_list
        experimentparamsTemp.save.onlyWhileTriggered                            =             experimentparamsChoices.save.onlyWhileTriggered
        
        experimentparamsTemp.robotspec.nRobots                                  =             experimentparamsChoices.robotspec.nRobots
        experimentparamsTemp.robotspec.width                                    =             experimentparamsChoices.robotspec.width
        experimentparamsTemp.robotspec.height                                   =             experimentparamsChoices.robotspec.height
        experimentparamsTemp.robotspec.isPresent                                =             experimentparamsChoices.robotspec.isPresent
        experimentparamsTemp.robotspec.description                              =             experimentparamsChoices.robotspec.description

        experimentparamsTemp.flyspec.nFlies                                     =             experimentparamsChoices.flyspec.nFlies
        experimentparamsTemp.flyspec.description                                =             experimentparamsChoices.flyspec.description
        
        experimentparamsTemp.tracking.exclusionzones.enabled                    =             experimentparamsChoices.tracking.exclusionzones.enabled
        experimentparamsTemp.tracking.exclusionzones.point_list                 =             experimentparamsChoices.tracking.exclusionzones.point_list
        experimentparamsTemp.tracking.exclusionzones.radius_list                =             experimentparamsChoices.tracking.exclusionzones.radius_list
        
        experimentparamsTemp.home.enabled                                       =             experimentparamsChoices.home.enabled
        if (experimentparamsTemp.home.enabled):
            experimentparamsTemp.home.x                                         =             experimentparamsChoices.home.x
            experimentparamsTemp.home.y                                         =             experimentparamsChoices.home.y
            experimentparamsTemp.home.speed                                     =             experimentparamsChoices.home.speed
            experimentparamsTemp.home.tolerance                                 =             experimentparamsChoices.home.tolerance

        experimentparamsTemp.pre.robot.enabled                                  =             experimentparamsChoices.pre.robot.enabled
        if (experimentparamsTemp.pre.robot.enabled):
            experimentparamsTemp.pre.robot.move.mode                            =             experimentparamsChoices.pre.robot.move.mode
            experimentparamsTemp.pre.robot.move.relative.tracking               = self.Choose(experimentparamsChoices.pre.robot.move.relative.tracking)                   
            experimentparamsTemp.pre.robot.move.relative.frameidOrigin          = self.Choose(experimentparamsChoices.pre.robot.move.relative.frameidOrigin)
            experimentparamsTemp.pre.robot.move.relative.distance               = self.Choose(experimentparamsChoices.pre.robot.move.relative.distance)
            experimentparamsTemp.pre.robot.move.relative.angleOffset            = self.Choose(experimentparamsChoices.pre.robot.move.relative.angleOffset)
            experimentparamsTemp.pre.robot.move.relative.angleVelocity          = self.Choose(experimentparamsChoices.pre.robot.move.relative.angleVelocity)
            experimentparamsTemp.pre.robot.move.relative.angleOscMag            = self.Choose(experimentparamsChoices.pre.robot.move.relative.angleOscMag)
            experimentparamsTemp.pre.robot.move.relative.angleOscFreq           = self.Choose(experimentparamsChoices.pre.robot.move.relative.angleOscFreq)
            experimentparamsTemp.pre.robot.move.relative.speedMax               = self.Choose(experimentparamsChoices.pre.robot.move.relative.speedMax)
            experimentparamsTemp.pre.robot.move.relative.tolerance              = self.Choose(experimentparamsChoices.pre.robot.move.relative.tolerance)
            experimentparamsTemp.pre.robot.move.relative.typeAngleOffset        = self.Choose(experimentparamsChoices.pre.robot.move.relative.typeAngleOffset)
            experimentparamsTemp.pre.robot.move.relative.typeAngleVelocity      = self.Choose(experimentparamsChoices.pre.robot.move.relative.typeAngleVelocity)
            experimentparamsTemp.pre.robot.move.relative.typeAngleOscMag        = self.Choose(experimentparamsChoices.pre.robot.move.relative.typeAngleOscMag)
            experimentparamsTemp.pre.robot.move.relative.typeAngleOscFreq       = self.Choose(experimentparamsChoices.pre.robot.move.relative.typeAngleOscFreq)
            experimentparamsTemp.pre.robot.move.relative.typeSpeedMax           = self.Choose(experimentparamsChoices.pre.robot.move.relative.typeSpeedMax)
            
            experimentparamsTemp.pre.robot.move.pattern.frameidPosition         = self.Choose(experimentparamsChoices.pre.robot.move.pattern.frameidPosition)
            experimentparamsTemp.pre.robot.move.pattern.frameidAngle            = self.Choose(experimentparamsChoices.pre.robot.move.pattern.frameidAngle)
            experimentparamsTemp.pre.robot.move.pattern.shape                   = self.Choose(experimentparamsChoices.pre.robot.move.pattern.shape)
            experimentparamsTemp.pre.robot.move.pattern.hzPattern               = self.Choose(experimentparamsChoices.pre.robot.move.pattern.hzPattern)
            experimentparamsTemp.pre.robot.move.pattern.hzPoint                 = self.Choose(experimentparamsChoices.pre.robot.move.pattern.hzPoint)
            experimentparamsTemp.pre.robot.move.pattern.count                   = self.Choose(experimentparamsChoices.pre.robot.move.pattern.count)
            experimentparamsTemp.pre.robot.move.pattern.size                    = self.Choose(experimentparamsChoices.pre.robot.move.pattern.size)
            experimentparamsTemp.pre.robot.move.pattern.param                   = self.Choose(experimentparamsChoices.pre.robot.move.pattern.param)
            experimentparamsTemp.pre.robot.move.pattern.direction               = self.Choose(experimentparamsChoices.pre.robot.move.pattern.direction)
            experimentparamsTemp.pre.robot.move.pattern.restart                 = self.Choose(experimentparamsChoices.pre.robot.move.pattern.restart)
        
        experimentparamsTemp.pre.lasergalvos                                    =             experimentparamsChoices.pre.lasergalvos
        
        experimentparamsTemp.pre.ledpanels.enabled                              =             experimentparamsChoices.pre.ledpanels.enabled
        if (experimentparamsTemp.pre.ledpanels.enabled):
            experimentparamsTemp.pre.ledpanels.command                          = self.Choose(experimentparamsChoices.pre.ledpanels.command)
            experimentparamsTemp.pre.ledpanels.idPattern                        = self.Choose(experimentparamsChoices.pre.ledpanels.idPattern)
            experimentparamsTemp.pre.ledpanels.origin                           = self.Choose(experimentparamsChoices.pre.ledpanels.origin)
            experimentparamsTemp.pre.ledpanels.frame_id                         = self.Choose(experimentparamsChoices.pre.ledpanels.frame_id)
            experimentparamsTemp.pre.ledpanels.statefilterHi                    = self.Choose(experimentparamsChoices.pre.ledpanels.statefilterHi)
            experimentparamsTemp.pre.ledpanels.statefilterLo                    = self.Choose(experimentparamsChoices.pre.ledpanels.statefilterLo)
            experimentparamsTemp.pre.ledpanels.statefilterCriteria              = self.Choose(experimentparamsChoices.pre.ledpanels.statefilterCriteria)
        
        experimentparamsTemp.pre.wait1                                          =             experimentparamsChoices.pre.wait1
        
        experimentparamsTemp.pre.trigger.enabled                                =             experimentparamsChoices.pre.trigger.enabled
        if (experimentparamsTemp.pre.trigger.enabled):
            experimentparamsTemp.pre.trigger.frameidParent                      =             experimentparamsChoices.pre.trigger.frameidParent
            experimentparamsTemp.pre.trigger.frameidChild                       =             experimentparamsChoices.pre.trigger.frameidChild
            experimentparamsTemp.pre.trigger.speedAbsParentMin                  =             experimentparamsChoices.pre.trigger.speedAbsParentMin
            experimentparamsTemp.pre.trigger.speedAbsParentMax                  =             experimentparamsChoices.pre.trigger.speedAbsParentMax
            experimentparamsTemp.pre.trigger.speedAbsChildMin                   =             experimentparamsChoices.pre.trigger.speedAbsChildMin
            experimentparamsTemp.pre.trigger.speedAbsChildMax                   =             experimentparamsChoices.pre.trigger.speedAbsChildMax
            experimentparamsTemp.pre.trigger.speedRelMin                        =             experimentparamsChoices.pre.trigger.speedRelMin
            experimentparamsTemp.pre.trigger.speedRelMax                        =             experimentparamsChoices.pre.trigger.speedRelMax
            experimentparamsTemp.pre.trigger.distanceMin                        =             experimentparamsChoices.pre.trigger.distanceMin
            experimentparamsTemp.pre.trigger.distanceMax                        =             experimentparamsChoices.pre.trigger.distanceMax
            experimentparamsTemp.pre.trigger.angleMin                           =             experimentparamsChoices.pre.trigger.angleMin
            experimentparamsTemp.pre.trigger.angleMax                           =             experimentparamsChoices.pre.trigger.angleMax
            experimentparamsTemp.pre.trigger.angleTest                          =             experimentparamsChoices.pre.trigger.angleTest
            experimentparamsTemp.pre.trigger.angleTestBilateral                 =             experimentparamsChoices.pre.trigger.angleTestBilateral
            experimentparamsTemp.pre.trigger.timeHold                           =             experimentparamsChoices.pre.trigger.timeHold
            experimentparamsTemp.pre.trigger.timeout                            =             experimentparamsChoices.pre.trigger.timeout
        
        experimentparamsTemp.pre.wait2                                          =             experimentparamsChoices.pre.wait2
        

        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        experimentparamsTemp.trial.robot.enabled                                =             experimentparamsChoices.trial.robot.enabled
        if (experimentparamsTemp.trial.robot.enabled):
            experimentparamsTemp.trial.robot.move.mode                          =             experimentparamsChoices.trial.robot.move.mode
            experimentparamsTemp.trial.robot.move.relative.tracking             = self.Choose(experimentparamsChoices.trial.robot.move.relative.tracking)                   
            experimentparamsTemp.trial.robot.move.relative.frameidOrigin        = self.Choose(experimentparamsChoices.trial.robot.move.relative.frameidOrigin)
            experimentparamsTemp.trial.robot.move.relative.distance             = self.Choose(experimentparamsChoices.trial.robot.move.relative.distance)
            experimentparamsTemp.trial.robot.move.relative.angleOffset          = self.Choose(experimentparamsChoices.trial.robot.move.relative.angleOffset)
            experimentparamsTemp.trial.robot.move.relative.angleVelocity        = self.Choose(experimentparamsChoices.trial.robot.move.relative.angleVelocity)
            experimentparamsTemp.trial.robot.move.relative.angleOscMag          = self.Choose(experimentparamsChoices.trial.robot.move.relative.angleOscMag)
            experimentparamsTemp.trial.robot.move.relative.angleOscFreq         = self.Choose(experimentparamsChoices.trial.robot.move.relative.angleOscFreq)
            experimentparamsTemp.trial.robot.move.relative.speedMax             = self.Choose(experimentparamsChoices.trial.robot.move.relative.speedMax)
            experimentparamsTemp.trial.robot.move.relative.tolerance            = self.Choose(experimentparamsChoices.trial.robot.move.relative.tolerance)
            experimentparamsTemp.trial.robot.move.relative.typeAngleOffset      = self.Choose(experimentparamsChoices.trial.robot.move.relative.typeAngleOffset)
            experimentparamsTemp.trial.robot.move.relative.typeAngleVelocity    = self.Choose(experimentparamsChoices.trial.robot.move.relative.typeAngleVelocity)
            experimentparamsTemp.trial.robot.move.relative.typeAngleOscMag      = self.Choose(experimentparamsChoices.trial.robot.move.relative.typeAngleOscMag)
            experimentparamsTemp.trial.robot.move.relative.typeAngleOscFreq     = self.Choose(experimentparamsChoices.trial.robot.move.relative.typeAngleOscFreq)
            experimentparamsTemp.trial.robot.move.relative.typeSpeedMax         = self.Choose(experimentparamsChoices.trial.robot.move.relative.typeSpeedMax)
            experimentparamsTemp.trial.robot.move.pattern.frameidPosition       = self.Choose(experimentparamsChoices.trial.robot.move.pattern.frameidPosition)
            experimentparamsTemp.trial.robot.move.pattern.frameidAngle          = self.Choose(experimentparamsChoices.trial.robot.move.pattern.frameidAngle)
            experimentparamsTemp.trial.robot.move.pattern.shape                 = self.Choose(experimentparamsChoices.trial.robot.move.pattern.shape)
            experimentparamsTemp.trial.robot.move.pattern.hzPattern             = self.Choose(experimentparamsChoices.trial.robot.move.pattern.hzPattern)
            experimentparamsTemp.trial.robot.move.pattern.hzPoint               = self.Choose(experimentparamsChoices.trial.robot.move.pattern.hzPoint)
            experimentparamsTemp.trial.robot.move.pattern.count                 = self.Choose(experimentparamsChoices.trial.robot.move.pattern.count)
            experimentparamsTemp.trial.robot.move.pattern.size                  = self.Choose(experimentparamsChoices.trial.robot.move.pattern.size)
            experimentparamsTemp.trial.robot.move.pattern.param                 = self.Choose(experimentparamsChoices.trial.robot.move.pattern.param)
            experimentparamsTemp.trial.robot.move.pattern.direction             = self.Choose(experimentparamsChoices.trial.robot.move.pattern.direction)
            experimentparamsTemp.trial.robot.move.pattern.restart               = self.Choose(experimentparamsChoices.trial.robot.move.pattern.restart)
        
        
        experimentparamsTemp.trial.lasergalvos                                  =             experimentparamsChoices.trial.lasergalvos

        
        experimentparamsTemp.trial.ledpanels.enabled                            =             experimentparamsChoices.trial.ledpanels.enabled
        if (experimentparamsTemp.trial.ledpanels.enabled):
            experimentparamsTemp.trial.ledpanels.command                        = self.Choose(experimentparamsChoices.trial.ledpanels.command)
            experimentparamsTemp.trial.ledpanels.idPattern                      = self.Choose(experimentparamsChoices.trial.ledpanels.idPattern)
            experimentparamsTemp.trial.ledpanels.origin                         = self.Choose(experimentparamsChoices.trial.ledpanels.origin)
            experimentparamsTemp.trial.ledpanels.frame_id                       = self.Choose(experimentparamsChoices.trial.ledpanels.frame_id)
            experimentparamsTemp.trial.ledpanels.statefilterHi                  = self.Choose(experimentparamsChoices.trial.ledpanels.statefilterHi)
            experimentparamsTemp.trial.ledpanels.statefilterLo                  = self.Choose(experimentparamsChoices.trial.ledpanels.statefilterLo)
            experimentparamsTemp.trial.ledpanels.statefilterCriteria            = self.Choose(experimentparamsChoices.trial.ledpanels.statefilterCriteria)

        experimentparamsTemp.post.trigger.enabled                               =             experimentparamsChoices.post.trigger.enabled
        if (experimentparamsTemp.post.trigger.enabled):
            experimentparamsTemp.post.trigger.frameidParent                     =             experimentparamsChoices.post.trigger.frameidParent
            experimentparamsTemp.post.trigger.frameidChild                      =             experimentparamsChoices.post.trigger.frameidChild
            experimentparamsTemp.post.trigger.speedAbsParentMin                 =             experimentparamsChoices.post.trigger.speedAbsParentMin
            experimentparamsTemp.post.trigger.speedAbsParentMax                 =             experimentparamsChoices.post.trigger.speedAbsParentMax
            experimentparamsTemp.post.trigger.speedAbsChildMin                  =             experimentparamsChoices.post.trigger.speedAbsChildMin
            experimentparamsTemp.post.trigger.speedAbsChildMax                  =             experimentparamsChoices.post.trigger.speedAbsChildMax
            experimentparamsTemp.post.trigger.speedRelMin                       =             experimentparamsChoices.post.trigger.speedRelMin
            experimentparamsTemp.post.trigger.speedRelMax                       =             experimentparamsChoices.post.trigger.speedRelMax
            experimentparamsTemp.post.trigger.distanceMin                       =             experimentparamsChoices.post.trigger.distanceMin
            experimentparamsTemp.post.trigger.distanceMax                       =             experimentparamsChoices.post.trigger.distanceMax
            experimentparamsTemp.post.trigger.angleMin                          =             experimentparamsChoices.post.trigger.angleMin
            experimentparamsTemp.post.trigger.angleMax                          =             experimentparamsChoices.post.trigger.angleMax
            experimentparamsTemp.post.trigger.angleTest                         =             experimentparamsChoices.post.trigger.angleTest
            experimentparamsTemp.post.trigger.angleTestBilateral                =             experimentparamsChoices.post.trigger.angleTestBilateral
            experimentparamsTemp.post.trigger.timeHold                          =             experimentparamsChoices.post.trigger.timeHold
            experimentparamsTemp.post.trigger.timeout                           =             experimentparamsChoices.post.trigger.timeout

        experimentparamsTemp.post.wait                                          =             experimentparamsChoices.post.wait


        
        #########################################
        # Use those temp values which are not None, otherwise keep the default values.
        experimentparams.experiment.description                             = (experimentparams.experiment.description, experimentparamsTemp.experiment.description)[experimentparamsTemp.experiment.description is not None]
        experimentparams.experiment.maxTrials                               = (experimentparams.experiment.maxTrials, experimentparamsTemp.experiment.maxTrials)[experimentparamsTemp.experiment.maxTrials is not None]
        experimentparams.experiment.timeout                                 = (experimentparams.experiment.timeout, experimentparamsTemp.experiment.timeout)[experimentparamsTemp.experiment.timeout is not None]
        
        experimentparams.save.filenamebase                                  = (experimentparams.save.filenamebase, experimentparamsTemp.save.filenamebase)[experimentparamsTemp.save.filenamebase is not None]
        experimentparams.save.csv                                           = (experimentparams.save.csv, experimentparamsTemp.save.csv)[experimentparamsTemp.save.csv is not None]
        experimentparams.save.bag                                           = (experimentparams.save.bag, experimentparamsTemp.save.bag)[experimentparamsTemp.save.bag is not None]
        experimentparams.save.mov                                           = (experimentparams.save.mov, experimentparamsTemp.save.mov)[experimentparamsTemp.save.mov is not None]
        experimentparams.save.fmf                                           = (experimentparams.save.fmf, experimentparamsTemp.save.fmf)[experimentparamsTemp.save.fmf is not None]
        experimentparams.save.imagetopic_list                               = (experimentparams.save.imagetopic_list, experimentparamsTemp.save.imagetopic_list)[experimentparamsTemp.save.imagetopic_list is not None]
        experimentparams.save.onlyWhileTriggered                            = (experimentparams.save.onlyWhileTriggered, experimentparamsTemp.save.onlyWhileTriggered)[experimentparamsTemp.save.onlyWhileTriggered is not None]
        
        experimentparams.robotspec.nRobots                                  = (experimentparams.robotspec.nRobots, experimentparamsTemp.robotspec.nRobots)[experimentparamsTemp.robotspec.nRobots is not None]
        experimentparams.robotspec.width                                    = (experimentparams.robotspec.width, experimentparamsTemp.robotspec.width)[experimentparamsTemp.robotspec.width is not None]
        experimentparams.robotspec.height                                   = (experimentparams.robotspec.height, experimentparamsTemp.robotspec.height)[experimentparamsTemp.robotspec.height is not None]
        experimentparams.robotspec.isPresent                                = (experimentparams.robotspec.isPresent, experimentparamsTemp.robotspec.isPresent)[experimentparamsTemp.robotspec.isPresent is not None]
        experimentparams.robotspec.description                              = (experimentparams.robotspec.description, experimentparamsTemp.robotspec.description)[experimentparamsTemp.robotspec.description is not None]

        experimentparams.flyspec.nFlies                                     = (experimentparams.flyspec.nFlies, experimentparamsTemp.flyspec.nFlies)[experimentparamsTemp.flyspec.nFlies is not None]
        experimentparams.flyspec.description                                = (experimentparams.flyspec.description, experimentparamsTemp.flyspec.description)[experimentparamsTemp.flyspec.description is not None]
        
        experimentparams.tracking.exclusionzones.enabled                    = (experimentparams.tracking.exclusionzones.enabled, experimentparamsTemp.tracking.exclusionzones.enabled)[experimentparamsTemp.tracking.exclusionzones.enabled is not None]
        experimentparams.tracking.exclusionzones.point_list                 = (experimentparams.tracking.exclusionzones.point_list, experimentparamsTemp.tracking.exclusionzones.point_list)[experimentparamsTemp.tracking.exclusionzones.point_list is not None]
        experimentparams.tracking.exclusionzones.radius_list                = (experimentparams.tracking.exclusionzones.radius_list, experimentparamsTemp.tracking.exclusionzones.radius_list)[experimentparamsTemp.tracking.exclusionzones.radius_list is not None]
        
        experimentparams.home.enabled                                       = (experimentparams.home.enabled, experimentparamsTemp.home.enabled)[experimentparamsTemp.home.enabled is not None]
        if (experimentparams.home.enabled):
            experimentparams.home.x                                         = (experimentparams.home.x, experimentparamsTemp.home.x)[experimentparamsTemp.home.x is not None]
            experimentparams.home.y                                         = (experimentparams.home.y, experimentparamsTemp.home.y)[experimentparamsTemp.home.y is not None]
            experimentparams.home.speed                                     = (experimentparams.home.speed, experimentparamsTemp.home.speed)[experimentparamsTemp.home.speed is not None]
            experimentparams.home.tolerance                                 = (experimentparams.home.tolerance, experimentparamsTemp.home.tolerance)[experimentparamsTemp.home.tolerance is not None]

        experimentparams.pre.robot.enabled                                  = (experimentparams.pre.robot.enabled, experimentparamsTemp.pre.robot.enabled)[experimentparamsTemp.pre.robot.enabled is not None]
        if (experimentparams.pre.robot.enabled):
            experimentparams.pre.robot.move.mode                            = (experimentparams.pre.robot.move.mode, experimentparamsTemp.pre.robot.move.mode)[experimentparamsTemp.pre.robot.move.mode is not None]
            experimentparams.pre.robot.move.relative.tracking               = (experimentparams.pre.robot.move.relative.tracking, experimentparamsTemp.pre.robot.move.relative.tracking)[experimentparamsTemp.pre.robot.move.relative.tracking is not None]                   
            experimentparams.pre.robot.move.relative.frameidOrigin          = (experimentparams.pre.robot.move.relative.frameidOrigin, experimentparamsTemp.pre.robot.move.relative.frameidOrigin)[experimentparamsTemp.pre.robot.move.relative.frameidOrigin is not None]
            experimentparams.pre.robot.move.relative.distance               = (experimentparams.pre.robot.move.relative.distance, experimentparamsTemp.pre.robot.move.relative.distance)[experimentparamsTemp.pre.robot.move.relative.distance is not None]
            experimentparams.pre.robot.move.relative.angleOffset            = (experimentparams.pre.robot.move.relative.angleOffset, experimentparamsTemp.pre.robot.move.relative.angleOffset)[experimentparamsTemp.pre.robot.move.relative.angleOffset is not None]
            experimentparams.pre.robot.move.relative.angleVelocity          = (experimentparams.pre.robot.move.relative.angleVelocity, experimentparamsTemp.pre.robot.move.relative.angleVelocity)[experimentparamsTemp.pre.robot.move.relative.angleVelocity is not None]
            experimentparams.pre.robot.move.relative.angleOscMag            = (experimentparams.pre.robot.move.relative.angleOscMag, experimentparamsTemp.pre.robot.move.relative.angleOscMag)[experimentparamsTemp.pre.robot.move.relative.angleOscMag is not None]
            experimentparams.pre.robot.move.relative.angleOscFreq           = (experimentparams.pre.robot.move.relative.angleOscFreq, experimentparamsTemp.pre.robot.move.relative.angleOscFreq)[experimentparamsTemp.pre.robot.move.relative.angleOscFreq is not None]
            experimentparams.pre.robot.move.relative.speedMax               = (experimentparams.pre.robot.move.relative.speedMax, experimentparamsTemp.pre.robot.move.relative.speedMax)[experimentparamsTemp.pre.robot.move.relative.speedMax is not None]
            experimentparams.pre.robot.move.relative.tolerance              = (experimentparams.pre.robot.move.relative.tolerance, experimentparamsTemp.pre.robot.move.relative.tolerance)[experimentparamsTemp.pre.robot.move.relative.tolerance is not None]
            experimentparams.pre.robot.move.relative.typeAngleOffset        = (experimentparams.pre.robot.move.relative.typeAngleOffset, experimentparamsTemp.pre.robot.move.relative.typeAngleOffset)[experimentparamsTemp.pre.robot.move.relative.typeAngleOffset is not None]
            experimentparams.pre.robot.move.relative.typeAngleVelocity      = (experimentparams.pre.robot.move.relative.typeAngleVelocity, experimentparamsTemp.pre.robot.move.relative.typeAngleVelocity)[experimentparamsTemp.pre.robot.move.relative.typeAngleVelocity is not None]
            experimentparams.pre.robot.move.relative.typeAngleOscMag        = (experimentparams.pre.robot.move.relative.typeAngleOscMag, experimentparamsTemp.pre.robot.move.relative.typeAngleOscMag)[experimentparamsTemp.pre.robot.move.relative.typeAngleOscMag is not None]
            experimentparams.pre.robot.move.relative.typeAngleOscFreq       = (experimentparams.pre.robot.move.relative.typeAngleOscFreq, experimentparamsTemp.pre.robot.move.relative.typeAngleOscFreq)[experimentparamsTemp.pre.robot.move.relative.typeAngleOscFreq is not None]
            experimentparams.pre.robot.move.relative.typeSpeedMax           = (experimentparams.pre.robot.move.relative.typeSpeedMax, experimentparamsTemp.pre.robot.move.relative.typeSpeedMax)[experimentparamsTemp.pre.robot.move.relative.typeSpeedMax is not None]
            experimentparams.pre.robot.move.pattern.frameidPosition         = (experimentparams.pre.robot.move.pattern.frameidPosition, experimentparamsTemp.pre.robot.move.pattern.frameidPosition)[experimentparamsTemp.pre.robot.move.pattern.frameidPosition is not None]
            experimentparams.pre.robot.move.pattern.frameidAngle            = (experimentparams.pre.robot.move.pattern.frameidAngle, experimentparamsTemp.pre.robot.move.pattern.frameidAngle)[experimentparamsTemp.pre.robot.move.pattern.frameidAngle is not None]
            experimentparams.pre.robot.move.pattern.shape                   = (experimentparams.pre.robot.move.pattern.shape, experimentparamsTemp.pre.robot.move.pattern.shape)[experimentparamsTemp.pre.robot.move.pattern.shape is not None]
            experimentparams.pre.robot.move.pattern.hzPattern               = (experimentparams.pre.robot.move.pattern.hzPattern, experimentparamsTemp.pre.robot.move.pattern.hzPattern)[experimentparamsTemp.pre.robot.move.pattern.hzPattern is not None]
            experimentparams.pre.robot.move.pattern.hzPoint                 = (experimentparams.pre.robot.move.pattern.hzPoint, experimentparamsTemp.pre.robot.move.pattern.hzPoint)[experimentparamsTemp.pre.robot.move.pattern.hzPoint is not None]
            experimentparams.pre.robot.move.pattern.count                   = (experimentparams.pre.robot.move.pattern.count, experimentparamsTemp.pre.robot.move.pattern.count)[experimentparamsTemp.pre.robot.move.pattern.count is not None]
            experimentparams.pre.robot.move.pattern.size                    = (experimentparams.pre.robot.move.pattern.size, experimentparamsTemp.pre.robot.move.pattern.size)[experimentparamsTemp.pre.robot.move.pattern.size is not None]
            experimentparams.pre.robot.move.pattern.param                   = (experimentparams.pre.robot.move.pattern.param, experimentparamsTemp.pre.robot.move.pattern.param)[experimentparamsTemp.pre.robot.move.pattern.param is not None]
            experimentparams.pre.robot.move.pattern.direction               = (experimentparams.pre.robot.move.pattern.direction, experimentparamsTemp.pre.robot.move.pattern.direction)[experimentparamsTemp.pre.robot.move.pattern.direction is not None]
            experimentparams.pre.robot.move.pattern.restart                 = (experimentparams.pre.robot.move.pattern.restart, experimentparamsTemp.pre.robot.move.pattern.restart)[experimentparamsTemp.pre.robot.move.pattern.restart is not None]
        
        experimentparams.pre.lasergalvos                                    = (experimentparams.pre.lasergalvos, experimentparamsTemp.pre.lasergalvos)[experimentparamsTemp.pre.lasergalvos is not None]
        
        experimentparams.pre.ledpanels.enabled                              = (experimentparams.pre.ledpanels.enabled, experimentparamsTemp.pre.ledpanels.enabled)[experimentparamsTemp.pre.ledpanels.enabled is not None]
        if (experimentparams.pre.ledpanels.enabled):
            experimentparams.pre.ledpanels.command                          = (experimentparams.pre.ledpanels.command, experimentparamsTemp.pre.ledpanels.command)[experimentparamsTemp.pre.ledpanels.command is not None]
            experimentparams.pre.ledpanels.idPattern                        = (experimentparams.pre.ledpanels.idPattern, experimentparamsTemp.pre.ledpanels.idPattern)[experimentparamsTemp.pre.ledpanels.idPattern is not None]
            experimentparams.pre.ledpanels.origin                           = (experimentparams.pre.ledpanels.origin, experimentparamsTemp.pre.ledpanels.origin)[experimentparamsTemp.pre.ledpanels.origin is not None]
            experimentparams.pre.ledpanels.frame_id                         = (experimentparams.pre.ledpanels.frame_id, experimentparamsTemp.pre.ledpanels.frame_id)[experimentparamsTemp.pre.ledpanels.frame_id is not None]
            experimentparams.pre.ledpanels.statefilterHi                    = (experimentparams.pre.ledpanels.statefilterHi, experimentparamsTemp.pre.ledpanels.statefilterHi)[experimentparamsTemp.pre.ledpanels.statefilterHi is not None]
            experimentparams.pre.ledpanels.statefilterLo                    = (experimentparams.pre.ledpanels.statefilterLo, experimentparamsTemp.pre.ledpanels.statefilterLo)[experimentparamsTemp.pre.ledpanels.statefilterLo is not None]
            experimentparams.pre.ledpanels.statefilterCriteria              = (experimentparams.pre.ledpanels.statefilterCriteria, experimentparamsTemp.pre.ledpanels.statefilterCriteria)[experimentparamsTemp.pre.ledpanels.statefilterCriteria is not None]
        
        experimentparams.pre.wait1                                          = (experimentparams.pre.wait1, experimentparamsTemp.pre.wait1)[experimentparamsTemp.pre.wait1 is not None]
        
        experimentparams.pre.trigger.enabled                                = (experimentparams.pre.trigger.enabled, experimentparamsTemp.pre.trigger.enabled)[experimentparamsTemp.pre.trigger.enabled is not None]
        if (experimentparams.pre.trigger.enabled):
            experimentparams.pre.trigger.frameidParent                      = (experimentparams.pre.trigger.frameidParent, experimentparamsTemp.pre.trigger.frameidParent)[experimentparamsTemp.pre.trigger.frameidParent is not None]
            experimentparams.pre.trigger.frameidChild                       = (experimentparams.pre.trigger.frameidChild, experimentparamsTemp.pre.trigger.frameidChild)[experimentparamsTemp.pre.trigger.frameidChild is not None]
            experimentparams.pre.trigger.speedAbsParentMin                  = (experimentparams.pre.trigger.speedAbsParentMin, experimentparamsTemp.pre.trigger.speedAbsParentMin)[experimentparamsTemp.pre.trigger.speedAbsParentMin is not None]
            experimentparams.pre.trigger.speedAbsParentMax                  = (experimentparams.pre.trigger.speedAbsParentMax, experimentparamsTemp.pre.trigger.speedAbsParentMax)[experimentparamsTemp.pre.trigger.speedAbsParentMax is not None]
            experimentparams.pre.trigger.speedAbsChildMin                   = (experimentparams.pre.trigger.speedAbsChildMin, experimentparamsTemp.pre.trigger.speedAbsChildMin)[experimentparamsTemp.pre.trigger.speedAbsChildMin is not None]
            experimentparams.pre.trigger.speedAbsChildMax                   = (experimentparams.pre.trigger.speedAbsChildMax, experimentparamsTemp.pre.trigger.speedAbsChildMax)[experimentparamsTemp.pre.trigger.speedAbsChildMax is not None]
            experimentparams.pre.trigger.speedRelMin                        = (experimentparams.pre.trigger.speedRelMin, experimentparamsTemp.pre.trigger.speedRelMin)[experimentparamsTemp.pre.trigger.speedRelMin is not None]
            experimentparams.pre.trigger.speedRelMax                        = (experimentparams.pre.trigger.speedRelMax, experimentparamsTemp.pre.trigger.speedRelMax)[experimentparamsTemp.pre.trigger.speedRelMax is not None]
            experimentparams.pre.trigger.distanceMin                        = (experimentparams.pre.trigger.distanceMin, experimentparamsTemp.pre.trigger.distanceMin)[experimentparamsTemp.pre.trigger.distanceMin is not None]
            experimentparams.pre.trigger.distanceMax                        = (experimentparams.pre.trigger.distanceMax, experimentparamsTemp.pre.trigger.distanceMax)[experimentparamsTemp.pre.trigger.distanceMax is not None]
            experimentparams.pre.trigger.angleMin                           = (experimentparams.pre.trigger.angleMin, experimentparamsTemp.pre.trigger.angleMin)[experimentparamsTemp.pre.trigger.angleMin is not None]
            experimentparams.pre.trigger.angleMax                           = (experimentparams.pre.trigger.angleMax, experimentparamsTemp.pre.trigger.angleMax)[experimentparamsTemp.pre.trigger.angleMax is not None]
            experimentparams.pre.trigger.angleTest                          = (experimentparams.pre.trigger.angleTest, experimentparamsTemp.pre.trigger.angleTest)[experimentparamsTemp.pre.trigger.angleTest is not None]
            experimentparams.pre.trigger.angleTestBilateral                 = (experimentparams.pre.trigger.angleTestBilateral, experimentparamsTemp.pre.trigger.angleTestBilateral)[experimentparamsTemp.pre.trigger.angleTestBilateral is not None]
            experimentparams.pre.trigger.timeHold                           = (experimentparams.pre.trigger.timeHold, experimentparamsTemp.pre.trigger.timeHold)[experimentparamsTemp.pre.trigger.timeHold is not None]
            experimentparams.pre.trigger.timeout                            = (experimentparams.pre.trigger.timeout, experimentparamsTemp.pre.trigger.timeout)[experimentparamsTemp.pre.trigger.timeout is not None]
        
        experimentparams.pre.wait2                                          = (experimentparams.pre.wait2, experimentparamsTemp.pre.wait2)[experimentparamsTemp.pre.wait2 is not None]
        

        # .robot, .lasergalvos, .ledpanels, and .post.trigger all run concurrently.
        # The first one to finish preempts the others.
        experimentparams.trial.robot.enabled                                = (experimentparams.trial.robot.enabled, experimentparamsTemp.trial.robot.enabled)[experimentparamsTemp.trial.robot.enabled is not None]
        if (experimentparams.trial.robot.enabled):
            experimentparams.trial.robot.move.mode                          = (experimentparams.trial.robot.move.mode, experimentparamsTemp.trial.robot.move.mode)[experimentparamsTemp.trial.robot.move.mode is not None]
            experimentparams.trial.robot.move.relative.tracking             = (experimentparams.trial.robot.move.relative.tracking, experimentparamsTemp.trial.robot.move.relative.tracking)[experimentparamsTemp.trial.robot.move.relative.tracking is not None]                   
            experimentparams.trial.robot.move.relative.frameidOrigin        = (experimentparams.trial.robot.move.relative.frameidOrigin, experimentparamsTemp.trial.robot.move.relative.frameidOrigin)[experimentparamsTemp.trial.robot.move.relative.frameidOrigin is not None]
            experimentparams.trial.robot.move.relative.distance             = (experimentparams.trial.robot.move.relative.distance, experimentparamsTemp.trial.robot.move.relative.distance)[experimentparamsTemp.trial.robot.move.relative.distance is not None]
            experimentparams.trial.robot.move.relative.angleOffset          = (experimentparams.trial.robot.move.relative.angleOffset, experimentparamsTemp.trial.robot.move.relative.angleOffset)[experimentparamsTemp.trial.robot.move.relative.angleOffset is not None]
            experimentparams.trial.robot.move.relative.angleVelocity        = (experimentparams.trial.robot.move.relative.angleVelocity, experimentparamsTemp.trial.robot.move.relative.angleVelocity)[experimentparamsTemp.trial.robot.move.relative.angleVelocity is not None]
            experimentparams.trial.robot.move.relative.angleOscMag          = (experimentparams.trial.robot.move.relative.angleOscMag, experimentparamsTemp.trial.robot.move.relative.angleOscMag)[experimentparamsTemp.trial.robot.move.relative.angleOscMag is not None]
            experimentparams.trial.robot.move.relative.angleOscFreq         = (experimentparams.trial.robot.move.relative.angleOscFreq, experimentparamsTemp.trial.robot.move.relative.angleOscFreq)[experimentparamsTemp.trial.robot.move.relative.angleOscFreq is not None]
            experimentparams.trial.robot.move.relative.speedMax             = (experimentparams.trial.robot.move.relative.speedMax, experimentparamsTemp.trial.robot.move.relative.speedMax)[experimentparamsTemp.trial.robot.move.relative.speedMax is not None]
            experimentparams.trial.robot.move.relative.tolerance            = (experimentparams.trial.robot.move.relative.tolerance, experimentparamsTemp.trial.robot.move.relative.tolerance)[experimentparamsTemp.trial.robot.move.relative.tolerance is not None]
            experimentparams.trial.robot.move.relative.typeAngleOffset      = (experimentparams.trial.robot.move.relative.typeAngleOffset, experimentparamsTemp.trial.robot.move.relative.typeAngleOffset)[experimentparamsTemp.trial.robot.move.relative.typeAngleOffset is not None]
            experimentparams.trial.robot.move.relative.typeAngleVelocity    = (experimentparams.trial.robot.move.relative.typeAngleVelocity, experimentparamsTemp.trial.robot.move.relative.typeAngleVelocity)[experimentparamsTemp.trial.robot.move.relative.typeAngleVelocity is not None]
            experimentparams.trial.robot.move.relative.typeAngleOscMag      = (experimentparams.trial.robot.move.relative.typeAngleOscMag, experimentparamsTemp.trial.robot.move.relative.typeAngleOscMag)[experimentparamsTemp.trial.robot.move.relative.typeAngleOscMag is not None]
            experimentparams.trial.robot.move.relative.typeAngleOscFreq     = (experimentparams.trial.robot.move.relative.typeAngleOscFreq, experimentparamsTemp.trial.robot.move.relative.typeAngleOscFreq)[experimentparamsTemp.trial.robot.move.relative.typeAngleOscFreq is not None]
            experimentparams.trial.robot.move.relative.typeSpeedMax         = (experimentparams.trial.robot.move.relative.typeSpeedMax, experimentparamsTemp.trial.robot.move.relative.typeSpeedMax)[experimentparamsTemp.trial.robot.move.relative.typeSpeedMax is not None]
            experimentparams.trial.robot.move.pattern.frameidPosition       = (experimentparams.trial.robot.move.pattern.frameidPosition, experimentparamsTemp.trial.robot.move.pattern.frameidPosition)[experimentparamsTemp.trial.robot.move.pattern.frameidPosition is not None]
            experimentparams.trial.robot.move.pattern.frameidAngle          = (experimentparams.trial.robot.move.pattern.frameidAngle, experimentparamsTemp.trial.robot.move.pattern.frameidAngle)[experimentparamsTemp.trial.robot.move.pattern.frameidAngle is not None]
            experimentparams.trial.robot.move.pattern.shape                 = (experimentparams.trial.robot.move.pattern.shape, experimentparamsTemp.trial.robot.move.pattern.shape)[experimentparamsTemp.trial.robot.move.pattern.shape is not None]
            experimentparams.trial.robot.move.pattern.hzPattern             = (experimentparams.trial.robot.move.pattern.hzPattern, experimentparamsTemp.trial.robot.move.pattern.hzPattern)[experimentparamsTemp.trial.robot.move.pattern.hzPattern is not None]
            experimentparams.trial.robot.move.pattern.hzPoint               = (experimentparams.trial.robot.move.pattern.hzPoint, experimentparamsTemp.trial.robot.move.pattern.hzPoint)[experimentparamsTemp.trial.robot.move.pattern.hzPoint is not None]
            experimentparams.trial.robot.move.pattern.count                 = (experimentparams.trial.robot.move.pattern.count, experimentparamsTemp.trial.robot.move.pattern.count)[experimentparamsTemp.trial.robot.move.pattern.count is not None]
            experimentparams.trial.robot.move.pattern.size                  = (experimentparams.trial.robot.move.pattern.size, experimentparamsTemp.trial.robot.move.pattern.size)[experimentparamsTemp.trial.robot.move.pattern.size is not None]
            experimentparams.trial.robot.move.pattern.param                 = (experimentparams.trial.robot.move.pattern.param, experimentparamsTemp.trial.robot.move.pattern.param)[experimentparamsTemp.trial.robot.move.pattern.param is not None]
            experimentparams.trial.robot.move.pattern.direction             = (experimentparams.trial.robot.move.pattern.direction, experimentparamsTemp.trial.robot.move.pattern.direction)[experimentparamsTemp.trial.robot.move.pattern.direction is not None]
            experimentparams.trial.robot.move.pattern.restart               = (experimentparams.trial.robot.move.pattern.restart, experimentparamsTemp.trial.robot.move.pattern.restart)[experimentparamsTemp.trial.robot.move.pattern.restart is not None]
        
        
        experimentparams.trial.lasergalvos                                  = (experimentparams.trial.lasergalvos, experimentparamsTemp.trial.lasergalvos)[experimentparamsTemp.trial.lasergalvos is not None]

        
        experimentparams.trial.ledpanels.enabled                            = (experimentparams.trial.ledpanels.enabled, experimentparamsTemp.trial.ledpanels.enabled)[experimentparamsTemp.trial.ledpanels.enabled is not None]
        if (experimentparams.trial.ledpanels.enabled):
            experimentparams.trial.ledpanels.command                        = (experimentparams.trial.ledpanels.command, experimentparamsTemp.trial.ledpanels.command)[experimentparamsTemp.trial.ledpanels.command is not None]
            experimentparams.trial.ledpanels.idPattern                      = (experimentparams.trial.ledpanels.idPattern, experimentparamsTemp.trial.ledpanels.idPattern)[experimentparamsTemp.trial.ledpanels.idPattern is not None]
            experimentparams.trial.ledpanels.origin                         = (experimentparams.trial.ledpanels.origin, experimentparamsTemp.trial.ledpanels.origin)[experimentparamsTemp.trial.ledpanels.origin is not None]
            experimentparams.trial.ledpanels.frame_id                       = (experimentparams.trial.ledpanels.frame_id, experimentparamsTemp.trial.ledpanels.frame_id)[experimentparamsTemp.trial.ledpanels.frame_id is not None]
            experimentparams.trial.ledpanels.statefilterHi                  = (experimentparams.trial.ledpanels.statefilterHi, experimentparamsTemp.trial.ledpanels.statefilterHi)[experimentparamsTemp.trial.ledpanels.statefilterHi is not None]
            experimentparams.trial.ledpanels.statefilterLo                  = (experimentparams.trial.ledpanels.statefilterLo, experimentparamsTemp.trial.ledpanels.statefilterLo)[experimentparamsTemp.trial.ledpanels.statefilterLo is not None]
            experimentparams.trial.ledpanels.statefilterCriteria            = (experimentparams.trial.ledpanels.statefilterCriteria, experimentparamsTemp.trial.ledpanels.statefilterCriteria)[experimentparamsTemp.trial.ledpanels.statefilterCriteria is not None]

        experimentparams.post.trigger.enabled                               = (experimentparams.post.trigger.enabled, experimentparamsTemp.post.trigger.enabled)[experimentparamsTemp.post.trigger.enabled is not None]
        if (experimentparams.post.trigger.enabled):
            experimentparams.post.trigger.frameidParent                     = (experimentparams.post.trigger.frameidParent, experimentparamsTemp.post.trigger.frameidParent)[experimentparamsTemp.post.trigger.frameidParent is not None]
            experimentparams.post.trigger.frameidChild                      = (experimentparams.post.trigger.frameidChild, experimentparamsTemp.post.trigger.frameidChild)[experimentparamsTemp.post.trigger.frameidChild is not None]
            experimentparams.post.trigger.speedAbsParentMin                 = (experimentparams.post.trigger.speedAbsParentMin, experimentparamsTemp.post.trigger.speedAbsParentMin)[experimentparamsTemp.post.trigger.speedAbsParentMin is not None]
            experimentparams.post.trigger.speedAbsParentMax                 = (experimentparams.post.trigger.speedAbsParentMax, experimentparamsTemp.post.trigger.speedAbsParentMax)[experimentparamsTemp.post.trigger.speedAbsParentMax is not None]
            experimentparams.post.trigger.speedAbsChildMin                  = (experimentparams.post.trigger.speedAbsChildMin, experimentparamsTemp.post.trigger.speedAbsChildMin)[experimentparamsTemp.post.trigger.speedAbsChildMin is not None]
            experimentparams.post.trigger.speedAbsChildMax                  = (experimentparams.post.trigger.speedAbsChildMax, experimentparamsTemp.post.trigger.speedAbsChildMax)[experimentparamsTemp.post.trigger.speedAbsChildMax is not None]
            experimentparams.post.trigger.speedRelMin                       = (experimentparams.post.trigger.speedRelMin, experimentparamsTemp.post.trigger.speedRelMin)[experimentparamsTemp.post.trigger.speedRelMin is not None]
            experimentparams.post.trigger.speedRelMax                       = (experimentparams.post.trigger.speedRelMax, experimentparamsTemp.post.trigger.speedRelMax)[experimentparamsTemp.post.trigger.speedRelMax is not None]
            experimentparams.post.trigger.distanceMin                       = (experimentparams.post.trigger.distanceMin, experimentparamsTemp.post.trigger.distanceMin)[experimentparamsTemp.post.trigger.distanceMin is not None]
            experimentparams.post.trigger.distanceMax                       = (experimentparams.post.trigger.distanceMax, experimentparamsTemp.post.trigger.distanceMax)[experimentparamsTemp.post.trigger.distanceMax is not None]
            experimentparams.post.trigger.angleMin                          = (experimentparams.post.trigger.angleMin, experimentparamsTemp.post.trigger.angleMin)[experimentparamsTemp.post.trigger.angleMin is not None]
            experimentparams.post.trigger.angleMax                          = (experimentparams.post.trigger.angleMax, experimentparamsTemp.post.trigger.angleMax)[experimentparamsTemp.post.trigger.angleMax is not None]
            experimentparams.post.trigger.angleTest                         = (experimentparams.post.trigger.angleTest, experimentparamsTemp.post.trigger.angleTest)[experimentparamsTemp.post.trigger.angleTest is not None]
            experimentparams.post.trigger.angleTestBilateral                = (experimentparams.post.trigger.angleTestBilateral, experimentparamsTemp.post.trigger.angleTestBilateral)[experimentparamsTemp.post.trigger.angleTestBilateral is not None]
            experimentparams.post.trigger.timeHold                          = (experimentparams.post.trigger.timeHold, experimentparamsTemp.post.trigger.timeHold)[experimentparamsTemp.post.trigger.timeHold is not None]
            experimentparams.post.trigger.timeout                           = (experimentparams.post.trigger.timeout, experimentparamsTemp.post.trigger.timeout)[experimentparamsTemp.post.trigger.timeout is not None]

        experimentparams.post.wait                                          = (experimentparams.post.wait, experimentparamsTemp.post.wait)[experimentparamsTemp.post.wait is not None]
        
        return experimentparams
        
        
    def execute(self, userdata):
        rv = 'exit'
        now = rospy.Time.now().to_sec()
        experimentparamsChoices = userdata.experimentparamsChoicesIn
        experimentparams = self.GetChoice(experimentparamsChoices)
        

        # Set the start time of the trial.   # in "experiment time", i.e. t=0 is start of experiment.
        experimentparams.experiment.timeTrialElapsed = 0.0 #g_timeExperimentElapsed

        # Increment the trial counter, and check for end.        
        experimentparams.experiment.trial = userdata.experimentparamsChoicesIn.experiment.trial+1
        if (experimentparams.experiment.maxTrials != -1):
            if (experimentparams.experiment.maxTrials < experimentparams.experiment.trial):
                return rv

        # Check for experiment timeout.
        if (experimentparams.experiment.timeout != -1):
            if (experimentparams.experiment.timeout <= g_timeExperimentElapsed):
                return rv

        # Make the timestamp.
        experimentparams.save.timestamp = '%04d%02d%02d%02d%02d%02d' % (time.localtime(now).tm_year,
                                                                        time.localtime(now).tm_mon,
                                                                        time.localtime(now).tm_mday,
                                                                        time.localtime(now).tm_hour,
                                                                        time.localtime(now).tm_min,
                                                                        time.localtime(now).tm_sec)
        

        rospy.logwarn ('EL State StartTrial(%s)' % experimentparams.experiment.trial)


        # Set the output data.
        userdata.experimentparamsOut            = experimentparams

        # Set up the tracking.    
        msgTrackingCommand                      = TrackingCommand()
        msgTrackingCommand.command              = 'initialize'
        msgTrackingCommand.exclusionzones       = experimentparams.tracking.exclusionzones
        msgTrackingCommand.nRobots              = experimentparams.robotspec.nRobots
        msgTrackingCommand.nFlies               = experimentparams.flyspec.nFlies
        msgTrackingCommand.bUseVisualServoing   = experimentparams.robotspec.isPresent
        
        self.pubTrackingCommand.publish(msgTrackingCommand)
        

        # Tell everyone that the trial is starting.
        try:
            self.TrialStartServices.notify(experimentparams)
            rv = 'continue'
        except (rospy.ServiceException, Exception), e:
            rospy.logwarn('self.TrialStartServices.notify(): %s' % e)
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
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])
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
            distance = np.linalg.norm([pointP.point.x, pointP.point.y, pointP.point.z])
            
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
                    stamp = self.tfrx.getLatestCommonTime(frameidParent, frameidChild) - self.dtVelocity
                    ((vx,vy,vz),(wx,wy,wz)) = self.tfrx.lookupTwist(frameidChild, frameidParent, stamp, self.dtVelocity)
                except (tf.Exception, AttributeError, TypeError), e:
                    ((vx,vy,vz),(wx,wy,wz)) = ((0,0,0),(0,0,0))
    
                speed = np.linalg.norm(np.array([vx,vy,vz]))
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
            angleToChild = np.arctan2(pointP.point.y, pointP.point.x) % (2.0*np.pi)
            
        return angleToChild



    # step_time_elapsed()
    # Increment the elapsed time, adjusting for the pause/continue status of the experiment.
    #
    def step_time_elapsed(self):
        global g_timeExperimentElapsed
        
        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.timeTriggerElapsed += dt
        self.experimentparams.experiment.timeTrialElapsed += dt.to_sec()
        g_timeExperimentElapsed += dt.to_sec()

        
        
    def execute(self, userdata):
        rospy.loginfo('EL State TriggerOnStates(%s)' % (self.mode))


        self.experimentparams = copy.deepcopy(userdata.experimentparamsIn)
        self.nRobots = self.experimentparams.robotspec.nRobots
        
        if self.mode == 'pre':
            trigger = self.experimentparams.pre.trigger
        else:
            trigger = self.experimentparams.post.trigger

        rv = 'disabled'
        if (trigger.enabled):
            self.timePrev = rospy.Time.now()
            self.timeTriggerElapsed = rospy.Time(0)
            
            self.isTriggered = False
            self.timeTriggered  = None
            
            # Wait for an arenastate.
            while (self.arenastate is None):
                self.step_time_elapsed()
                if (trigger.timeout != -1):
                    if ((self.timeTriggerElapsed.to_sec()) > trigger.timeout):
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
                    
                    angleA1 = trigger.angleMin % (2.0*np.pi)
                    angleA2 = trigger.angleMax % (2.0*np.pi)
                    angleB1 = (2.0*np.pi - angleA2) # % (2.0*np.pi)
                    angleB2 = (2.0*np.pi - angleA1) # % (2.0*np.pi)
    
                    if angle is not None:
                        # Test for angle meeting the angle criteria.
                        isAngleInRange = False
                        if trigger.angleTestBilateral:
                            if trigger.angleTest=='inclusive':
                                #rospy.loginfo('EL angles %s' % [angleA1, angleA2, angleB1, angleB2, angle])
                                if (angleA1 <= angle <= angleA2) or (angleB1 <= angle <= angleB2):
                                    isAngleInRange = True
                                    
                            elif trigger.angleTest=='exclusive':
                                if (0.0 <= angle < angleA1) or (angleA2 < angle < angleB1) or (angleB2 < angle <= (2.0*np.pi)):
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
                        self.timeTriggered = self.timeTriggerElapsed
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
                    duration = self.timeTriggerElapsed.to_sec() - self.timeTriggered.to_sec()
                    
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
                
                # Check for trigger timeout.
                if (trigger.timeout != -1):
                    if (trigger.timeout < self.timeTriggerElapsed.to_sec()):
                        rv = 'timeout'
                        break

                # Check for experiment timeout.
                if (self.experimentparams.experiment.timeout != -1):
                    if (self.experimentparams.experiment.timeout <= g_timeExperimentElapsed):
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
        userdata.experimentparamsOut = self.experimentparams
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
                             input_keys=['experimentparamsIn'],
                             output_keys=['experimentparamsOut'])

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
        global g_timeExperimentElapsed

        timeNow = rospy.Time.now()
        if (self.commandExperiment != 'pause_now'):
            dt = timeNow - self.timePrev
        else:
            dt = rospy.Duration(0)

        self.timePrev = timeNow
        self.timeTriggerElapsed += dt
        self.experimentparams.experiment.timeTrialElapsed += dt.to_sec()
        g_timeExperimentElapsed += dt.to_sec()


        
        
    def execute(self, userdata):
        self.experimentparams = copy.deepcopy(userdata.experimentparamsIn)
        
        if self.mode=='pre1':
            timeout = self.experimentparams.pre.wait1
        elif self.mode=='pre2':
            timeout = self.experimentparams.pre.wait2
        elif self.mode=='post':
            timeout = self.experimentparams.post.wait
        else:
            rospy.logwarn ('TriggerOnTime mode must be one of: pre1, pre2, post.')
            

        rospy.loginfo('EL State TriggerOnTime(%s, %s)' % (self.mode, timeout))

        self.timePrev = rospy.Time.now()
        self.timeTriggerElapsed = rospy.Time(0)
            
        rv = 'success'
        while not rospy.is_shutdown():
            self.step_time_elapsed()

            if (self.commandExperiment=='exit_now'):
                rv = 'aborted'
                break

            # Check for trigger timeout.
            if (timeout <= self.timeTriggerElapsed.to_sec()):
                rv = 'success'
                break

            # Check for experiment timeout.
            if (self.experimentparams.experiment.timeout != -1):
                if (self.experimentparams.experiment.timeout <= g_timeExperimentElapsed):
                    rv = 'timeout'
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
        
        userdata.experimentparamsOut = self.experimentparams
        return rv
# End class TriggerOnTime()



#######################################################################################################
#######################################################################################################
class ExperimentLib():
    def __init__(self, experimentparamsChoices=None, startexperiment_callback=None, starttrial_callback=None, endtrial_callback=None):
        
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
        self.smachTop.userdata.experimentparamsChoices = experimentparamsChoices
        #self.smachTop.userdata.experimentparams = ExperimentParams() #experimentparams # Should get set in START_TRIAL.


        # Create the 'RESET' concurrency state.
        ################################################################################### RESET
        smachReset = smach.Concurrence(outcomes = ['success','aborted'],
                                         default_outcome = 'aborted',
                                         child_termination_cb = self.AnyResetTerm_callback,
                                         outcome_cb = self.AllResetTerm_callback,
                                         input_keys = ['experimentparamsChoicesIn'])
        with smachReset:
            for (name,classHardware) in self.actions_dict.iteritems():
                smach.Concurrence.add(name, 
                                      classHardware.Reset (tfrx=self.tfrx),
                                      remapping={'experimentparamsChoicesIn':'experimentparamsChoicesIn'})


        ################################################################################### Pre Qualifier:  wait -> trigger -> wait.
        self.smachPreQualifier = smach.StateMachine(outcomes = ['success','preempt','aborted'],
                                                    input_keys = ['experimentparamsIn'],
                                                    output_keys = ['experimentparamsOut'])
        #self.smachPreQualifier.userdata.experimentparams = self.smachTop.userdata.experimentparams #experimentparams # Should get set in START_TRIAL.
        with self.smachPreQualifier:
            smach.StateMachine.add('PREWAIT1', 
                                   TriggerOnTime(mode='pre1', tfrx=self.tfrx),
                                   transitions={'success':'PRETRIGGER',   # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparamsIn',
                                              'experimentparamsOut':'experimentparamsOut'})


            smach.StateMachine.add('PRETRIGGER', 
                                   TriggerOnStates(mode='pre', tfrx=self.tfrx),
                                   transitions={'success':'PREWAIT2',        # Trigger service signal goes True.
                                                'disabled':'PREWAIT2',       # Trigger service signal goes True.
                                                'timeout':'PREWAIT2',        # Trigger service signal goes True.
                                                'preempt':'preempt',
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparamsIn',
                                              'experimentparamsOut':'experimentparamsOut'})


            smach.StateMachine.add('PREWAIT2', 
                                   TriggerOnTime(mode='pre2', tfrx=self.tfrx),
                                   transitions={'success':'success',   # Trigger service signal goes True.
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparamsIn',
                                              'experimentparamsOut':'experimentparamsOut'})
        
        
        # Create the 'Pre' concurrency state.  This is where the pre qualifier runs alongside the pre actions.
        ################################################################################### Pre qualifiers & trials.
        smachPre = smach.Concurrence(outcomes = ['success','disabled','timeout','aborted'],
                                       default_outcome = 'aborted',
                                       child_termination_cb = self.AnyPreTerm_callback,
                                       outcome_cb = self.AllPreTerm_callback,
                                       input_keys = ['experimentparamsIn'],
                                       output_keys = ['experimentparamsOut'])
        with smachPre:
            smach.Concurrence.add('PREQUALIFIER', 
                                  self.smachPreQualifier,
                                  remapping={'experimentparamsIn':'experimentparamsIn',
                                             'experimentparamsOut':'experimentparamsOut'})
            for (name,classHardware) in self.actions_dict.iteritems():   # Add a state for each action class in the dict.
                smach.Concurrence.add(name, 
                                      classHardware.Action (mode='pre', tfrx=self.tfrx),
                                      remapping={'experimentparamsIn':'experimentparamsIn',
                                                 'experimentparamsOut':'experimentparamsOut'})

        
        # Create the main 'Trial' concurrency state.
        ################################################################################### TRIAL
        smachTrial = smach.Concurrence(outcomes = ['success','disabled','aborted'],
                                         default_outcome = 'aborted',
                                         child_termination_cb = self.AnyTrialTerm_callback,
                                         outcome_cb = self.AllTrialTerm_callback,
                                         input_keys = ['experimentparamsIn'],
                                         output_keys = ['experimentparamsOut'])
        with smachTrial:
            for (name,classHardware) in self.actions_dict.iteritems():   # Add a state for each action class in the dict.
                smach.Concurrence.add(name, 
                                      classHardware.Action (mode='trial', tfrx=self.tfrx),
                                      remapping={'experimentparamsIn':'experimentparamsIn',
                                                 'experimentparamsOut':'experimentparamsOut'})
            smach.Concurrence.add('POSTTRIGGER', 
                                  TriggerOnStates(mode='post', tfrx=self.tfrx),
                                  remapping={'experimentparamsIn':'experimentparamsIn',
                                             'experimentparamsOut':'experimentparamsOut'})


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
                                   remapping={'experimentparamsChoicesIn':'experimentparamsChoices',
                                              'experimentparamsChoicesOut':'experimentparamsChoices'})


            ################################################################################### STARTEXPERIMENTCALLBACK -> RESETHARDWARE
            if startexperiment_callback is not None:
                smach.StateMachine.add('STARTEXPERIMENTCALLBACK', 
                                       smach.CBState(startexperiment_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsChoicesIn'],
                                                     output_keys = ['experimentparamsChoicesOut']),
                                       transitions={'success':'RESETHARDWARE',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsChoicesIn':'experimentparamsChoices',
                                                  'experimentparamsChoicesOut':'experimentparamsChoices'})



            ################################################################################### RESETHARDWARE -> (STARTTRIALCALLBACK or STARTTRIAL)
            if starttrial_callback is not None:
                stateAfterResetHardware = 'STARTTRIALCALLBACK'
            else:
                stateAfterResetHardware = 'STARTTRIAL'

            smach.StateMachine.add('RESETHARDWARE',                         
                                   smachReset,
                                   transitions={'success':stateAfterResetHardware,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsChoicesIn':'experimentparamsChoices'})


            ################################################################################### STARTTRIALCALLBACK -> STARTTRIAL
            if starttrial_callback is not None:
                smach.StateMachine.add('STARTTRIALCALLBACK', 
                                       smach.CBState(starttrial_callback, 
                                                     outcomes = ['success','aborted'],
                                                     input_keys = ['experimentparamsChoicesIn'],
                                                     output_keys = ['experimentparamsChoicesOut']),
                                       transitions={'success':'STARTTRIAL',
                                                    'aborted':'aborted'},
                                       remapping={'experimentparamsChoicesIn':'experimentparamsChoices',
                                                  'experimentparamsChoicesOut':'experimentparamsChoices'})


            ################################################################################### STARTTRIAL -> PRE
            smach.StateMachine.add('STARTTRIAL',                         
                                   StartTrial(),
                                   transitions={'continue':'PRE',         # Trigger service signal goes False.
                                                'exit':'ENDEXPERIMENT',   # Trigger service signal goes False.
                                                'aborted':'aborted'},     # Trigger service signal goes False.
                                   remapping={'experimentparamsChoicesIn':'experimentparamsChoices',
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
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


            ################################################################################### POSTWAIT -> (ENDTRIALCALLBACK or RESETHARDWARE)
            if endtrial_callback is not None:
                stateAfterPostWait = 'ENDTRIALCALLBACK'
            else:
                stateAfterPostWait = 'ENDTRIAL'

            smach.StateMachine.add('POSTWAIT', 
                                   TriggerOnTime(mode='post', tfrx=self.tfrx),
                                   transitions={'success':stateAfterPostWait,
                                                'aborted':'aborted'},
                                   remapping={'experimentparamsIn':'experimentparams',
                                              'experimentparamsOut':'experimentparams'})


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
                                   remapping={'experimentparamsChoicesIn':'experimentparamsChoices',
                                              'experimentparamsChoicesOut':'experimentparamsChoices'})


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
    
