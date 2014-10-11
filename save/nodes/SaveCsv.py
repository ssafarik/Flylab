#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save')
import rospy

import tf
import sys
import time, os, subprocess
import threading
import numpy as np
from std_msgs.msg import String
from flycore.msg import MsgFrameState
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsChoices
from tracking.msg import ArenaState




###############################################################################
# SaveCsv() is a ROS node.  It saves Arenastate messages into .csv files.
#
class SaveCsv:
    def __init__(self):
        self.initialized = False

        # Create new directory each day
        self.dirBase = os.path.expanduser("~/FlylabData")
        self.dirCsv = self.dirBase + "/" + time.strftime("%Y_%m_%d")

        # Make sure dir exists.
        try:
            os.makedirs(self.dirCsv)
        except OSError, e:
            pass


        self.bTriggered = False
        self.bSaveOnlyWhileTriggered = True # False: Save everything from one trial_start to the trial_end.  True:  Save everything from trigger=on to trigger=off.


        
        ############# Arenastate/CSV stuff
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.sub_arenastate = rospy.Subscriber("ArenaState", ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)
        self.subCommand = rospy.Subscriber('experiment/command', String, self.CommandExperiment_callback)

        self.lock = threading.Lock()
        
        self.filenameCsv = None
        self.fid = None
        self.bSaveCsv = False
        self.bSavingCsv = False
        self.commandExperiment = 'continue'

        self.format_align = ">"
        self.format_sign = " "
        self.format_width = "7"
        self.format_precision = "3"
        self.format_type = "f"

        # 2.84 added robotspec.isPresent
        # 2.85 changed structure of robot.move.relative
        #################################################################################
        self.versionFile = '2.85'    # Increment this when the file format changes.
        #################################################################################

        
        self.headerVersionFileTxt = 'versionFile\n'
        self.templateVersionFile =  '{versionFile:s}\n'
        
        self.headerExperimentTxt =  'date_time, '\
                                    'description, '\
                                    'maxTrials, '\
                                    'trial\n'
        self.templateExperiment =   '{date_time:s}, '\
                                    '{description:s}, '\
                                    '{maxTrials:s}, '\
                                    '{trial:s}\n'
                                  
        self.headerRobotSpecTxt =   'nRobots, '\
                                    'widthRobot, '\
                                    'heightRobot, '\
                                    'isPresent, '\
                                    'descriptionRobot\n'
        self.templateRobotSpec =    '{nRobots:s}, '\
                                    '{widthRobot:s}, '\
                                    '{heightRobot:s}, '\
                                    '{isPresent:s}, '\
                                    '{descriptionRobot:s}\n'
                                  
        self.headerFlySpecTxt =     'nFlies, '\
                                    'descriptionFlies\n'
        self.templateFlySpec =      '{nFlies:s}, '\
                                    '{descriptionFlies:s}\n'

        self.headerTrackingTxtA =   'trackingExclusionzoneEnabled'
        self.headerTrackingTxtB =   ', trackingExclusionzoneX, '\
                                    'trackingExclusionzoneY, '\
                                    'trackingExclusionzoneRadius'
        self.templateTrackingA =    '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB =    ', {trackingExclusionzoneX:s}, '\
                                    '{trackingExclusionzoneY:s}, '\
                                    '{trackingExclusionzoneRadius:s}'

        self.headerPreRobotTxt =    'preRobotEnabled, '\
                                    'preRobotMovePatternFramePosition, '\
                                    'preRobotMovePatternFrameAngle, '\
                                    'preRobotMovePatternShape, '\
                                    'preRobotMovePatternHzPattern, '\
                                    'preRobotMovePatternHzPoint, '\
                                    'preRobotMovePatternCount, '\
                                    'preRobotMovePatternSizeX, '\
                                    'preRobotMovePatternSizeY, '\
                                    'preRobotMovePatternParam, '\
                                    'preRobotMovePatternDirection, '\
                                    'preRobotMoveRelTracking, '\
                                    'preRobotMoveRelFrame, '\
                                    'preRobotMoveRelDistance, '\
                                    'preRobotMoveRelAngleOffset, '\
                                    'preRobotMoveRelAngleVelocity, '\
                                    'preRobotMoveRelAngleOscMag, '\
                                    'preRobotMoveRelAngleOscFreq, '\
                                    'preRobotMoveRelSpeedMax, '\
                                    'preRobotMoveRelTolerance, '\
                                    'preRobotMoveRelTypeAngleOffset, '\
                                    'preRobotMoveRelTypeAngleVelocity, '\
                                    'preRobotMoveRelTypeAngleOscMag, '\
                                    'preRobotMoveRelTypeAngleOscFreq, '\
                                    'preRobotMoveRelTypeSpeedMax\n'

        self.templatePreRobot     = '{preRobotEnabled:s}, '\
                                    '{preRobotMovePatternFramePosition:s}, '\
                                    '{preRobotMovePatternFrameAngle:s}, '\
                                    '{preRobotMovePatternShape:s}, '\
                                    '{preRobotMovePatternHzPattern:s}, '\
                                    '{preRobotMovePatternHzPoint:s}, '\
                                    '{preRobotMovePatternCount:s}, '\
                                    '{preRobotMovePatternSizeX:s}, '\
                                    '{preRobotMovePatternSizeY:s}, '\
                                    '{preRobotMovePatternParam:s}, '\
                                    '{preRobotMovePatternDirection:s}, '\
                                    '{preRobotMoveRelTracking:s}, '\
                                    '{preRobotMoveRelFrame:s}, '\
                                    '{preRobotMoveRelDistance:s}, '\
                                    '{preRobotMoveRelAngleOffset:s}, '\
                                    '{preRobotMoveRelAngleVelocity:s}, '\
                                    '{preRobotMoveRelAngleOscMag:s}, '\
                                    '{preRobotMoveRelAngleOscFreq:s}, '\
                                    '{preRobotMoveRelSpeedMax:s}, '\
                                    '{preRobotMoveRelTolerance:s}, '\
                                    '{preRobotMoveRelTypeAngleOffset:s}, '\
                                    '{preRobotMoveRelTypeAngleVelocity:s}, '\
                                    '{preRobotMoveRelTypeAngleOscMag:s}, '\
                                    '{preRobotMoveRelTypeAngleOscFreq:s}, '\
                                    '{preRobotMoveRelTypeSpeedMax:s}\n'
                                    
        self.headerPreLaserTxt    = 'preLaserEnabled, '\
                                    'preLaserPatternFramePosition, '\
                                    'preLaserPatternFrameAngle, '\
                                    'preLaserPatternShape, '\
                                    'preLaserPatternHzPattern, '\
                                    'preLaserPatternHzPoint, '\
                                    'preLaserPatternCount, '\
                                    'preLaserPatternSizeX, '\
                                    'preLaserPatternSizeY, '\
                                    'preLaserPatternParam, '\
                                    'preLaserPatternDirection, '\
                                    'preLaserStatefilterLo, '\
                                    'preLaserStatefilterHi, '\
                                    'preLaserStatefilterCriteria\n'
        self.templatePreLaser =     '{preLaserEnabled:s}, '\
                                    '{preLaserPatternFramePosition:s}, '\
                                    '{preLaserPatternFrameAngle:s}, '\
                                    '{preLaserPatternShape:s}, '\
                                    '{preLaserPatternHzPattern:s}, '\
                                    '{preLaserPatternHzPoint:s}, '\
                                    '{preLaserPatternCount:s}, '\
                                    '{preLaserPatternSizeX:s}, '\
                                    '{preLaserPatternSizeY:s}, '\
                                    '{preLaserPatternParam:s}, '\
                                    '{preLaserPatternDirection:s}, '\
                                    '\"{preLaserStatefilterLo:s}\", '\
                                    '\"{preLaserStatefilterHi:s}\", '\
                                    '{preLaserStatefilterCriteria:s}\n'
        
        self.headerPreLEDPanelsTxt ='preLEDPanelsEnabled, '\
                                    'preLEDPanelsCommand, '\
                                    'preLEDPanelsIdPattern, '\
                                    'preLEDPanelsFrameid, '\
                                    'preLEDPanelsStatefilterLo, '\
                                    'preLEDPanelsStatefilterHi, '\
                                    'preLEDPanelsStatefilterCriteria\n'
        self.templatePreLEDPanels = '{preLEDPanelsEnabled:s}, '\
                                    '{preLEDPanelsCommand:s}, '\
                                    '{preLEDPanelsIdPattern:s}, '\
                                    '{preLEDPanelsFrameid:s}, '\
                                    '{preLEDPanelsStatefilterLo:s}, '\
                                    '{preLEDPanelsStatefilterHi:s}, '\
                                    '{preLEDPanelsStatefilterCriteria:s}\n'

        
        self.headerPreWait1Txt =    'preWait1\n'
        self.templatePreWait1 =     '{preWait1:s}\n'
        
        self.headerPreTriggerTxt =  'preTriggerEnabled, '\
                                    'preTriggerFrameidParent, '\
                                    'preTriggerFrameidChild, '\
                                    'preTriggerSpeedAbsParentMin, '\
                                    'preTriggerSpeedAbsParentMax, '\
                                    'preTriggerSpeedAbsChildMin, '\
                                    'preTriggerSpeedAbsChildMax, '\
                                    'preTriggerSpeedRelMin, '\
                                    'preTriggerSpeedRelMax, '\
                                    'preTriggerDistanceMin, '\
                                    'preTriggerDistanceMax, '\
                                    'preTriggerAngleMin, '\
                                    'preTriggerAngleMax, '\
                                    'preTriggerAngleTest, '\
                                    'preTriggerAngleTestBilateral, '\
                                    'preTriggerTimeHold, '\
                                    'preTriggerTimeout\n'
        self.templatePreTrigger =   '{preTriggerEnabled:s}, '\
                                    '{preTriggerFrameidParent:s}, '\
                                    '{preTriggerFrameidChild:s}, '\
                                    '{preTriggerSpeedAbsParentMin:s}, '\
                                    '{preTriggerSpeedAbsParentMax:s}, '\
                                    '{preTriggerSpeedAbsChildMin:s}, '\
                                    '{preTriggerSpeedAbsChildMax:s}, '\
                                    '{preTriggerSpeedRelMin:s}, '\
                                    '{preTriggerSpeedRelMax:s}, '\
                                    '{preTriggerDistanceMin:s}, '\
                                    '{preTriggerDistanceMax:s}, '\
                                    '{preTriggerAngleMin:s}, '\
                                    '{preTriggerAngleMax:s}, '\
                                    '{preTriggerAngleTest:s}, '\
                                    '{preTriggerAngleTestBilateral:s}, '\
                                    '{preTriggerTimeHold:s}, '\
                                    '{preTriggerTimeout:s}\n'
                                    
        self.headerPreWait2Txt =    'preWait2\n'
        self.templatePreWait2 =     '{preWait2:s}\n'
        
        
        self.headerTrialRobotTxt =  'trialRobotEnabled, '\
                                    'trialRobotMovePatternFramePosition, '\
                                    'trialRobotMovePatternFrameAngle, '\
                                    'trialRobotMovePatternShape, '\
                                    'trialRobotMovePatternHzPattern, '\
                                    'trialRobotMovePatternHzPoint, '\
                                    'trialRobotMovePatternCount, '\
                                    'trialRobotMovePatternSizeX, '\
                                    'trialRobotMovePatternSizeY, '\
                                    'trialRobotMovePatternParam, '\
                                    'trialRobotMovePatternDirection, '\
                                    'trialRobotMoveRelTracking, '\
                                    'trialRobotMoveRelFrame, '\
                                    'trialRobotMoveRelDistance, '\
                                    'trialRobotMoveRelAngleOffset, '\
                                    'trialRobotMoveRelAngleVelocity, '\
                                    'trialRobotMoveRelAngleOscMag, '\
                                    'trialRobotMoveRelAngleOscFreq, '\
                                    'trialRobotMoveRelSpeedMax, '\
                                    'trialRobotMoveRelTolerance, '\
                                    'trialRobotMoveRelTypeAngleOffset, '\
                                    'trialRobotMoveRelTypeAngleVelocity, '\
                                    'trialRobotMoveRelTypeAngleOscMag, '\
                                    'trialRobotMoveRelTypeAngleOscFreq, '\
                                    'trialRobotMoveRelTypeSpeedMax\n'
        self.templateTrialRobot =   '{trialRobotEnabled:s}, '\
                                    '{trialRobotMovePatternFramePosition:s}, '\
                                    '{trialRobotMovePatternFrameAngle:s}, '\
                                    '{trialRobotMovePatternShape:s}, '\
                                    '{trialRobotMovePatternHzPattern:s}, '\
                                    '{trialRobotMovePatternHzPoint:s}, '\
                                    '{trialRobotMovePatternCount:s}, '\
                                    '{trialRobotMovePatternSizeX:s}, '\
                                    '{trialRobotMovePatternSizeY:s}, '\
                                    '{trialRobotMovePatternParam:s}, '\
                                    '{trialRobotMovePatternDirection:s}, '\
                                    '{trialRobotMoveRelTracking:s}, '\
                                    '{trialRobotMoveRelFrame:s}, '\
                                    '{trialRobotMoveRelDistance:s}, '\
                                    '{trialRobotMoveRelAngleOffset:s}, '\
                                    '{trialRobotMoveRelAngleVelocity:s}, '\
                                    '{trialRobotMoveRelAngleOscMag:s}, '\
                                    '{trialRobotMoveRelAngleOscFreq:s}, '\
                                    '{trialRobotMoveRelSpeedMax:s}, '\
                                    '{trialRobotMoveRelTolerance:s}, '\
                                    '{trialRobotMoveRelTypeAngleOffset:s}, '\
                                    '{trialRobotMoveRelTypeAngleVelocity:s}, '\
                                    '{trialRobotMoveRelTypeAngleOscMag:s}, '\
                                    '{trialRobotMoveRelTypeAngleOscFreq:s}, '\
                                    '{trialRobotMoveRelTypeSpeedMax:s}\n'
                                    
        self.headerTrialLaserTxt =  'trialLaserEnabled, '\
                                    'trialLaserPatternFramePosition, '\
                                    'trialLaserPatternFrameAngle, '\
                                    'trialLaserPatternShape, '\
                                    'trialLaserPatternHzPattern, '\
                                    'trialLaserPatternHzPoint, '\
                                    'trialLaserPatternCount, '\
                                    'trialLaserPatternSizeX, '\
                                    'trialLaserPatternSizeY, '\
                                    'trialLaserPatternParam, '\
                                    'trialLaserPatternDirection, '\
                                    'trialLaserStatefilterLo, '\
                                    'trialLaserStatefilterHi, '\
                                    'trialLaserStatefilterCriteria\n'
        self.templateTrialLaser =   '{trialLaserEnabled:s}, '\
                                    '{trialLaserPatternFramePosition:s}, '\
                                    '{trialLaserPatternFrameAngle:s}, '\
                                    '{trialLaserPatternShape:s}, '\
                                    '{trialLaserPatternHzPattern:s}, '\
                                    '{trialLaserPatternHzPoint:s}, '\
                                    '{trialLaserPatternCount:s}, '\
                                    '{trialLaserPatternSizeX:s}, '\
                                    '{trialLaserPatternSizeY:s}, '\
                                    '{trialLaserPatternParam:s}, '\
                                    '{trialLaserPatternDirection:s}, '\
                                    '\"{trialLaserStatefilterLo:s}\", '\
                                    '\"{trialLaserStatefilterHi:s}\", '\
                                    '{trialLaserStatefilterCriteria:s}\n'
        
        self.headerTrialLEDPanelsTxt = 'trialLEDPanelsEnabled, '\
                                    'trialLEDPanelsCommand, '\
                                    'trialLEDPanelsIdPattern, '\
                                    'trialLEDPanelsFrameid, '\
                                    'trialLEDPanelsStatefilterLo, '\
                                    'trialLEDPanelsStatefilterHi, '\
                                    'trialLEDPanelsStatefilterCriteria\n'
        self.templateTrialLEDPanels = '{trialLEDPanelsEnabled:s}, '\
                                    '{trialLEDPanelsCommand:s}, '\
                                    '{trialLEDPanelsIdPattern:s}, '\
                                    '{trialLEDPanelsFrameid:s}, '\
                                    '{trialLEDPanelsStatefilterLo:s}, '\
                                    '{trialLEDPanelsStatefilterHi:s}, '\
                                    '{trialLEDPanelsStatefilterCriteria:s}\n'
        
        self.headerPostTriggerTxt = 'postTriggerEnabled, '\
                                    'postTriggerFrameidParent, '\
                                    'postTriggerFrameidChild, '\
                                    'postTriggerSpeedAbsParentMin, '\
                                    'postTriggerSpeedAbsParentMax, '\
                                    'postTriggerSpeedAbsChildMin, '\
                                    'postTriggerSpeedAbsChildMax, '\
                                    'postTriggerSpeedRelMin, '\
                                    'postTriggerSpeedRelMax, '\
                                    'postTriggerDistanceMin, '\
                                    'postTriggerDistanceMax, '\
                                    'postTriggerAngleMin, '\
                                    'postTriggerAngleMax, '\
                                    'postTriggerAngleTest, '\
                                    'postTriggerAngleTestBilateral, '\
                                    'postTriggerTimeHold, '\
                                    'postTriggerTimeout\n'
        self.templatePostTrigger =  '{postTriggerEnabled:s}, '\
                                    '{postTriggerFrameidParent:s}, '\
                                    '{postTriggerFrameidChild:s}, '\
                                    '{postTriggerSpeedAbsParentMin:s}, '\
                                    '{postTriggerSpeedAbsParentMax:s}, '\
                                    '{postTriggerSpeedAbsChildMin:s}, '\
                                    '{postTriggerSpeedAbsChildMax:s}, '\
                                    '{postTriggerSpeedRelMin:s}, '\
                                    '{postTriggerSpeedRelMax:s}, '\
                                    '{postTriggerDistanceMin:s}, '\
                                    '{postTriggerDistanceMax:s}, '\
                                    '{postTriggerAngleMin:s}, '\
                                    '{postTriggerAngleMax:s}, '\
                                    '{postTriggerAngleTest:s}, '\
                                    '{postTriggerAngleTestBilateral:s}, '\
                                    '{postTriggerTimeHold:s}, '\
                                    '{postTriggerTimeout:s}\n'
                                  
        self.headerPostWaitTxt =    'postWait\n'
        self.templatePostWait =     '{postWait:s}\n'

        # Construct a variable-length state line, depending on the number of flies.
        self.stateLeftTxt       = 'time, triggered'
        self.stateRobotTxt      = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, aRobotWingLeft, aRobotWingRight'
        self.stateFlyTxt        = ', xFly, yFly, aFly, vxFly, vyFly, vaFly, aFlyWingLeft, aFlyWingRight'
        self.templateStateLeft  = '{time:0.4f}, {triggered:s}'
        self.templateStateRobot = ', {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {aRobotWingLeft:{align}{sign}{width}.{precision}{type}}, {aRobotWingRight:{align}{sign}{width}.{precision}{type}}'
        self.templateStateFly   = ', {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}, {aFlyWingLeft:{align}{sign}{width}.{precision}{type}}, {aFlyWingRight:{align}{sign}{width}.{precision}{type}}'


        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Offer some services.
        self.services = {}
        self.services['savearenastate/init']            = rospy.Service('savearenastate/init',            ExperimentParamsChoices, self.Init_callback)
        self.services['savearenastate/trial_start']     = rospy.Service('savearenastate/trial_start',     ExperimentParams,        self.TrialStart_callback)
        self.services['savearenastate/trial_end']       = rospy.Service('savearenastate/trial_end',       ExperimentParams,        self.TrialEnd_callback)
        self.services['savearenastate/trigger']         = rospy.Service('savearenastate/trigger',         Trigger,                 self.Trigger_callback)
        self.services['savearenastate/wait_until_done'] = rospy.Service('savearenastate/wait_until_done', ExperimentParamsChoices, self.WaitUntilDone_callback)

        self.initialized = True


    def OnShutdown_callback(self):
        with self.lock:
            if (self.fid is not None) and (not self.fid.closed):
                self.fid.close()
                rospy.logwarn('Closed csv file.')

        

    # Service callback to perform initialization that requires experimentparams, e.g. subscribing to image topics.
    def Init_callback(self, experimentparamsChoices):
        self.paramsSave = experimentparamsChoices.save

        return True


    def CommandExperiment_callback(self, msgString):
        #rospy.logwarn('TriggerOnTime received: %s' % msgString.data)
        self.commandExperiment = msgString.data
            
        

    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
    #    Closes the .csv file if no longer saving.
    #
    def Trigger_callback(self, reqTrigger):
        while (not self.initialized):
            rospy.sleep(0.5)

        bRisingEdge = False
        bFallingEdge = False
        if self.bTriggered != reqTrigger.triggered:
            if reqTrigger.triggered: # Rising edge.
                bRisingEdge = True
            else:
                bFallingEdge = True
                
        self.bTriggered = reqTrigger.triggered


        if (self.bSaveOnlyWhileTriggered) and (self.bSaveCsv):
            if (reqTrigger.triggered):
                self.bSavingCsv = True
            else:
                self.bSavingCsv = False
        

        # At the end of a run, close the file if we're no longer saving.
        if (self.bSaveOnlyWhileTriggered):
            if (self.bSaveCsv):
                with self.lock:
                    if (bFallingEdge) and (self.fid is not None) and (not self.fid.closed):
                        self.fid.close()
                        rospy.logwarn('Closed csv file.')

        return self.bTriggered
        

    # TrialStart_callback()
    # Open a new .csv file when we start a new trial.
    # Possibly set flag self.bSavingCsv to save arenastate.
    # Returns with a file open.
    # 
    def TrialStart_callback(self, experimentparamsReq):
        self.bSaveCsv = experimentparamsReq.save.csv
        
        while (not self.initialized):
            rospy.sleep(0.5)

        self.bSaveOnlyWhileTriggered = experimentparamsReq.save.onlyWhileTriggered

        if (self.bSaveCsv):
            # Get the directory:  dirBag = 'FlylabData/YYYY_MM_DD'
            self.dirCsv = self.dirBase + '/' + time.strftime('%Y_%m_%d')

            # Make sure path exists.
            try:
                os.makedirs(self.dirCsv)
            except OSError:
                pass
            
            # Determine if we should be saving.
            if (self.bSaveOnlyWhileTriggered):
                self.bSavingCsv = False
            else:
                self.bSavingCsv = True
            
            self.OpenCsvAndWriteHeader(experimentparamsReq)


                
        return True
                
                
    # TrialEnd_callback()
    # Close old .csv file if there was one.
    # 
    def TrialEnd_callback(self, experimentparamsReq):
        while (not self.initialized):
            rospy.sleep(0.5)

        if (self.bSaveCsv):
            # Close old .csv file if there was one.
            with self.lock:
                if (self.fid is not None) and (not self.fid.closed):
                    self.fid.close()
                    rospy.logwarn('Closed csv file.')
                    

        return True
                
                
    def OpenCsvAndWriteHeader(self, experimentparamsReq):                
        self.filenameCsv = "%s%s.csv" % (experimentparamsReq.save.filenamebase, experimentparamsReq.save.timestamp)
        
        headerVersionFile = self.templateVersionFile.format(
                                                versionFile                = self.versionFile
                                                )
        headerExperiment = self.templateExperiment.format(
                                                date_time                  = str(rospy.Time.now().to_sec()),
                                                description                = str(experimentparamsReq.experiment.description),
                                                maxTrials                  = str(experimentparamsReq.experiment.maxTrials),
                                                trial                      = str(experimentparamsReq.experiment.trial),
                                                )
        headerRobotSpec = self.templateRobotSpec.format(
                                                nRobots                    = str(experimentparamsReq.robotspec.nRobots),
                                                widthRobot                 = str(experimentparamsReq.robotspec.width),
                                                heightRobot                = str(experimentparamsReq.robotspec.height),
                                                isPresent                  = str(experimentparamsReq.robotspec.isPresent),
                                                descriptionRobot           = str(experimentparamsReq.robotspec.description),
                                                )
        headerFlySpec = self.templateFlySpec.format(
                                                nFlies                     = str(experimentparamsReq.flyspec.nFlies),
                                                descriptionFlies           = str(experimentparamsReq.flyspec.description),
                                                )

        self.headerTrackingTxt = self.headerTrackingTxtA
        headerTracking = self.templateTrackingA.format(trackingExclusionzoneEnabled = str(experimentparamsReq.tracking.exclusionzones.enabled))
        for i in range(len(experimentparamsReq.tracking.exclusionzones.point_list)):
            self.headerTrackingTxt += self.headerTrackingTxtB
            headerTracking += self.templateTrackingB.format(
                                                trackingExclusionzoneX       = str(experimentparamsReq.tracking.exclusionzones.point_list[i].x),
                                                trackingExclusionzoneY       = str(experimentparamsReq.tracking.exclusionzones.point_list[i].y),
                                                trackingExclusionzoneRadius  = str(experimentparamsReq.tracking.exclusionzones.radius_list[i]),
                                                )
        self.headerTrackingTxt += '\n'
        headerTracking += '\n'

        #######################################################################
        headerPreRobot = self.templatePreRobot.format(
                                                preRobotEnabled                  = str(experimentparamsReq.pre.robot.enabled),
                                                preRobotMovePatternFramePosition = str(experimentparamsReq.pre.robot.move.pattern.frameidPosition),
                                                preRobotMovePatternFrameAngle    = str(experimentparamsReq.pre.robot.move.pattern.frameidAngle),
                                                preRobotMovePatternShape         = str(experimentparamsReq.pre.robot.move.pattern.shape),
                                                preRobotMovePatternHzPattern     = str(experimentparamsReq.pre.robot.move.pattern.hzPattern),
                                                preRobotMovePatternHzPoint       = str(experimentparamsReq.pre.robot.move.pattern.hzPoint),
                                                preRobotMovePatternCount         = str(experimentparamsReq.pre.robot.move.pattern.count),
                                                preRobotMovePatternSizeX         = str(experimentparamsReq.pre.robot.move.pattern.size.x),
                                                preRobotMovePatternSizeY         = str(experimentparamsReq.pre.robot.move.pattern.size.y),
                                                preRobotMovePatternParam         = str(experimentparamsReq.pre.robot.move.pattern.param),
                                                preRobotMovePatternDirection     = str(experimentparamsReq.pre.robot.move.pattern.direction),
                                                preRobotMoveRelTracking          = str(experimentparamsReq.pre.robot.move.relative.tracking),
                                                preRobotMoveRelFrame             = str(experimentparamsReq.pre.robot.move.relative.frameidOrigin),
                                                preRobotMoveRelDistance          = str(experimentparamsReq.pre.robot.move.relative.distance),
                                                preRobotMoveRelAngleOffset       = str(experimentparamsReq.pre.robot.move.relative.angleOffset),
                                                preRobotMoveRelAngleVelocity     = str(experimentparamsReq.pre.robot.move.relative.angleVelocity),
                                                preRobotMoveRelAngleOscMag       = str(experimentparamsReq.pre.robot.move.relative.angleOscMag),
                                                preRobotMoveRelAngleOscFreq      = str(experimentparamsReq.pre.robot.move.relative.angleOscFreq),
                                                preRobotMoveRelSpeedMax          = str(experimentparamsReq.pre.robot.move.relative.speedMax),
                                                preRobotMoveRelTolerance         = str(experimentparamsReq.pre.robot.move.relative.tolerance),
                                                preRobotMoveRelTypeAngleOffset   = str(experimentparamsReq.pre.robot.move.relative.typeAngleOffset),
                                                preRobotMoveRelTypeAngleVelocity = str(experimentparamsReq.pre.robot.move.relative.typeAngleVelocity),
                                                preRobotMoveRelTypeAngleOscMag   = str(experimentparamsReq.pre.robot.move.relative.typeAngleOscMag),
                                                preRobotMoveRelTypeAngleOscFreq  = str(experimentparamsReq.pre.robot.move.relative.typeAngleOscFreq),
                                                preRobotMoveRelTypeSpeedMax      = str(experimentparamsReq.pre.robot.move.relative.typeSpeedMax),
                                                )

        #######################################################################
        if len(experimentparamsReq.pre.lasergalvos.pattern_list) > 0:
            patternFramePosition   = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].frameidPosition)
            patternFrameAngle      = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].frameidAngle)
            patternShape           = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].shape)
            patternHzPattern       = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].hzPattern)
            patternHzPoint         = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].hzPoint)
            patternCount           = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].count)
            patternSizeX           = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].size.x)
            patternSizeY           = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].size.y)
            patternParam           = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].param)
            patternDirection       = str(experimentparamsReq.pre.lasergalvos.pattern_list[0].direction)
        else:
            patternFramePosition   = ""
            patternFrameAngle      = ""
            patternShape           = ""
            patternHzPattern       = str(0.0)
            patternHzPoint         = str(0.0)
            patternCount           = str(0)
            patternSizeX           = str(0.0)
            patternSizeY           = str(0.0)
            patternParam           = str(0.0)
            patternDirection       = str(1)

        if len(experimentparamsReq.pre.lasergalvos.statefilterHi_list) > 0:
            statefilterHi          = str(experimentparamsReq.pre.lasergalvos.statefilterHi_list[0])
            statefilterLo          = str(experimentparamsReq.pre.lasergalvos.statefilterLo_list[0])
            statefilterCriteria    = str(experimentparamsReq.pre.lasergalvos.statefilterCriteria_list[0])
        else:
            statefilterHi          = ""
            statefilterLo          = ""
            statefilterCriteria    = ""

        headerPreLaser = self.templatePreLaser.format(
                                                preLaserEnabled               = str(experimentparamsReq.pre.lasergalvos.enabled),
                                                preLaserPatternFramePosition  = patternFramePosition,
                                                preLaserPatternFrameAngle     = patternFrameAngle,
                                                preLaserPatternShape          = patternShape,
                                                preLaserPatternHzPattern      = patternHzPattern,
                                                preLaserPatternHzPoint        = patternHzPoint,
                                                preLaserPatternCount          = patternCount,
                                                preLaserPatternSizeX          = patternSizeX,
                                                preLaserPatternSizeY          = patternSizeY,
                                                preLaserPatternParam          = patternParam,
                                                preLaserPatternDirection      = patternDirection,
                                                preLaserStatefilterHi         = statefilterHi,
                                                preLaserStatefilterLo         = statefilterLo,
                                                preLaserStatefilterCriteria   = statefilterCriteria,
                                                )
            
        #######################################################################
        headerPreLEDPanels = self.templatePreLEDPanels.format(
                                                preLEDPanelsEnabled             = str(experimentparamsReq.pre.ledpanels.enabled),
                                                preLEDPanelsCommand             = str(experimentparamsReq.pre.ledpanels.command),
                                                preLEDPanelsIdPattern           = str(experimentparamsReq.pre.ledpanels.idPattern),
                                                preLEDPanelsFrameid             = str(experimentparamsReq.pre.ledpanels.frame_id),
                                                preLEDPanelsStatefilterLo       = str(experimentparamsReq.pre.ledpanels.statefilterLo),
                                                preLEDPanelsStatefilterHi       = str(experimentparamsReq.pre.ledpanels.statefilterHi),
                                                preLEDPanelsStatefilterCriteria = str(experimentparamsReq.pre.ledpanels.statefilterCriteria),
                                                )
        #######################################################################
        headerPreWait1 = self.templatePreWait1.format(
                                                preWait1                  = str(experimentparamsReq.pre.wait1),
                                                )
        #######################################################################
        headerPreTrigger = self.templatePreTrigger.format(
                                                preTriggerEnabled            = str(experimentparamsReq.pre.trigger.enabled),
                                                preTriggerFrameidParent      = str(experimentparamsReq.pre.trigger.frameidParent),
                                                preTriggerFrameidChild       = str(experimentparamsReq.pre.trigger.frameidChild),
                                                preTriggerSpeedAbsParentMin  = str(experimentparamsReq.pre.trigger.speedAbsParentMin),
                                                preTriggerSpeedAbsParentMax  = str(experimentparamsReq.pre.trigger.speedAbsParentMax),
                                                preTriggerSpeedAbsChildMin   = str(experimentparamsReq.pre.trigger.speedAbsChildMin),
                                                preTriggerSpeedAbsChildMax   = str(experimentparamsReq.pre.trigger.speedAbsChildMax),
                                                preTriggerSpeedRelMin        = str(experimentparamsReq.pre.trigger.speedRelMin),
                                                preTriggerSpeedRelMax        = str(experimentparamsReq.pre.trigger.speedRelMax),
                                                preTriggerDistanceMin        = str(experimentparamsReq.pre.trigger.distanceMin),
                                                preTriggerDistanceMax        = str(experimentparamsReq.pre.trigger.distanceMax),
                                                preTriggerAngleMin           = str(experimentparamsReq.pre.trigger.angleMin),
                                                preTriggerAngleMax           = str(experimentparamsReq.pre.trigger.angleMax),
                                                preTriggerAngleTest          = str(experimentparamsReq.pre.trigger.angleTest),
                                                preTriggerAngleTestBilateral = str(experimentparamsReq.pre.trigger.angleTestBilateral),
                                                preTriggerTimeHold           = str(experimentparamsReq.pre.trigger.timeHold),
                                                preTriggerTimeout            = str(experimentparamsReq.pre.trigger.timeout),
                                                )
        #######################################################################
        headerPreWait2 = self.templatePreWait2.format(
                                                preWait2                       = str(experimentparamsReq.pre.wait2),
                                                )

        
        #######################################################################
        headerTrialRobot = self.templateTrialRobot.format(
                                                trialRobotEnabled                  = str(experimentparamsReq.trial.robot.enabled),
                                                trialRobotMovePatternFramePosition = str(experimentparamsReq.trial.robot.move.pattern.frameidPosition),
                                                trialRobotMovePatternFrameAngle    = str(experimentparamsReq.trial.robot.move.pattern.frameidAngle),
                                                trialRobotMovePatternShape         = str(experimentparamsReq.trial.robot.move.pattern.shape),
                                                trialRobotMovePatternHzPattern     = str(experimentparamsReq.trial.robot.move.pattern.hzPattern),
                                                trialRobotMovePatternHzPoint       = str(experimentparamsReq.trial.robot.move.pattern.hzPoint),
                                                trialRobotMovePatternCount         = str(experimentparamsReq.trial.robot.move.pattern.count),
                                                trialRobotMovePatternSizeX         = str(experimentparamsReq.trial.robot.move.pattern.size.x),
                                                trialRobotMovePatternSizeY         = str(experimentparamsReq.trial.robot.move.pattern.size.y),
                                                trialRobotMovePatternParam         = str(experimentparamsReq.trial.robot.move.pattern.param),
                                                trialRobotMovePatternDirection     = str(experimentparamsReq.trial.robot.move.pattern.direction),
                                                trialRobotMoveRelTracking          = str(experimentparamsReq.trial.robot.move.relative.tracking),
                                                trialRobotMoveRelFrame             = str(experimentparamsReq.trial.robot.move.relative.frameidOrigin),
                                                trialRobotMoveRelDistance          = str(experimentparamsReq.trial.robot.move.relative.distance),
                                                trialRobotMoveRelAngleOffset       = str(experimentparamsReq.trial.robot.move.relative.angleOffset),
                                                trialRobotMoveRelAngleVelocity     = str(experimentparamsReq.trial.robot.move.relative.angleVelocity),
                                                trialRobotMoveRelAngleOscMag       = str(experimentparamsReq.trial.robot.move.relative.angleOscMag),
                                                trialRobotMoveRelAngleOscFreq      = str(experimentparamsReq.trial.robot.move.relative.angleOscFreq),
                                                trialRobotMoveRelSpeedMax          = str(experimentparamsReq.trial.robot.move.relative.speedMax),
                                                trialRobotMoveRelTolerance         = str(experimentparamsReq.trial.robot.move.relative.tolerance),
                                                trialRobotMoveRelTypeAngleOffset   = str(experimentparamsReq.trial.robot.move.relative.typeAngleOffset),
                                                trialRobotMoveRelTypeAngleVelocity = str(experimentparamsReq.trial.robot.move.relative.typeAngleVelocity),
                                                trialRobotMoveRelTypeAngleOscMag   = str(experimentparamsReq.trial.robot.move.relative.typeAngleOscMag),
                                                trialRobotMoveRelTypeAngleOscFreq  = str(experimentparamsReq.trial.robot.move.relative.typeAngleOscFreq),
                                                trialRobotMoveRelTypeSpeedMax      = str(experimentparamsReq.trial.robot.move.relative.typeSpeedMax),
                                                )
        
        #######################################################################
        if len(experimentparamsReq.trial.lasergalvos.pattern_list) > 0:
            patternFramePosition   = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].frameidPosition)
            patternFrameAngle      = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].frameidAngle)
            patternShape           = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].shape)
            patternHzPattern       = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].hzPattern)
            patternHzPoint         = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].hzPoint)
            patternCount           = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].count)
            patternSizeX           = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].size.x)
            patternSizeY           = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].size.y)
            patternParam           = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].param)
            patternDirection       = str(experimentparamsReq.trial.lasergalvos.pattern_list[0].direction)
        else:
            patternFramePosition   = ""
            patternFrameAngle      = ""
            patternShape           = ""
            patternHzPattern       = str(0.0)
            patternHzPoint         = str(0.0)
            patternCount           = str(0)
            patternSizeX           = str(0.0)
            patternSizeY           = str(0.0)
            patternParam           = str(0.0)
            patternDirection       = str(0)

        if len(experimentparamsReq.trial.lasergalvos.statefilterHi_list) > 0:
            statefilterHi          = str(experimentparamsReq.trial.lasergalvos.statefilterHi_list[0])
            statefilterLo          = str(experimentparamsReq.trial.lasergalvos.statefilterLo_list[0])
            statefilterCriteria    = str(experimentparamsReq.trial.lasergalvos.statefilterCriteria_list[0])
        else:
            statefilterHi          = ""
            statefilterLo          = ""
            statefilterCriteria    = ""

        headerTrialLaser = self.templateTrialLaser.format(
                                                trialLaserEnabled               = str(experimentparamsReq.trial.lasergalvos.enabled),
                                                trialLaserPatternFramePosition  = patternFramePosition,
                                                trialLaserPatternFrameAngle     = patternFrameAngle,
                                                trialLaserPatternShape          = patternShape,
                                                trialLaserPatternHzPattern      = patternHzPattern,
                                                trialLaserPatternHzPoint        = patternHzPoint,
                                                trialLaserPatternCount          = patternCount,
                                                trialLaserPatternSizeX          = patternSizeX,
                                                trialLaserPatternSizeY          = patternSizeY,
                                                trialLaserPatternParam          = patternParam,
                                                trialLaserPatternDirection      = patternDirection,
                                                trialLaserStatefilterHi         = statefilterHi,
                                                trialLaserStatefilterLo         = statefilterLo,
                                                trialLaserStatefilterCriteria   = statefilterCriteria,
                                                )
            
        #######################################################################
        headerTrialLEDPanels = self.templateTrialLEDPanels.format(
                                                trialLEDPanelsEnabled             = str(experimentparamsReq.trial.ledpanels.enabled),
                                                trialLEDPanelsCommand             = str(experimentparamsReq.trial.ledpanels.command),
                                                trialLEDPanelsIdPattern           = str(experimentparamsReq.trial.ledpanels.idPattern),
                                                trialLEDPanelsFrameid             = str(experimentparamsReq.trial.ledpanels.frame_id),
                                                trialLEDPanelsStatefilterLo       = str(experimentparamsReq.trial.ledpanels.statefilterLo),
                                                trialLEDPanelsStatefilterHi       = str(experimentparamsReq.trial.ledpanels.statefilterHi),
                                                trialLEDPanelsStatefilterCriteria = str(experimentparamsReq.trial.ledpanels.statefilterCriteria),
                                                )
            
        #######################################################################
        headerPostTrigger = self.templatePostTrigger.format(
                                                postTriggerEnabled            = str(experimentparamsReq.post.trigger.enabled),
                                                postTriggerFrameidParent      = str(experimentparamsReq.post.trigger.frameidParent),
                                                postTriggerFrameidChild       = str(experimentparamsReq.post.trigger.frameidChild),
                                                postTriggerSpeedAbsParentMin  = str(experimentparamsReq.post.trigger.speedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(experimentparamsReq.post.trigger.speedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(experimentparamsReq.post.trigger.speedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(experimentparamsReq.post.trigger.speedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(experimentparamsReq.post.trigger.speedRelMin),
                                                postTriggerSpeedRelMax        = str(experimentparamsReq.post.trigger.speedRelMax),
                                                postTriggerDistanceMin        = str(experimentparamsReq.post.trigger.distanceMin),
                                                postTriggerDistanceMax        = str(experimentparamsReq.post.trigger.distanceMax),
                                                postTriggerAngleMin           = str(experimentparamsReq.post.trigger.angleMin),
                                                postTriggerAngleMax           = str(experimentparamsReq.post.trigger.angleMax),
                                                postTriggerAngleTest          = str(experimentparamsReq.post.trigger.angleTest),
                                                postTriggerAngleTestBilateral = str(experimentparamsReq.post.trigger.angleTestBilateral),
                                                postTriggerTimeHold           = str(experimentparamsReq.post.trigger.timeHold),
                                                postTriggerTimeout            = str(experimentparamsReq.post.trigger.timeout),
                                                )
        #######################################################################
        headerPostWait = self.templatePostWait.format(
                                                postWait                      = str(experimentparamsReq.post.wait),
                                                )
        #######################################################################


        with self.lock:
            fullpathCsv = self.dirCsv+'/'+self.filenameCsv
            self.fid = open(fullpathCsv, 'w')
            rospy.logwarn('Saving csv file:  %s' % fullpathCsv)

            self.fid.write(self.headerVersionFileTxt)
            self.fid.write(headerVersionFile)
            self.fid.write('\n')

            self.fid.write(self.headerExperimentTxt)
            self.fid.write(headerExperiment)
            self.fid.write('\n')

            self.fid.write(self.headerRobotSpecTxt)
            self.fid.write(headerRobotSpec)
            self.fid.write('\n')

            self.fid.write(self.headerFlySpecTxt)
            self.fid.write(headerFlySpec)
            self.fid.write('\n')

            self.fid.write(self.headerTrackingTxt)
            self.fid.write(headerTracking)
            self.fid.write('\n')

            self.fid.write(self.headerPreRobotTxt)
            self.fid.write(headerPreRobot)
            self.fid.write('\n')

            self.fid.write(self.headerPreLaserTxt)
            self.fid.write(headerPreLaser)
            self.fid.write('\n')

            self.fid.write(self.headerPreLEDPanelsTxt)
            self.fid.write(headerPreLEDPanels)
            self.fid.write('\n')
            
            self.fid.write(self.headerPreWait1Txt)
            self.fid.write(headerPreWait1)
            self.fid.write('\n')

            self.fid.write(self.headerPreTriggerTxt)
            self.fid.write(headerPreTrigger)
            self.fid.write('\n')

            self.fid.write(self.headerPreWait2Txt)
            self.fid.write(headerPreWait2)
            self.fid.write('\n')

            self.fid.write(self.headerTrialRobotTxt)
            self.fid.write(headerTrialRobot)
            self.fid.write('\n')

            self.fid.write(self.headerTrialLaserTxt)
            self.fid.write(headerTrialLaser)
            self.fid.write('\n')

            self.fid.write(self.headerTrialLEDPanelsTxt)
            self.fid.write(headerTrialLEDPanels)
            self.fid.write('\n')
            
            self.fid.write(self.headerPostTriggerTxt)
            self.fid.write(headerPostTrigger)
            self.fid.write('\n')

            self.fid.write(self.headerPostWaitTxt)
            self.fid.write(headerPostWait)
            self.fid.write('\n')
            
            
            self.stateTxt = self.stateLeftTxt + self.stateRobotTxt
            for i in range(experimentparamsReq.flyspec.nFlies):
                self.stateTxt += self.stateFlyTxt
            self.stateTxt += '\n'

            self.fid.write(self.stateTxt)

    
    

    def ArenaState_callback(self, arenastate):
        if (self.initialized) and (self.bSavingCsv) and (self.fid is not None) and (self.commandExperiment != 'pause_now'):

            # Get latest time.
            stamp = max(0.0, arenastate.robot.header.stamp.to_sec())
            for iFly in range(len(arenastate.flies)):
                stamp = max(stamp, arenastate.flies[iFly].header.stamp.to_sec())
            
            # Get the state of the robot.
            stateRobot = arenastate.robot
            q = stateRobot.pose.orientation
            rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            angleRobot = rpy[2] % (2.0 * np.pi)


            # Start putting the state row together.
            stateLeft = self.templateStateLeft.format(
                                                    align       = self.format_align,
                                                    sign        = self.format_sign,
                                                    width       = self.format_width,
                                                    precision   = self.format_precision,
                                                    type        = self.format_type,
                                                    time        = stamp, #rospy.Time.now().to_sec(), #stateRobot.header.stamp,
                                                    triggered   = str(int(self.bTriggered)),
                                                    )
            stateRobot = self.templateStateRobot.format(
                                                    align           = self.format_align,
                                                    sign            = self.format_sign,
                                                    width           = self.format_width,
                                                    precision       = self.format_precision,
                                                    type            = self.format_type,
                                                    xRobot          = stateRobot.pose.position.x,
                                                    yRobot          = stateRobot.pose.position.y,
                                                    aRobot          = angleRobot,
                                                    vxRobot         = stateRobot.velocity.linear.x,
                                                    vyRobot         = stateRobot.velocity.linear.y,
                                                    vaRobot         = stateRobot.velocity.angular.z,
                                                    aRobotWingLeft  = stateRobot.wings.left.angle,
                                                    aRobotWingRight = stateRobot.wings.right.angle,
                                                    )
            stateRow = stateLeft + stateRobot


            # Go through the flies.                
            for iFly in range(len(arenastate.flies)):
                # Get the state of each fly.
                if len(arenastate.flies)>iFly:
                    stateFly = arenastate.flies[iFly]
                else:
                    stateFly = MsgFrameState()
    
                q = stateFly.pose.orientation
                rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                angleFly = rpy[2] % (2.0 * np.pi)

                # Append the fly to the state row.            
                stateFly = self.templateStateFly.format(
                                                   align         = self.format_align,
                                                   sign          = self.format_sign,
                                                   width         = self.format_width,
                                                   precision     = self.format_precision,
                                                   type          = self.format_type,
                                                   xFly          = stateFly.pose.position.x,
                                                   yFly          = stateFly.pose.position.y,
                                                   aFly          = angleFly,
                                                   vxFly         = stateFly.velocity.linear.x,
                                                   vyFly         = stateFly.velocity.linear.y,
                                                   vaFly         = stateFly.velocity.angular.z,
                                                   aFlyWingLeft  = stateFly.wings.left.angle,
                                                   aFlyWingRight = stateFly.wings.right.angle,
                                                   )
                stateRow += stateFly
                
            stateRow += '\n'
            

            # Write the robot & fly state to the file.
            with self.lock:
                if (not self.fid.closed):
                    self.fid.write(stateRow)


    def WaitUntilDone_callback(self, experimentparamsChoices):
        return True

            

    
    def Main(self):
        rospy.spin()

#        # Shutdown all the services we offered.
#        for key in self.services:
#            self.services[key].shutdown()

        

if __name__ == '__main__':
    rospy.init_node('SaveCsv', log_level=rospy.INFO)
    rospy.sleep(1)
    savearenastate = SaveCsv()
    savearenastate.Main()
    
