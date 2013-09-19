#!/usr/bin/env python
from __future__ import division
import sys
import os
import glob
import shutil
import numpy as N


###############################################################################
## How to use this utility:
##
##   1. Scroll to the bottom of this file.
##   2. Edit the lines for your input and output directories.  
##   3. Edit the version number you want to write.
##   4. python ConvertCsv.py
##
##   You're done!  The directory tree will be duplicated, except that all
##   the .csv files will be converted to the specified version.
##   
###############################################################################



def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


class ConvertCsv:
    def __init__(self):

        #######################################################################
        # Structure of the Header lines.
        #######################################################################
        self.headerExperimentTxt_V22 =  'date_time, '\
                                        'description, '\
                                        'maxTrials, '\
                                        'trial\n'
        self.templateExperiment_V22 =   '{date_time:s}, '\
                                        '{description:s}, '\
                                        '{maxTrials:s}, '\
                                        '{trial:s}\n'
                                  
        self.headerRobotSpecTxt_V22 =   'nRobots, '\
                                        'widthRobot, '\
                                        'heightRobot, '\
                                        'visibleRobot, '\
                                        'paintRobot, '\
                                        'scentRobot\n'
        self.templateRobotSpec_V22 =    '{nRobots:s}, '\
                                        '{widthRobot:s}, '\
                                        '{heightRobot:s}, '\
                                        '{visibleRobot:s}, '\
                                        '{paintRobot:s}, '\
                                        '{scentRobot:s}\n'
                                  
        self.headerFlySpecTxt_V22 =     'nFlies, '\
                                        'typeFlies, '\
                                        'genderFlies\n'
        self.templateFlySpec_V22 =      '{nFlies:s}, '\
                                        '{typeFlies:s}, '\
                                        '{genderFlies:s}\n'

        self.headerTrackingTxtA_V22 =   'trackingExclusionzoneEnabled'
        self.headerTrackingTxtB_V22 =   ', trackingExclusionzoneX, '\
                                        'trackingExclusionzoneY, '\
                                        'trackingExclusionzoneRadius'
        self.templateTrackingA_V22 =    '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB_V22 =    ', {trackingExclusionzoneX:s}, '\
                                        '{trackingExclusionzoneY:s}, '\
                                        '{trackingExclusionzoneRadius:s}'

        self.headerPreWait1Txt_V22 =    'waitEntry\n'
        self.templatePreWait1_V22 =     '{preWait1:s}\n'
        
        self.headerPreTriggerTxt_V22 =  'trigger1Enabled, '\
                                        'trigger1FrameidParent, '\
                                        'trigger1FrameidChild, '\
                                        'trigger1SpeedAbsParentMin, '\
                                        'trigger1SpeedAbsParentMax, '\
                                        'trigger1SpeedAbsChildMin, '\
                                        'trigger1SpeedAbsChildMax, '\
                                        'trigger1SpeedRelMin, '\
                                        'trigger1SpeedRelMax, '\
                                        'trigger1DistanceMin, '\
                                        'trigger1DistanceMax, '\
                                        'trigger1AngleMin, '\
                                        'trigger1AngleMax, '\
                                        'trigger1AngleTest, '\
                                        'trigger1AngleTestBilateral, '\
                                        'trigger1TimeHold, '\
                                        'trigger1Timeout\n'
        self.templatePreTrigger_V22 =   '{preTriggerEnabled:s}, '\
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
                                    
        self.headerTrialRobotTxt_V22 =  'moverobotEnabled, '\
                                        'moverobotPatternShape, '\
                                        'moverobotPatternHzPattern, '\
                                        'moverobotPatternHzPoint, '\
                                        'moverobotPatternCount, '\
                                        'moverobotPatternSizeX, '\
                                        'moverobotPatternSizeY, '\
                                        'moverobotPatternParam, '\
                                        'moverobotRelTracking, '\
                                        'moverobotRelOriginPosition, '\
                                        'moverobotRelOriginAngle, '\
                                        'moverobotRelDistance, '\
                                        'moverobotRelAngle, '\
                                        'moverobotRelAngleType, '\
                                        'moverobotRelSpeed, '\
                                        'moverobotRelSpeedType, '\
                                        'moverobotRelTolerance\n'
        self.templateTrialRobot_V22 =   '{trialRobotEnabled:s}, '\
                                        '{trialRobotMovePatternShape:s}, '\
                                        '{trialRobotMovePatternHzPattern:s}, '\
                                        '{trialRobotMovePatternHzPoint:s}, '\
                                        '{trialRobotMovePatternCount:s}, '\
                                        '{trialRobotMovePatternSizeX:s}, '\
                                        '{trialRobotMovePatternSizeY:s}, '\
                                        '{trialRobotMovePatternParam:s}, '\
                                        '{trialRobotMoveRelTracking:s}, '\
                                        '{trialRobotMoveRelOriginPosition:s}, '\
                                        '{trialRobotMoveRelOriginAngle:s}, '\
                                        '{trialRobotMoveRelDistance:s}, '\
                                        '{trialRobotMoveRelAngle:s}, '\
                                        '{trialRobotMoveRelAngleType:s}, '\
                                        '{trialRobotMoveRelSpeed:s}, '\
                                        '{trialRobotMoveRelSpeedType:s}, '\
                                        '{trialRobotMoveRelTolerance:s}\n'
                                    
        self.headerTrialLaserTxt_V22 =  'laserEnabled, '\
                                        'laserPatternShape, '\
                                        'laserPatternHzPattern, '\
                                        'laserPatternHzPoint, '\
                                        'laserPatternCount, '\
                                        'laserPatternSizeX, '\
                                        'laserPatternSizeY, '\
                                        'laserPatternParam, '\
                                        'laserStatefilterLo, '\
                                        'laserStatefilterHi, '\
                                        'laserStatefilterCriteria\n'
        self.templateTrialLaser_V22 =   '{trialLaserEnabled:s}, '\
                                        '{trialLaserPatternShape:s}, '\
                                        '{trialLaserPatternHzPattern:s}, '\
                                        '{trialLaserPatternHzPoint:s}, '\
                                        '{trialLaserPatternCount:s}, '\
                                        '{trialLaserPatternSizeX:s}, '\
                                        '{trialLaserPatternSizeY:s}, '\
                                        '{trialLaserPatternParam:s}, '\
                                        '\"{trialLaserStatefilterLo:s}\", '\
                                        '\"{trialLaserStatefilterHi:s}\", '\
                                        '{trialLaserStatefilterCriteria:s}\n'
        
        self.headerPostTriggerTxt_V22 = 'trigger2Enabled, '\
                                        'trigger2FrameidParent, '\
                                        'trigger2FrameidChild, '\
                                        'trigger2SpeedAbsParentMin, '\
                                        'trigger2SpeedAbsParentMax, '\
                                        'trigger2SpeedAbsChildMin, '\
                                        'trigger2SpeedAbsChildMax, '\
                                        'trigger2SpeedRelMin, '\
                                        'trigger2SpeedRelMax, '\
                                        'trigger2DistanceMin, '\
                                        'trigger2DistanceMax, '\
                                        'trigger2AngleMin, '\
                                        'trigger2AngleMax, '\
                                        'trigger2AngleTest, '\
                                        'trigger2AngleTestBilateral, '\
                                        'trigger2TimeHold, '\
                                        'trigger2Timeout\n'
        self.templatePostTrigger_V22 =  '{postTriggerEnabled:s}, '\
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
                                  
        self.headerPostWaitTxt_V22 =    'waitExit\n'
        self.templatePostWait_V22 =     '{postWait:s}\n'

        ### The state lines.
        self.stateLeftTxt_Vpre26       = 'time'
        self.stateRobotTxt_Vpre26      = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot'
        self.stateFlyTxt_Vpre26        = ', xFly, yFly, aFly, vxFly, vyFly, vaFly'
        self.templateStateLeft_Vpre26  = '{time:s}'
        self.templateStateRobot_Vpre26 = ', {xRobot:s}, {yRobot:s}, {aRobot:s}, {vxRobot:s}, {vyRobot:s}, {vaRobot:s}'
        self.templateStateFly_Vpre26   = ', {xFly:s}, {yFly:s}, {aFly:s}, {vxFly:s}, {vyFly:s}, {vaFly:s}'



        #######################################################################
        self.headerVersionFileTxt_V26 = 'versionFile\n'
        self.templateVersionFile_V26 = '{versionFile:s}\n'
        
        self.headerExperimentTxt_V26 =  self.headerExperimentTxt_V22
        self.templateExperiment_V26 =  self.templateExperiment_V22
                                  
        self.headerRobotSpecTxt_V26 =       self.headerRobotSpecTxt_V22
        self.templateRobotSpec_V26 =       self.templateRobotSpec_V22
                                  
        self.headerFlySpecTxt_V26 =       self.headerFlySpecTxt_V22
        self.templateFlySpec_V26 =       self.templateFlySpec_V22

        self.headerTrackingTxtA_V26 =   self.headerTrackingTxtA_V22
        self.headerTrackingTxtB_V26 =   self.headerTrackingTxtB_V22
        self.templateTrackingA_V26 =   self.templateTrackingA_V22
        self.templateTrackingB_V26 =   self.templateTrackingB_V22

        self.headerPreRobotTxt_V26 =     'preRobotEnabled, '\
                                        'preRobotMovePatternShape, '\
                                        'preRobotMovePatternHzPattern, '\
                                        'preRobotMovePatternHzPoint, '\
                                        'preRobotMovePatternCount, '\
                                        'preRobotMovePatternSizeX, '\
                                        'preRobotMovePatternSizeY, '\
                                        'preRobotMovePatternParam, '\
                                        'preRobotMoveRelTracking, '\
                                        'preRobotMoveRelOriginPosition, '\
                                        'preRobotMoveRelOriginAngle, '\
                                        'preRobotMoveRelDistance, '\
                                        'preRobotMoveRelAngle, '\
                                        'preRobotMoveRelAngleType, '\
                                        'preRobotMoveRelSpeed, '\
                                        'preRobotMoveRelSpeedType, '\
                                        'preRobotMoveRelTolerance, '\
                                        'preRobotMoveTimeout\n'
        self.templatePreRobot_V26 =     '{preRobotEnabled:s}, '\
                                        '{preRobotMovePatternShape:s}, '\
                                        '{preRobotMovePatternHzPattern:s}, '\
                                        '{preRobotMovePatternHzPoint:s}, '\
                                        '{preRobotMovePatternCount:s}, '\
                                        '{preRobotMovePatternSizeX:s}, '\
                                        '{preRobotMovePatternSizeY:s}, '\
                                        '{preRobotMovePatternParam:s}, '\
                                        '{preRobotMoveRelTracking:s}, '\
                                        '{preRobotMoveRelOriginPosition:s}, '\
                                        '{preRobotMoveRelOriginAngle:s}, '\
                                        '{preRobotMoveRelDistance:s}, '\
                                        '{preRobotMoveRelAngle:s}, '\
                                        '{preRobotMoveRelAngleType:s}, '\
                                        '{preRobotMoveRelSpeed:s}, '\
                                        '{preRobotMoveRelSpeedType:s}, '\
                                        '{preRobotMoveRelTolerance:s}, '\
                                        '{preRobotMoveTimeout:s}\n'
                                    
        self.headerPreLaserTxt_V26 =    'preLaserEnabled, '\
                                        'preLaserPatternShape, '\
                                        'preLaserPatternHzPattern, '\
                                        'preLaserPatternHzPoint, '\
                                        'preLaserPatternCount, '\
                                        'preLaserPatternSizeX, '\
                                        'preLaserPatternSizeY, '\
                                        'preLaserPatternParam, '\
                                        'preLaserStatefilterLo, '\
                                        'preLaserStatefilterHi, '\
                                        'preLaserStatefilterCriteria, '\
                                        'preLaserTimeout\n'
        self.templatePreLaser_V26 =     '{preLaserEnabled:s}, '\
                                        '{preLaserPatternShape:s}, '\
                                        '{preLaserPatternHzPattern:s}, '\
                                        '{preLaserPatternHzPoint:s}, '\
                                        '{preLaserPatternCount:s}, '\
                                        '{preLaserPatternSizeX:s}, '\
                                        '{preLaserPatternSizeY:s}, '\
                                        '{preLaserPatternParam:s}, '\
                                        '\"{preLaserStatefilterLo:s}\", '\
                                        '\"{preLaserStatefilterHi:s}\", '\
                                        '{preLaserStatefilterCriteria:s}, '\
                                        '{preLaserTimeout:s}\n'
        
        self.headerPreLEDPanelsTxt_V26 = 'preLEDPanelsEnabled, '\
                                        'preLEDPanelsCommand, '\
                                        'preLEDPanelsIdPattern, '\
                                        'preLEDPanelsFrameid, '\
                                        'preLEDPanelsStatefilterLo, '\
                                        'preLEDPanelsStatefilterHi, '\
                                        'preLEDPanelsStatefilterCriteria, '\
                                        'preLEDPanelsTimeout\n'
        self.templatePreLEDPanels_V26 = '{preLEDPanelsEnabled:s}, '\
                                        '{preLEDPanelsCommand:s}, '\
                                        '{preLEDPanelsIdPattern:s}, '\
                                        '{preLEDPanelsFrameid:s}, '\
                                        '{preLEDPanelsStatefilterLo:s}, '\
                                        '{preLEDPanelsStatefilterHi:s}, '\
                                        '{preLEDPanelsStatefilterCriteria:s}, '\
                                        '{preLEDPanelsTimeout:s}\n'

        self.headerPreWait1Txt_V26 =     'preWait1\n'
        self.templatePreWait1_V26 =     '{preWait1:s}\n'
        
        self.headerPreTriggerTxt_V26 =   'preTriggerEnabled, '\
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
        self.templatePreTrigger_V26 =   '{preTriggerEnabled:s}, '\
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
                                    
        self.headerPreWait2Txt_V26 =    'preWait2\n'
        self.templatePreWait2_V26 =    '{preWait2:s}\n'
        
        self.headerTrialRobotTxt_V26 =   'trialRobotEnabled, '\
                                        'trialRobotMovePatternShape, '\
                                        'trialRobotMovePatternHzPattern, '\
                                        'trialRobotMovePatternHzPoint, '\
                                        'trialRobotMovePatternCount, '\
                                        'trialRobotMovePatternSizeX, '\
                                        'trialRobotMovePatternSizeY, '\
                                        'trialRobotMovePatternParam, '\
                                        'trialRobotMoveRelTracking, '\
                                        'trialRobotMoveRelOriginPosition, '\
                                        'trialRobotMoveRelOriginAngle, '\
                                        'trialRobotMoveRelDistance, '\
                                        'trialRobotMoveRelAngle, '\
                                        'trialRobotMoveRelAngleType, '\
                                        'trialRobotMoveRelSpeed, '\
                                        'trialRobotMoveRelSpeedType, '\
                                        'trialRobotMoveRelTolerance, '\
                                        'trialRobotMoveTimeout\n'
        self.templateTrialRobot_V26 =   '{trialRobotEnabled:s}, '\
                                        '{trialRobotMovePatternShape:s}, '\
                                        '{trialRobotMovePatternHzPattern:s}, '\
                                        '{trialRobotMovePatternHzPoint:s}, '\
                                        '{trialRobotMovePatternCount:s}, '\
                                        '{trialRobotMovePatternSizeX:s}, '\
                                        '{trialRobotMovePatternSizeY:s}, '\
                                        '{trialRobotMovePatternParam:s}, '\
                                        '{trialRobotMoveRelTracking:s}, '\
                                        '{trialRobotMoveRelOriginPosition:s}, '\
                                        '{trialRobotMoveRelOriginAngle:s}, '\
                                        '{trialRobotMoveRelDistance:s}, '\
                                        '{trialRobotMoveRelAngle:s}, '\
                                        '{trialRobotMoveRelAngleType:s}, '\
                                        '{trialRobotMoveRelSpeed:s}, '\
                                        '{trialRobotMoveRelSpeedType:s}, '\
                                        '{trialRobotMoveRelTolerance:s}, '\
                                        '{trialRobotMoveTimeout:s}\n'
                                    
        self.headerTrialLaserTxt_V26 =   'trialLaserEnabled, '\
                                        'trialLaserPatternShape, '\
                                        'trialLaserPatternHzPattern, '\
                                        'trialLaserPatternHzPoint, '\
                                        'trialLaserPatternCount, '\
                                        'trialLaserPatternSizeX, '\
                                        'trialLaserPatternSizeY, '\
                                        'trialLaserPatternParam, '\
                                        'trialLaserStatefilterLo, '\
                                        'trialLaserStatefilterHi, '\
                                        'trialLaserStatefilterCriteria, '\
                                        'trialLaserTimeout\n'
        self.templateTrialLaser_V26 =   '{trialLaserEnabled:s}, '\
                                        '{trialLaserPatternShape:s}, '\
                                        '{trialLaserPatternHzPattern:s}, '\
                                        '{trialLaserPatternHzPoint:s}, '\
                                        '{trialLaserPatternCount:s}, '\
                                        '{trialLaserPatternSizeX:s}, '\
                                        '{trialLaserPatternSizeY:s}, '\
                                        '{trialLaserPatternParam:s}, '\
                                        '\"{trialLaserStatefilterLo:s}\", '\
                                        '\"{trialLaserStatefilterHi:s}\", '\
                                        '{trialLaserStatefilterCriteria:s}, '\
                                        '{trialLaserTimeout:s}\n'
        
        self.headerTrialLEDPanelsTxt_V26 = 'trialLEDPanelsEnabled, '\
                                        'trialLEDPanelsCommand, '\
                                        'trialLEDPanelsIdPattern, '\
                                        'trialLEDPanelsFrameid, '\
                                        'trialLEDPanelsStatefilterLo, '\
                                        'trialLEDPanelsStatefilterHi, '\
                                        'trialLEDPanelsStatefilterCriteria, '\
                                        'trialLEDPanelsTimeout\n'
        self.templateTrialLEDPanels_V26 = '{trialLEDPanelsEnabled:s}, '\
                                        '{trialLEDPanelsCommand:s}, '\
                                        '{trialLEDPanelsIdPattern:s}, '\
                                        '{trialLEDPanelsFrameid:s}, '\
                                        '{trialLEDPanelsStatefilterLo:s}, '\
                                        '{trialLEDPanelsStatefilterHi:s}, '\
                                        '{trialLEDPanelsStatefilterCriteria:s}, '\
                                        '{trialLEDPanelsTimeout:s}\n'
        
        self.headerPostTriggerTxt_V26 =  'postTriggerEnabled, '\
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
        self.templatePostTrigger_V26 =  '{postTriggerEnabled:s}, '\
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
                                  
        self.headerPostWaitTxt_V26 =     'postWait\n'
        self.templatePostWait_V26 =     '{postWait:s}\n'


        # The state lines.
        self.stateLeftTxt_V26           = 'time, triggered'
        self.stateRobotTxt_V26          = self.stateRobotTxt_Vpre26
        self.stateFlyTxt_V26            = self.stateFlyTxt_Vpre26
        self.templateStateLeft_V26      = '{time:s}, {triggered:s}'
        self.templateStateRobot_V26     = self.templateStateRobot_Vpre26
        self.templateStateFly_V26       = self.templateStateFly_Vpre26

       
       
        #######################################################################
        self.headerVersionFileTxt_V27 = self.headerVersionFileTxt_V26
        self.templateVersionFile_V27 = self.templateVersionFile_V26
        self.headerExperimentTxt_V27 =  self.headerExperimentTxt_V26
        self.templateExperiment_V27 =  self.templateExperiment_V26
        self.headerRobotSpecTxt_V27 =       self.headerRobotSpecTxt_V26
        self.templateRobotSpec_V27 =      self.templateRobotSpec_V26
        self.headerFlySpecTxt_V27 =       self.headerFlySpecTxt_V26
        self.templateFlySpec_V27 =       self.templateFlySpec_V26
        self.headerTrackingTxtA_V27 =   self.headerTrackingTxtA_V26
        self.headerTrackingTxtB_V27 =   self.headerTrackingTxtB_V26
        self.templateTrackingA_V27 =   self.templateTrackingA_V26
        self.templateTrackingB_V27 =   self.templateTrackingB_V26
        self.headerPreRobotTxt_V27 =    self.headerPreRobotTxt_V26
        self.templatePreRobot_V27 =    self.templatePreRobot_V26
        self.headerPreLaserTxt_V27 =    self.headerPreLaserTxt_V26
        self.templatePreLaser_V27 =    self.templatePreLaser_V26
        self.headerPreLEDPanelsTxt_V27 =self.headerPreLEDPanelsTxt_V26
        self.templatePreLEDPanels_V27 =self.templatePreLEDPanels_V26
        self.headerPreWait1Txt_V27 =    self.headerPreWait1Txt_V26
        self.templatePreWait1_V27 =    self.templatePreWait1_V26
        self.headerPreTriggerTxt_V27 =  self.headerPreTriggerTxt_V26
        self.templatePreTrigger_V27 =  self.templatePreTrigger_V26
        self.headerPreWait2Txt_V27 =    self.headerPreWait2Txt_V26
        self.templatePreWait2_V27 =    self.templatePreWait2_V26
        self.headerTrialRobotTxt_V27 =  self.headerTrialRobotTxt_V26
        self.templateTrialRobot_V27 =  self.templateTrialRobot_V26
        self.headerTrialLaserTxt_V27 =  self.headerTrialLaserTxt_V26
        self.templateTrialLaser_V27 =  self.templateTrialLaser_V26
        self.headerTrialLEDPanelsTxt_V27 = self.headerTrialLEDPanelsTxt_V26
        self.templateTrialLEDPanels_V27 = self.templateTrialLEDPanels_V26
        self.headerPostTriggerTxt_V27 = self.headerPostTriggerTxt_V26
        self.templatePostTrigger_V27 = self.templatePostTrigger_V26
        self.headerPostWaitTxt_V27 =    self.headerPostWaitTxt_V26
        self.templatePostWait_V27 =    self.templatePostWait_V26

        
        # The state lines.
        self.stateLeftTxt_V27           = self.stateLeftTxt_V26
        self.stateRobotTxt_V27          = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, aRobotWingLeft, aRobotWingRight'
        self.stateFlyTxt_V27            = ', xFly, yFly, aFly, vxFly, vyFly, vaFly, aFlyWingLeft, aFlyWingRight'
        self.templateStateLeft_V27      = self.templateStateLeft_V26
        self.templateStateRobot_V27     = ', {xRobot:s}, {yRobot:s}, {aRobot:s}, {vxRobot:s}, {vyRobot:s}, {vaRobot:s}, {aRobotWingLeft:s}, {aRobotWingRight:s}'
        self.templateStateFly_V27       = ', {xFly:s}, {yFly:s}, {aFly:s}, {vxFly:s}, {vyFly:s}, {vaFly:s}, {aFlyWingLeft:s}, {aFlyWingRight:s}'
        
        
        
        #######################################################################
        self.headerVersionFileTxt_V28 = self.headerVersionFileTxt_V27
        self.templateVersionFile_V28 = self.templateVersionFile_V27
        self.headerExperimentTxt_V28 =  self.headerExperimentTxt_V27
        self.templateExperiment_V28 =  self.templateExperiment_V27
        self.headerRobotSpecTxt_V28 =       self.headerRobotSpecTxt_V27
        self.templateRobotSpec_V28 =      self.templateRobotSpec_V27
        self.headerFlySpecTxt_V28 =       self.headerFlySpecTxt_V27
        self.templateFlySpec_V28 =       self.templateFlySpec_V27
        self.headerTrackingTxtA_V28 =   self.headerTrackingTxtA_V27
        self.headerTrackingTxtB_V28 =   self.headerTrackingTxtB_V27
        self.templateTrackingA_V28 =   self.templateTrackingA_V27
        self.templateTrackingB_V28 =   self.templateTrackingB_V27
        self.headerPreRobotTxt_V28 =    'preRobotEnabled, '\
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
                                        'preRobotMoveRelOriginPosition, '\
                                        'preRobotMoveRelOriginAngle, '\
                                        'preRobotMoveRelDistance, '\
                                        'preRobotMoveRelAngle, '\
                                        'preRobotMoveRelAngleType, '\
                                        'preRobotMoveRelSpeed, '\
                                        'preRobotMoveRelSpeedType, '\
                                        'preRobotMoveRelTolerance, '\
                                        'preRobotMoveTimeout\n'
        self.templatePreRobot_V28 =     '{preRobotEnabled:s}, '\
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
                                        '{preRobotMoveRelOriginPosition:s}, '\
                                        '{preRobotMoveRelOriginAngle:s}, '\
                                        '{preRobotMoveRelDistance:s}, '\
                                        '{preRobotMoveRelAngle:s}, '\
                                        '{preRobotMoveRelAngleType:s}, '\
                                        '{preRobotMoveRelSpeed:s}, '\
                                        '{preRobotMoveRelSpeedType:s}, '\
                                        '{preRobotMoveRelTolerance:s}, '\
                                        '{preRobotMoveTimeout:s}\n'
                                    
        self.headerPreLaserTxt_V28 =    'preLaserEnabled, '\
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
                                        'preLaserStatefilterCriteria, '\
                                        'preLaserTimeout\n'
        self.templatePreLaser_V28 =     '{preLaserEnabled:s}, '\
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
                                        '{preLaserStatefilterCriteria:s}, '\
                                        '{preLaserTimeout:s}\n'
        self.headerPreLEDPanelsTxt_V28 =self.headerPreLEDPanelsTxt_V27
        self.templatePreLEDPanels_V28 =self.templatePreLEDPanels_V27
        self.headerPreWait1Txt_V28 =    self.headerPreWait1Txt_V27
        self.templatePreWait1_V28 =    self.templatePreWait1_V27
        self.headerPreTriggerTxt_V28 =  self.headerPreTriggerTxt_V27
        self.templatePreTrigger_V28 =  self.templatePreTrigger_V27
        self.headerPreWait2Txt_V28 =    self.headerPreWait2Txt_V27
        self.templatePreWait2_V28 =    self.templatePreWait2_V27
        self.headerTrialRobotTxt_V28 =  'trialRobotEnabled, '\
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
                                        'trialRobotMoveRelOriginPosition, '\
                                        'trialRobotMoveRelOriginAngle, '\
                                        'trialRobotMoveRelDistance, '\
                                        'trialRobotMoveRelAngle, '\
                                        'trialRobotMoveRelAngleType, '\
                                        'trialRobotMoveRelSpeed, '\
                                        'trialRobotMoveRelSpeedType, '\
                                        'trialRobotMoveRelTolerance, '\
                                        'trialRobotMoveTimeout\n'
        self.templateTrialRobot_V28 =   '{trialRobotEnabled:s}, '\
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
                                        '{trialRobotMoveRelOriginPosition:s}, '\
                                        '{trialRobotMoveRelOriginAngle:s}, '\
                                        '{trialRobotMoveRelDistance:s}, '\
                                        '{trialRobotMoveRelAngle:s}, '\
                                        '{trialRobotMoveRelAngleType:s}, '\
                                        '{trialRobotMoveRelSpeed:s}, '\
                                        '{trialRobotMoveRelSpeedType:s}, '\
                                        '{trialRobotMoveRelTolerance:s}, '\
                                        '{trialRobotMoveTimeout:s}\n'
                                    
        self.headerTrialLaserTxt_V28 =  'trialLaserEnabled, '\
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
                                        'trialLaserStatefilterCriteria, '\
                                        'trialLaserTimeout\n'
        self.templateTrialLaser_V28 =   '{trialLaserEnabled:s}, '\
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
                                        '{trialLaserStatefilterCriteria:s}, '\
                                        '{trialLaserTimeout:s}\n'
        self.headerTrialLEDPanelsTxt_V28 = self.headerTrialLEDPanelsTxt_V27
        self.templateTrialLEDPanels_V28 = self.templateTrialLEDPanels_V27
        self.headerPostTriggerTxt_V28 = self.headerPostTriggerTxt_V27
        self.templatePostTrigger_V28 = self.templatePostTrigger_V27
        self.headerPostWaitTxt_V28 =    self.headerPostWaitTxt_V27
        self.templatePostWait_V28 =    self.templatePostWait_V27

        
        # The state lines.
        self.stateLeftTxt_V28           = self.stateLeftTxt_V27
        self.stateRobotTxt_V28          = self.stateRobotTxt_V27
        self.stateFlyTxt_V28            = self.stateFlyTxt_V27
        self.templateStateLeft_V28      = self.templateStateLeft_V27
        self.templateStateRobot_V28     = self.templateStateRobot_V27
        self.templateStateFly_V28       = self.templateStateFly_V27
        
        
        #######################################################################
        # version 2.81 removes the timeout fields.
        self.headerVersionFileTxt_V281 =self.headerVersionFileTxt_V28
        self.templateVersionFile_V281 = self.templateVersionFile_V28
        self.headerExperimentTxt_V281 = self.headerExperimentTxt_V28
        self.templateExperiment_V281 =  self.templateExperiment_V28
        self.headerRobotSpecTxt_V281 =      self.headerRobotSpecTxt_V28
        self.templateRobotSpec_V281 =      self.templateRobotSpec_V28
        self.headerFlySpecTxt_V281 =      self.headerFlySpecTxt_V28
        self.templateFlySpec_V281 =       self.templateFlySpec_V28
        self.headerTrackingTxtA_V281 =  self.headerTrackingTxtA_V28
        self.headerTrackingTxtB_V281 =  self.headerTrackingTxtB_V28
        self.templateTrackingA_V281 =   self.templateTrackingA_V28
        self.templateTrackingB_V281 =   self.templateTrackingB_V28
        self.headerPreRobotTxt_V281 =   'preRobotEnabled, '\
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
                                        'preRobotMoveRelOriginPosition, '\
                                        'preRobotMoveRelOriginAngle, '\
                                        'preRobotMoveRelDistance, '\
                                        'preRobotMoveRelAngle, '\
                                        'preRobotMoveRelAngleType, '\
                                        'preRobotMoveRelSpeed, '\
                                        'preRobotMoveRelSpeedType, '\
                                        'preRobotMoveRelTolerance\n'
        self.templatePreRobot_V281 =    '{preRobotEnabled:s}, '\
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
                                        '{preRobotMoveRelOriginPosition:s}, '\
                                        '{preRobotMoveRelOriginAngle:s}, '\
                                        '{preRobotMoveRelDistance:s}, '\
                                        '{preRobotMoveRelAngle:s}, '\
                                        '{preRobotMoveRelAngleType:s}, '\
                                        '{preRobotMoveRelSpeed:s}, '\
                                        '{preRobotMoveRelSpeedType:s}, '\
                                        '{preRobotMoveRelTolerance:s}\n'
                                    
        self.headerPreLaserTxt_V281 =   'preLaserEnabled, '\
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
        self.templatePreLaser_V281 =     '{preLaserEnabled:s}, '\
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
        self.headerPreLEDPanelsTxt_V281 = 'preLEDPanelsEnabled, '\
                                        'preLEDPanelsCommand, '\
                                        'preLEDPanelsIdPattern, '\
                                        'preLEDPanelsFrameid, '\
                                        'preLEDPanelsStatefilterLo, '\
                                        'preLEDPanelsStatefilterHi, '\
                                        'preLEDPanelsStatefilterCriteria\n'
        self.templatePreLEDPanels_V281 = '{preLEDPanelsEnabled:s}, '\
                                        '{preLEDPanelsCommand:s}, '\
                                        '{preLEDPanelsIdPattern:s}, '\
                                        '{preLEDPanelsFrameid:s}, '\
                                        '{preLEDPanelsStatefilterLo:s}, '\
                                        '{preLEDPanelsStatefilterHi:s}, '\
                                        '{preLEDPanelsStatefilterCriteria:s}\n'
        self.headerPreWait1Txt_V281 =    self.headerPreWait1Txt_V28
        self.templatePreWait1_V281 =    self.templatePreWait1_V28
        self.headerPreTriggerTxt_V281 =  self.headerPreTriggerTxt_V28
        self.templatePreTrigger_V281 =  self.templatePreTrigger_V28
        self.headerPreWait2Txt_V281 =    self.headerPreWait2Txt_V28
        self.templatePreWait2_V281 =    self.templatePreWait2_V28
        self.headerTrialRobotTxt_V281 =   'trialRobotEnabled, '\
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
                                        'trialRobotMoveRelOriginPosition, '\
                                        'trialRobotMoveRelOriginAngle, '\
                                        'trialRobotMoveRelDistance, '\
                                        'trialRobotMoveRelAngle, '\
                                        'trialRobotMoveRelAngleType, '\
                                        'trialRobotMoveRelSpeed, '\
                                        'trialRobotMoveRelSpeedType, '\
                                        'trialRobotMoveRelTolerance\n'
        self.templateTrialRobot_V281 =   '{trialRobotEnabled:s}, '\
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
                                        '{trialRobotMoveRelOriginPosition:s}, '\
                                        '{trialRobotMoveRelOriginAngle:s}, '\
                                        '{trialRobotMoveRelDistance:s}, '\
                                        '{trialRobotMoveRelAngle:s}, '\
                                        '{trialRobotMoveRelAngleType:s}, '\
                                        '{trialRobotMoveRelSpeed:s}, '\
                                        '{trialRobotMoveRelSpeedType:s}, '\
                                        '{trialRobotMoveRelTolerance:s}\n'
                                    
        self.headerTrialLaserTxt_V281 =   'trialLaserEnabled, '\
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
        self.templateTrialLaser_V281 =   '{trialLaserEnabled:s}, '\
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
        self.headerTrialLEDPanelsTxt_V281 = 'trialLEDPanelsEnabled, '\
                                        'trialLEDPanelsCommand, '\
                                        'trialLEDPanelsIdPattern, '\
                                        'trialLEDPanelsFrameid, '\
                                        'trialLEDPanelsStatefilterLo, '\
                                        'trialLEDPanelsStatefilterHi, '\
                                        'trialLEDPanelsStatefilterCriteria\n'
        self.templateTrialLEDPanels_V281 = '{trialLEDPanelsEnabled:s}, '\
                                        '{trialLEDPanelsCommand:s}, '\
                                        '{trialLEDPanelsIdPattern:s}, '\
                                        '{trialLEDPanelsFrameid:s}, '\
                                        '{trialLEDPanelsStatefilterLo:s}, '\
                                        '{trialLEDPanelsStatefilterHi:s}, '\
                                        '{trialLEDPanelsStatefilterCriteria:s}\n'
        self.headerPostTriggerTxt_V281 = self.headerPostTriggerTxt_V28
        self.templatePostTrigger_V281 = self.templatePostTrigger_V28
        self.headerPostWaitTxt_V281 =    self.headerPostWaitTxt_V28
        self.templatePostWait_V281 =    self.templatePostWait_V28

        
        # The state lines.
        self.stateLeftTxt_V281           = self.stateLeftTxt_V28
        self.stateRobotTxt_V281          = self.stateRobotTxt_V28
        self.stateFlyTxt_V281            = self.stateFlyTxt_V28
        self.templateStateLeft_V281      = self.templateStateLeft_V28
        self.templateStateRobot_V281     = self.templateStateRobot_V28
        self.templateStateFly_V281       = self.templateStateFly_V28
        
        
        #######################################################################
        # version 2.82 adds MoveRelAngleOscMag, MoveRelAngleOscFreq, and MoveRelAngle->MoveRelAngleOffset 
        self.headerVersionFileTxt_V282 =self.headerVersionFileTxt_V281
        self.templateVersionFile_V282 = self.templateVersionFile_V281
        self.headerExperimentTxt_V282 = self.headerExperimentTxt_V281
        self.templateExperiment_V282 =  self.templateExperiment_V281
        self.headerRobotSpecTxt_V282 =      self.headerRobotSpecTxt_V281
        self.templateRobotSpec_V282 =      self.templateRobotSpec_V281
        self.headerFlySpecTxt_V282 =      self.headerFlySpecTxt_V281
        self.templateFlySpec_V282 =       self.templateFlySpec_V281
        self.headerTrackingTxtA_V282 =  self.headerTrackingTxtA_V281
        self.headerTrackingTxtB_V282 =  self.headerTrackingTxtB_V281
        self.templateTrackingA_V282 =   self.templateTrackingA_V281
        self.templateTrackingB_V282 =   self.templateTrackingB_V281
        self.headerPreRobotTxt_V282 =   'preRobotEnabled, '\
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
                                        'preRobotMoveRelOriginPosition, '\
                                        'preRobotMoveRelOriginAngle, '\
                                        'preRobotMoveRelDistance, '\
                                        'preRobotMoveRelAngleOffset, '\
                                        'preRobotMoveRelAngleType, '\
                                        'preRobotMoveRelAngleOscMag, '\
                                        'preRobotMoveRelAngleOscFreq, '\
                                        'preRobotMoveRelSpeed, '\
                                        'preRobotMoveRelSpeedType, '\
                                        'preRobotMoveRelTolerance\n'
        self.templatePreRobot_V282 =    '{preRobotEnabled:s}, '\
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
                                        '{preRobotMoveRelOriginPosition:s}, '\
                                        '{preRobotMoveRelOriginAngle:s}, '\
                                        '{preRobotMoveRelDistance:s}, '\
                                        '{preRobotMoveRelAngleOffset:s}, '\
                                        '{preRobotMoveRelAngleType:s}, '\
                                        '{preRobotMoveRelAngleOscMag:s}, '\
                                        '{preRobotMoveRelAngleOscFreq:s}, '\
                                        '{preRobotMoveRelSpeed:s}, '\
                                        '{preRobotMoveRelSpeedType:s}, '\
                                        '{preRobotMoveRelTolerance:s}\n'
                                    
        self.headerPreLaserTxt_V282 =   self.headerPreLaserTxt_V281
        self.templatePreLaser_V282 =    self.templatePreLaser_V281
        self.headerPreLEDPanelsTxt_V282 = self.headerPreLEDPanelsTxt_V281
        self.templatePreLEDPanels_V282 = self.templatePreLEDPanels_V281
        self.headerPreWait1Txt_V282 =   self.headerPreWait1Txt_V281
        self.templatePreWait1_V282 =    self.templatePreWait1_V281
        self.headerPreTriggerTxt_V282 = self.headerPreTriggerTxt_V281
        self.templatePreTrigger_V282 =  self.templatePreTrigger_V281
        self.headerPreWait2Txt_V282 =   self.headerPreWait2Txt_V281
        self.templatePreWait2_V282 =    self.templatePreWait2_V281
        self.headerTrialRobotTxt_V282 = 'trialRobotEnabled, '\
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
                                        'trialRobotMoveRelOriginPosition, '\
                                        'trialRobotMoveRelOriginAngle, '\
                                        'trialRobotMoveRelDistance, '\
                                        'trialRobotMoveRelAngleOffset, '\
                                        'trialRobotMoveRelAngleType, '\
                                        'trialRobotMoveRelAngleOscMag, '\
                                        'trialRobotMoveRelAngleOscFreq, '\
                                        'trialRobotMoveRelSpeed, '\
                                        'trialRobotMoveRelSpeedType, '\
                                        'trialRobotMoveRelTolerance\n'
        self.templateTrialRobot_V282 =  '{trialRobotEnabled:s}, '\
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
                                        '{trialRobotMoveRelOriginPosition:s}, '\
                                        '{trialRobotMoveRelOriginAngle:s}, '\
                                        '{trialRobotMoveRelDistance:s}, '\
                                        '{trialRobotMoveRelAngleOffset:s}, '\
                                        '{trialRobotMoveRelAngleType:s}, '\
                                        '{trialRobotMoveRelAngleOscMag:s}, '\
                                        '{trialRobotMoveRelAngleOscFreq:s}, '\
                                        '{trialRobotMoveRelSpeed:s}, '\
                                        '{trialRobotMoveRelSpeedType:s}, '\
                                        '{trialRobotMoveRelTolerance:s}\n'
                                    
        self.headerTrialLaserTxt_V282 =  self.headerTrialLaserTxt_V281
        self.templateTrialLaser_V282 =   self.templateTrialLaser_V281
        self.headerTrialLEDPanelsTxt_V282 = self.headerTrialLEDPanelsTxt_V281
        self.templateTrialLEDPanels_V282 = self.templateTrialLEDPanels_V281
        self.headerPostTriggerTxt_V282 = self.headerPostTriggerTxt_V281
        self.templatePostTrigger_V282 = self.templatePostTrigger_V281
        self.headerPostWaitTxt_V282 =    self.headerPostWaitTxt_V281
        self.templatePostWait_V282 =    self.templatePostWait_V281

        
        # The state lines.
        self.stateLeftTxt_V282           = self.stateLeftTxt_V281
        self.stateRobotTxt_V282          = self.stateRobotTxt_V281
        self.stateFlyTxt_V282            = self.stateFlyTxt_V281
        self.templateStateLeft_V282      = self.templateStateLeft_V281
        self.templateStateRobot_V282     = self.templateStateRobot_V281
        self.templateStateFly_V282       = self.templateStateFly_V281

        
        #######################################################################
        # version 2.83 changes the fields for robotspec and flyspec. 
        self.headerVersionFileTxt_V283 =self.headerVersionFileTxt_V282
        self.templateVersionFile_V283 = self.templateVersionFile_V282
        self.headerExperimentTxt_V283 = self.headerExperimentTxt_V282
        self.templateExperiment_V283 =  self.templateExperiment_V282
        self.headerRobotSpecTxt_V283 =  'nRobots, '\
                                        'widthRobot, '\
                                        'heightRobot, '\
                                        'descriptionRobot\n'
        self.templateRobotSpec_V283 =   '{nRobots:s}, '\
                                        '{widthRobot:s}, '\
                                        '{heightRobot:s}, '\
                                        '{descriptionRobot:s}\n'
                                  
        self.headerFlySpecTxt_V283 =    'nFlies, '\
                                        'descriptionFlies\n'
        self.templateFlySpec_V283 =     '{nFlies:s}, '\
                                        '{descriptionFlies:s}\n'
        self.headerTrackingTxtA_V283 =  self.headerTrackingTxtA_V282
        self.headerTrackingTxtB_V283 =  self.headerTrackingTxtB_V282
        self.templateTrackingA_V283 =   self.templateTrackingA_V282
        self.templateTrackingB_V283 =   self.templateTrackingB_V282
        self.headerPreRobotTxt_V283 =   self.headerPreRobotTxt_V282
        self.templatePreRobot_V283 =    self.templatePreRobot_V282
                                    
        self.headerPreLaserTxt_V283 =   self.headerPreLaserTxt_V282
        self.templatePreLaser_V283 =    self.templatePreLaser_V282
        self.headerPreLEDPanelsTxt_V283 = self.headerPreLEDPanelsTxt_V282
        self.templatePreLEDPanels_V283 = self.templatePreLEDPanels_V282
        self.headerPreWait1Txt_V283 =   self.headerPreWait1Txt_V282
        self.templatePreWait1_V283 =    self.templatePreWait1_V282
        self.headerPreTriggerTxt_V283 = self.headerPreTriggerTxt_V282
        self.templatePreTrigger_V283 =  self.templatePreTrigger_V282
        self.headerPreWait2Txt_V283 =   self.headerPreWait2Txt_V282
        self.templatePreWait2_V283 =    self.templatePreWait2_V282
        self.headerTrialRobotTxt_V283 = self.headerTrialRobotTxt_V282
        self.templateTrialRobot_V283 =  self.templateTrialRobot_V282
                                    
        self.headerTrialLaserTxt_V283 =  self.headerTrialLaserTxt_V282
        self.templateTrialLaser_V283 =   self.templateTrialLaser_V282
        self.headerTrialLEDPanelsTxt_V283 = self.headerTrialLEDPanelsTxt_V282
        self.templateTrialLEDPanels_V283 = self.templateTrialLEDPanels_V282
        self.headerPostTriggerTxt_V283 = self.headerPostTriggerTxt_V282
        self.templatePostTrigger_V283 = self.templatePostTrigger_V282
        self.headerPostWaitTxt_V283 =    self.headerPostWaitTxt_V282
        self.templatePostWait_V283 =    self.templatePostWait_V282

        
        # The state lines.
        self.stateLeftTxt_V283           = self.stateLeftTxt_V282
        self.stateRobotTxt_V283          = self.stateRobotTxt_V282
        self.stateFlyTxt_V283            = self.stateFlyTxt_V282
        self.templateStateLeft_V283      = self.templateStateLeft_V282
        self.templateStateRobot_V283     = self.templateStateRobot_V282
        self.templateStateFly_V283       = self.templateStateFly_V282
        

        #######################################################################
#         # version 999 changes... 
#         self.headerVersionFileTxt_V999 =self.headerVersionFileTxt_V888
#         self.templateVersionFile_V999 = self.templateVersionFile_V888
#         self.headerExperimentTxt_V999 = self.headerExperimentTxt_V888
#         self.templateExperiment_V999 =  self.templateExperiment_V888
#         self.headerRobotSpecTxt_V999 =      self.headerRobotSpecTxt_V888
#         self.templateRobotSpec_V999 =      self.templateRobotSpec_V888
#         self.headerFlySpecTxt_V999 =      self.headerFlySpecTxt_V888
#         self.templateFlySpec_V999 =       self.templateFlySpec_V888
#         self.headerTrackingTxtA_V999 =  self.headerTrackingTxtA_V888
#         self.headerTrackingTxtB_V999 =  self.headerTrackingTxtB_V888
#         self.templateTrackingA_V999 =   self.templateTrackingA_V888
#         self.templateTrackingB_V999 =   self.templateTrackingB_V888
#         self.headerPreRobotTxt_V999 =   self.headerPreRobotTxt_V888
#         self.templatePreRobot_V999 =    self.templatePreRobot_V888
#                                     
#         self.headerPreLaserTxt_V999 =   self.headerPreLaserTxt_V888
#         self.templatePreLaser_V999 =    self.templatePreLaser_V888
#         self.headerPreLEDPanelsTxt_V999 = self.headerPreLEDPanelsTxt_V888
#         self.templatePreLEDPanels_V999 = self.templatePreLEDPanels_V888
#         self.headerPreWait1Txt_V999 =   self.headerPreWait1Txt_V888
#         self.templatePreWait1_V999 =    self.templatePreWait1_V888
#         self.headerPreTriggerTxt_V999 = self.headerPreTriggerTxt_V888
#         self.templatePreTrigger_V999 =  self.templatePreTrigger_V888
#         self.headerPreWait2Txt_V999 =   self.headerPreWait2Txt_V888
#         self.templatePreWait2_V999 =    self.templatePreWait2_V888
#         self.headerTrialRobotTxt_V999 = self.headerTrialRobotTxt_V888
#         self.templateTrialRobot_V999 =  self.templateTrialRobot_V888
#                                     
#         self.headerTrialLaserTxt_V999 =  self.headerTrialLaserTxt_V888
#         self.templateTrialLaser_V999 =   self.templateTrialLaser_V888
#         self.headerTrialLEDPanelsTxt_V999 = self.headerTrialLEDPanelsTxt_V888
#         self.templateTrialLEDPanels_V999 = self.templateTrialLEDPanels_V888
#         self.headerPostTriggerTxt_V999 = self.headerPostTriggerTxt_V888
#         self.templatePostTrigger_V999 = self.templatePostTrigger_V888
#         self.headerPostWaitTxt_V999 =    self.headerPostWaitTxt_V888
#         self.templatePostWait_V999 =    self.templatePostWait_V888
# 
#         
#         # The state lines.
#         self.stateLeftTxt_V999           = self.stateLeftTxt_V888
#         self.stateRobotTxt_V999          = self.stateRobotTxt_V888
#         self.stateFlyTxt_V999            = self.stateFlyTxt_V888
#         self.templateStateLeft_V999      = self.templateStateLeft_V888
#         self.templateStateRobot_V999     = self.templateStateRobot_V888
#         self.templateStateFly_V999       = self.templateStateFly_V888
        
        #######################################################################
        


    # Advance the file seek position until the next line to be read has fieldGoal as the first csv field.
    # Returns the number of lines before the goal line.
    #
    def AdvanceUntilField(self, fid, fieldGoal):
        field = 'bogus'
        nLines = -1
        while field != fieldGoal:
            pos = fid.tell()
            line = fid.readline()
            field = line.split(',')[0].strip(' \n')
            nLines += 1
            
        fid.seek(pos)
        return nLines
        

    # GetFileInfo()
    # Return the (version,nLinesHeader,nRobots,nFlies) of a Flylab .csv file.
    #                
    def GetFileInfo(self, filename):
        with open(filename, 'r') as fid:
            posOrigin = fid.tell()
            nLinesHeader = self.AdvanceUntilField(fid, 'time')
            fid.seek(posOrigin)

            # Read all the lines in the header.
            line = []
            for i in range(nLinesHeader):
                line.append(fid.readline())
                
            # Read the state header line.
            lineHeaderState = fid.readline()
            
            
            # Determine the version from which header are on which lines, or from the version number.
            field = line[0].split(',')[0].strip(' \n')
            if field=='versionFile':
                field = line[1].split(',')[0].strip(' \n')
                version = field
            else:
                field = line[3].split(',')[0].strip(' \n')
                if field=='time':
                    field = line[0].split(',')[4].strip(' \n')
                    if field=='waitEntry':
                        version = '1.0'
                    elif field=='robot_width':
                        version = '1.1'
                    else:
                        version = '9999'
    
                elif field=='nRobots':
                    field = line[9].split(',')[0].strip(' \n')
                    if field=='waitEntry':
                        version = '2.0'
                    elif field=='trackingExclusionzoneEnabled':
                        field = line[12].split(',')[0].strip(' \n')
                        if field=='waitEntry':
                            field = line[15].split(',')[7].strip(' \n')
                            if field=='trigger1SpeedRelMin':
                                field = line[27].split(',')[0].strip(' \n')
                                if field=='ledpanelsEnabled':
                                    version = '2.3'
                                else:
                                    version = '2.2'
                            else:
                                version = '2.1'
                                
                        elif field=='waitEntry1':
                            field = line[27].split(',')[0].strip(' \n')
                            if field=='trigger2Enabled':
                                version = '2.4'
                            elif field=='ledpanelsEnabled':
                                version = '2.5'
                            else:
                                version = '9998'
                        else:
                            version = '9997'
                    else:
                        version = '9996'
    
                else:
                    version = '9995'


            # Count the robots & flies.
            nFields = len(lineHeaderState.split(','))
            nFields -= 1  # For the 'time' field.
                        
            field = lineHeaderState.split(',')[1].strip(' \n')
            if field=='triggered':
                nFields -= 1  # For the 'triggered' field.

            # Number of state columns is six or eight:  (x,y,a,vx,vy,va) or (x,y,a,vx,vy,va,al,ar) for each robot or fly.
            if (float(version) < 2.7):
                nObjects = int(nFields/6)
                
            elif (float(version) >= 2.7):
                nObjects = int(nFields/8)
            
            
            nRobots = 1 # There's always a robot in the file.
            nFlies = nObjects-nRobots
            
                        
        return (version, nLinesHeader, nRobots, nFlies)


    # Given a comma separated line of text, return a list of cleaned-up fields.
    def ListFromCsv (self, csv):
        field_list = csv.split(',')
        for i in range(len(field_list)):
            field_list[i] = field_list[i].strip(' \n\"')
            
        return field_list


    def SetDefaultHeader(self):
            self.param_date_time                    = '0'
            self.param_description                  = 'unspecified'
            self.param_maxTrials                    = '0'
            self.param_trial                        = '0'
            
            self.param_nRobots                      = '0'
            self.param_widthRobot                   = '0'
            self.param_heightRobot                  = '0'
            self.param_visibleRobot                 = 'unspecified'
            self.param_paintRobot                   = 'unspecified'
            self.param_scentRobot                   = 'unspecified'
            self.param_descriptionRobot             = 'unspecified'

            self.param_nFlies                       = '0'
            self.param_typeFlies                    = 'unspecified'
            self.param_genderFlies                  = 'unspecified'
            self.param_descriptionFlies             = 'unspecified'
            
            self.param_trackingExclusionzoneEnabled     = 'False'
            self.param_trackingExclusionzoneX_list      = []
            self.param_trackingExclusionzoneY_list      = []
            self.param_trackingExclusionzoneRadius_list = []

            self.param_preRobotEnabled                 = 'False'
            self.param_preRobotMovePatternFramePosition= 'unspecified'
            self.param_preRobotMovePatternFrameAngle   = 'unspecified'
            self.param_preRobotMovePatternShape        = 'unspecified'
            self.param_preRobotMovePatternHzPattern    = '0'
            self.param_preRobotMovePatternHzPoint      = '0'
            self.param_preRobotMovePatternCount        = '0'
            self.param_preRobotMovePatternSizeX        = '0'
            self.param_preRobotMovePatternSizeY        = '0'
            self.param_preRobotMovePatternParam        = '0'
            self.param_preRobotMovePatternDirection    = '0'
            self.param_preRobotMoveRelTracking         = 'unspecified'
            self.param_preRobotMoveRelOriginPosition   = 'unspecified'
            self.param_preRobotMoveRelOriginAngle      = 'unspecified'
            self.param_preRobotMoveRelDistance         = '0'
            self.param_preRobotMoveRelAngleOffset      = '0'
            self.param_preRobotMoveRelAngleType        = 'unspecified'
            self.param_preRobotMoveRelAngleOscMag      = '0'
            self.param_preRobotMoveRelAngleOscFreq     = '0'
            self.param_preRobotMoveRelSpeed            = '0'
            self.param_preRobotMoveRelSpeedType        = 'unspecified'
            self.param_preRobotMoveRelTolerance        = '0'
            self.param_preRobotMoveTimeout             = '-1'
    
            self.param_preLaserEnabled                 = 'False'
            self.param_preLaserPatternFramePosition    = 'unspecified'
            self.param_preLaserPatternFrameAngle       = 'unspecified'
            self.param_preLaserPatternShape            = 'unspecified'
            self.param_preLaserPatternHzPattern        = '0'
            self.param_preLaserPatternHzPoint          = '0'
            self.param_preLaserPatternCount            = '0'
            self.param_preLaserPatternSizeX            = '0'
            self.param_preLaserPatternSizeY            = '0'
            self.param_preLaserPatternParam            = '0'
            self.param_preLaserPatternDirection        = '0'
            self.param_preLaserStatefilterLo           = ''
            self.param_preLaserStatefilterHi           = ''
            self.param_preLaserStatefilterCriteria     = ''
            self.param_preLaserTimeout                 = '-1'
        
            self.param_preLEDPanelsEnabled              = 'False'
            self.param_preLEDPanelsCommand              = 'unspecified'
            self.param_preLEDPanelsIdPattern            = '0'
            self.param_preLEDPanelsFrameid              = 'unspecified'
            self.param_preLEDPanelsStatefilterLo        = ''
            self.param_preLEDPanelsStatefilterHi        = ''
            self.param_preLEDPanelsStatefilterCriteria  = ''
            self.param_preLEDPanelsTimeout              = '-1'

            self.param_preWait1                         = '0.0'
            
            self.param_preTriggerEnabled              = 'False'
            self.param_preTriggerFrameidParent        = 'unspecified'
            self.param_preTriggerFrameidChild         = 'unspecified'
            self.param_preTriggerSpeedAbsParentMin    = '0'
            self.param_preTriggerSpeedAbsParentMax    = '0'
            self.param_preTriggerSpeedAbsChildMin     = '0'
            self.param_preTriggerSpeedAbsChildMax     = '0'
            self.param_preTriggerSpeedRelMin          = '0'
            self.param_preTriggerSpeedRelMax          = '0'
            self.param_preTriggerDistanceMin          = '0'
            self.param_preTriggerDistanceMax          = '0'
            self.param_preTriggerAngleMin             = '0'
            self.param_preTriggerAngleMax             = '0'
            self.param_preTriggerAngleTest            = 'unspecified'
            self.param_preTriggerAngleTestBilateral   = 'unspecified'
            self.param_preTriggerTimeHold             = '0'
            self.param_preTriggerTimeout              = '-1'

            self.param_preWait2                         = '0.0'

            self.param_trialRobotEnabled                = 'False'
            self.param_trialRobotMovePatternFramePosition= 'unspecified'
            self.param_trialRobotMovePatternFrameAngle  = 'unspecified'
            self.param_trialRobotMovePatternShape       = 'unspecified'
            self.param_trialRobotMovePatternHzPattern   = '0'
            self.param_trialRobotMovePatternHzPoint     = '0'
            self.param_trialRobotMovePatternCount       = '0'
            self.param_trialRobotMovePatternSizeX       = '0'
            self.param_trialRobotMovePatternSizeY       = '0'
            self.param_trialRobotMovePatternParam       = '0'
            self.param_trialRobotMovePatternDirection   = '0'
            self.param_trialRobotMoveRelTracking        = 'unspecified'
            self.param_trialRobotMoveRelOriginPosition  = 'unspecified'
            self.param_trialRobotMoveRelOriginAngle     = 'unspecified'
            self.param_trialRobotMoveRelDistance        = '0'
            self.param_trialRobotMoveRelAngleOffset     = '0'
            self.param_trialRobotMoveRelAngleType       = 'unspecified'
            self.param_trialRobotMoveRelAngleOscMag     = '0'
            self.param_trialRobotMoveRelAngleOscFreq    = '0'
            self.param_trialRobotMoveRelSpeed           = '0'
            self.param_trialRobotMoveRelSpeedType       = '0'
            self.param_trialRobotMoveRelTolerance       = '0'
            self.param_trialRobotMoveTimeout            = '-1'
    
            self.param_trialLaserEnabled                = 'False'
            self.param_trialLaserPatternFramePosition   = 'unspecified'
            self.param_trialLaserPatternFrameAngle      = 'unspecified'
            self.param_trialLaserPatternShape           = 'unspecified'
            self.param_trialLaserPatternHzPattern       = '0'
            self.param_trialLaserPatternHzPoint         = '0'
            self.param_trialLaserPatternCount           = '0'
            self.param_trialLaserPatternSizeX           = '0'
            self.param_trialLaserPatternSizeY           = '0'
            self.param_trialLaserPatternParam           = '0'
            self.param_trialLaserPatternDirection       = '0'
            self.param_trialLaserStatefilterLo          = ''
            self.param_trialLaserStatefilterHi          = ''
            self.param_trialLaserStatefilterCriteria    = ''
            self.param_trialLaserTimeout                = '-1'
        
            self.param_trialLEDPanelsEnabled            = 'False'
            self.param_trialLEDPanelsCommand            = 'unspecified'
            self.param_trialLEDPanelsIdPattern          = '0'
            self.param_trialLEDPanelsFrameid            = 'unspecified'
            self.param_trialLEDPanelsStatefilterLo      = ''
            self.param_trialLEDPanelsStatefilterHi      = ''
            self.param_trialLEDPanelsStatefilterCriteria= ''
            self.param_trialLEDPanelsTimeout            = '-1'

            self.param_postTriggerEnabled              = 'False'
            self.param_postTriggerFrameidParent        = 'unspecified'
            self.param_postTriggerFrameidChild         = 'unspecified'
            self.param_postTriggerSpeedAbsParentMin    = '0'
            self.param_postTriggerSpeedAbsParentMax    = '0'
            self.param_postTriggerSpeedAbsChildMin     = '0'
            self.param_postTriggerSpeedAbsChildMax     = '0'
            self.param_postTriggerSpeedRelMin          = '0'
            self.param_postTriggerSpeedRelMax          = '0'
            self.param_postTriggerDistanceMin          = '0'
            self.param_postTriggerDistanceMax          = '0'
            self.param_postTriggerAngleMin             = '0'
            self.param_postTriggerAngleMax             = '0'
            self.param_postTriggerAngleTest            = 'unspecified'
            self.param_postTriggerAngleTestBilateral   = 'unspecified'
            self.param_postTriggerTimeHold             = '0'
            self.param_postTriggerTimeout              = '-1'

            self.param_postWait                         = '0.0'


        
    def ReadHeader_V1 (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        
        with open(filename, 'r') as fid:
            self.AdvanceUntilField(fid, 'date_time')
            headerTxt = fid.readline()
            header = fid.readline()

        #fieldPre_list = header.split(',')
        #field_list = []
        #for field in fieldPre_list:
        #    field_list.append(field.strip(' \n'))
        field_list = self.ListFromCsv(header)    
            

        if versionIn=='1.0': 
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            self.param_preWait1                     = field_list[4]
            
            self.param_preTriggerDistanceMin        = field_list[5]
            self.param_preTriggerDistanceMax        = field_list[6]
            self.param_preTriggerSpeedRelMin        = field_list[7]
            self.param_preTriggerSpeedRelMax        = field_list[8]
            self.param_preTriggerAngleMin           = field_list[9]
            self.param_preTriggerAngleMax           = field_list[10]
            self.param_preTriggerAngleTest          = field_list[11]
            self.param_preTriggerAngleTestBilateral = field_list[12]
            self.param_preTriggerTimeHold           = field_list[13]

            self.param_postTriggerDistanceMin       = field_list[14]
            self.param_postTriggerDistanceMax       = field_list[15]
            self.param_postTriggerSpeedRelMin       = field_list[16]
            self.param_postTriggerSpeedRelMax       = field_list[17]
            self.param_postTriggerAngleMin          = field_list[18]
            self.param_postTriggerAngleMax          = field_list[19]
            self.param_postTriggerAngleTest         = field_list[20]
            self.param_postTriggerAngleTestBilateral = field_list[21]
            self.param_postTriggerTimeHold          = field_list[22]

            self.param_widthRobot                   = field_list[23]
            self.param_heightRobot                  = field_list[24]
            self.param_visibleRobot                 = field_list[25]
            self.param_paintRobot                   = field_list[26]
            self.param_scentRobot                   = field_list[27]
            self.param_descriptionRobot             = self.param_visibleRobot + ';' + self.param_paintRobot + ';' + self.param_scentRobot

            self.param_trialRobotMovePatternShape   = field_list[28]
            self.param_trialRobotMovePatternHzPattern = field_list[29]
            self.param_trialRobotMovePatternHzPoint = field_list[30]
            self.param_trialRobotMovePatternCount   = field_list[31]
            self.param_trialRobotMovePatternSizeX   = field_list[32]
            self.param_trialRobotMovePatternSizeY   = field_list[33]
            self.param_trialRobotMoveRelTracking    = field_list[34]
            self.param_trialRobotMoveRelOriginPosition = field_list[35]
            self.param_trialRobotMoveRelOriginAngle = field_list[36]
            self.param_trialRobotMoveRelDistance    = field_list[37]
            self.param_trialRobotMoveRelAngleOffset = field_list[38]
            self.param_trialRobotMoveRelAngleType   = field_list[39]
            self.param_trialRobotMoveRelSpeed       = field_list[40]
            self.param_trialRobotMoveRelSpeedType   = field_list[41]
            self.param_trialRobotMoveRelTolerance   = field_list[42]
    
        


        if versionIn=='1.1': 
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            self.param_widthRobot                   = field_list[4]
            self.param_heightRobot                  = field_list[5]
            self.param_visibleRobot                 = field_list[6]
            self.param_paintRobot                   = field_list[7]
            self.param_scentRobot                   = field_list[8]
            self.param_descriptionRobot             = self.param_visibleRobot + ';' + self.param_paintRobot + ';' + self.param_scentRobot

            self.param_preWait1                     = field_list[9]
            
            self.param_preTriggerFrameidParent      = field_list[10]
            self.param_preTriggerFrameidChild       = field_list[11]
            self.param_preTriggerSpeedAbsParentMin  = field_list[12]
            self.param_preTriggerSpeedAbsParentMax  = field_list[13]
            self.param_preTriggerSpeedAbsChildMin   = field_list[14]
            self.param_preTriggerSpeedAbsChildMax   = field_list[15]
            self.param_preTriggerDistanceMin        = field_list[16]
            self.param_preTriggerDistanceMax        = field_list[17]
            self.param_preTriggerAngleMin           = field_list[18]
            self.param_preTriggerAngleMax           = field_list[19]
            self.param_preTriggerAngleTest          = field_list[20]
            self.param_preTriggerAngleTestBilateral = field_list[21]
            self.param_preTriggerTimeHold           = field_list[22]

            self.param_trialRobotMovePatternShape        = field_list[23]
            self.param_trialRobotMovePatternHzPattern    = field_list[24]
            self.param_trialRobotMovePatternHzPoint      = field_list[25]
            self.param_trialRobotMovePatternCount        = field_list[26]
            self.param_trialRobotMovePatternSizeX        = field_list[27]
            self.param_trialRobotMovePatternSizeY        = field_list[28]
            self.param_trialRobotMoveRelTracking         = field_list[29]
            self.param_trialRobotMoveRelOriginPosition   = field_list[30]
            self.param_trialRobotMoveRelOriginAngle      = field_list[31]
            self.param_trialRobotMoveRelDistance         = field_list[32]
            self.param_trialRobotMoveRelAngleOffset      = field_list[33]
            self.param_trialRobotMoveRelAngleType        = field_list[34]
            self.param_trialRobotMoveRelSpeed            = field_list[35]
            self.param_trialRobotMoveRelSpeedType        = field_list[36]
            self.param_trialRobotMoveRelTolerance        = field_list[37]
    
            self.param_trialLaserEnabled                 = field_list[38]

            self.param_postTriggerFrameidParent        = field_list[39]
            self.param_postTriggerFrameidChild         = field_list[40]
            self.param_postTriggerSpeedAbsParentMin    = field_list[41]
            self.param_postTriggerSpeedAbsParentMax    = field_list[42]
            self.param_postTriggerSpeedAbsChildMin     = field_list[43]
            self.param_postTriggerSpeedAbsChildMax     = field_list[44]
            self.param_postTriggerDistanceMin          = field_list[45]
            self.param_postTriggerDistanceMax          = field_list[46]
            self.param_postTriggerAngleMin             = field_list[47]
            self.param_postTriggerAngleMax             = field_list[48]
            self.param_postTriggerAngleTest            = field_list[49]
            self.param_postTriggerAngleTestBilateral   = field_list[50]
            self.param_postTriggerTimeHold             = field_list[51]


    def ReadHeader_V2 (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        
        with open(filename, 'r') as fid:
            # Read all the lines in the header.
            line = []
            for i in range(nLinesHeader):
                line.append(fid.readline())

            
            if versionIn == '2.0':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.1':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.2':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait1Txt   = line[i]; i=i+1
                headerPreWait1      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreTriggerTxt = line[i]; i=i+1
                headerPreTrigger    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialRobotTxt = line[i]; i=i+1
                headerTrialRobot    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt = line[i]; i=i+1
                headerTrialLaser    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt= line[i]; i=i+1
                headerPostTrigger   = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.3':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait1Txt   = line[i]; i=i+1
                headerPreWait1      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreTriggerTxt = line[i]; i=i+1
                headerPreTrigger    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialRobotTxt = line[i]; i=i+1
                headerTrialRobot    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt = line[i]; i=i+1
                headerTrialLaser    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt= line[i]; i=i+1
                headerPostTrigger   = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLEDPanelsTxt = line[i]; i=i+1
                headerTrialLEDPanels    = line[i]; i=i+1
                blank                   = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.4':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait1Txt   = line[i]; i=i+1
                headerPreWait1      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreTriggerTxt = line[i]; i=i+1
                headerPreTrigger    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait2Txt   = line[i]; i=i+1
                headerPreWait2      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialRobotTxt = line[i]; i=i+1
                headerTrialRobot    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt = line[i]; i=i+1
                headerTrialLaser    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt = line[i]; i=i+1
                headerPostTrigger   = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLEDPanelsTxt  = line[i]; i=i+1
                headerTrialLEDPanels     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.5':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait1Txt   = line[i]; i=i+1
                headerPreWait1      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreTriggerTxt = line[i]; i=i+1
                headerPreTrigger    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait2Txt   = line[i]; i=i+1
                headerPreWait2      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialRobotTxt = line[i]; i=i+1
                headerTrialRobot    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt = line[i]; i=i+1
                headerTrialLaser    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLEDPanelsTxt  = line[i]; i=i+1
                headerTrialLEDPanels     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt = line[i]; i=i+1
                headerPostTrigger   = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if (versionIn in ['2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
                i = 0;
                headerVersionTxt    = line[i]; i=i+1
                headerVersion       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotSpecTxt  = line[i]; i=i+1
                headerRobotSpec     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFlySpecTxt    = line[i]; i=i+1
                headerFlySpec       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrackingTxt   = line[i]; i=i+1
                headerTracking      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreRobotTxt   = line[i]; i=i+1
                headerPreRobot      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreLaserTxt   = line[i]; i=i+1
                headerPreLaser      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreLEDPanelsTxt = line[i]; i=i+1
                headerPreLEDPanels  = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait1Txt   = line[i]; i=i+1
                headerPreWait1      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreTriggerTxt = line[i]; i=i+1
                headerPreTrigger    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPreWait2Txt   = line[i]; i=i+1
                headerPreWait2      = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialRobotTxt = line[i]; i=i+1
                headerTrialRobot    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt = line[i]; i=i+1
                headerTrialLaser    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLEDPanelsTxt  = line[i]; i=i+1
                headerTrialLEDPanels     = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt = line[i]; i=i+1
                headerPostTrigger   = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


        ##################################################################
        # Put the fields from each line into their respective variables.
        ##################################################################

        ### Experiment Setup
        if (versionIn in ['2.0', '2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            
        ### Robot spec
        if (versionIn in ['2.0', '2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82']):
            field_list = self.ListFromCsv(headerRobotSpec)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]
            self.param_descriptionRobot             = self.param_visibleRobot + ';' + self.param_paintRobot + ';' + self.param_scentRobot
        if (versionIn in ['2.83']):
            field_list = self.ListFromCsv(headerRobotSpec)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_descriptionRobot             = field_list[3]

        
        ### Flies
        if (versionIn in ['2.0']):
            field_list = self.ListFromCsv(headerFlySpec)
            self.param_nFlies                       = field_list[0]
        if (versionIn in ['2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82']):
            field_list = self.ListFromCsv(headerFlySpec)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            self.param_descriptionFlies             = self.param_typeFlies + ';' + self.param_genderFlies
        if (versionIn in ['2.83']):
            field_list = self.ListFromCsv(headerFlySpec)
            self.param_nFlies                       = field_list[0]
            self.param_descriptionFlies             = field_list[1]
            

        ### Exclusion Zone.
        if (versionIn in ['2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerTracking)
            self.param_trackingExclusionzoneEnabled     = field_list[0]
            if len(field_list)>1:
                self.param_trackingExclusionzoneX_list      = [field_list[1],] # BUG: X_list is actually fields 1,4,7,etc
                self.param_trackingExclusionzoneY_list      = [field_list[2],] # BUG: X_list is actually fields 2,5,8,etc
                self.param_trackingExclusionzoneRadius_list = [field_list[3],] # BUG: X_list is actually fields 3,6,9,etc
            else:
                self.param_trackingExclusionzoneX_list      = []
                self.param_trackingExclusionzoneY_list      = []
                self.param_trackingExclusionzoneRadius_list = []

        
        ### Pre Wait1
        if (versionIn in ['2.0', '2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]

        
        ### Pre Robot Move
        if (versionIn in ['2.6', '2.7']):
            field_list = self.ListFromCsv(headerPreRobot)
            self.param_preRobotEnabled             = field_list[0]
            self.param_preRobotMovePatternShape        = field_list[1]
            self.param_preRobotMovePatternHzPattern    = field_list[2]
            self.param_preRobotMovePatternHzPoint      = field_list[3]
            self.param_preRobotMovePatternCount        = field_list[4]
            self.param_preRobotMovePatternSizeX        = field_list[5]
            self.param_preRobotMovePatternSizeY        = field_list[6]
            self.param_preRobotMovePatternParam        = field_list[7]
            self.param_preRobotMoveRelTracking         = field_list[8]
            self.param_preRobotMoveRelOriginPosition   = field_list[9]
            self.param_preRobotMoveRelOriginAngle      = field_list[10]
            self.param_preRobotMoveRelDistance         = field_list[11]
            self.param_preRobotMoveRelAngleOffset      = field_list[12]
            self.param_preRobotMoveRelAngleType        = field_list[13]
            self.param_preRobotMoveRelSpeed            = field_list[14]
            self.param_preRobotMoveRelSpeedType        = field_list[15]
            self.param_preRobotMoveRelTolerance        = field_list[16]
            self.param_preRobotMoveTimeout             = field_list[17]
        if (versionIn in ['2.8']):
            field_list = self.ListFromCsv(headerPreRobot)
            self.param_preRobotEnabled                 = field_list[0]
            self.param_preRobotMovePatternFramePosition= field_list[1]
            self.param_preRobotMovePatternFrameAngle   = field_list[2]
            self.param_preRobotMovePatternShape        = field_list[3]
            self.param_preRobotMovePatternHzPattern    = field_list[4]
            self.param_preRobotMovePatternHzPoint      = field_list[5]
            self.param_preRobotMovePatternCount        = field_list[6]
            self.param_preRobotMovePatternSizeX        = field_list[7]
            self.param_preRobotMovePatternSizeY        = field_list[8]
            self.param_preRobotMovePatternParam        = field_list[9]
            self.param_preRobotMovePatternDirection    = field_list[10]
            self.param_preRobotMoveRelTracking         = field_list[11]
            self.param_preRobotMoveRelOriginPosition   = field_list[12]
            self.param_preRobotMoveRelOriginAngle      = field_list[13]
            self.param_preRobotMoveRelDistance         = field_list[14]
            self.param_preRobotMoveRelAngleOffset      = field_list[15]
            self.param_preRobotMoveRelAngleType        = field_list[16]
            self.param_preRobotMoveRelSpeed            = field_list[17]
            self.param_preRobotMoveRelSpeedType        = field_list[18]
            self.param_preRobotMoveRelTolerance        = field_list[19]
            self.param_preRobotMoveTimeout             = field_list[20]
        if (versionIn in ['2.81']):
            field_list = self.ListFromCsv(headerPreRobot)
            self.param_preRobotEnabled                 = field_list[0]
            self.param_preRobotMovePatternFramePosition= field_list[1]
            self.param_preRobotMovePatternFrameAngle   = field_list[2]
            self.param_preRobotMovePatternShape        = field_list[3]
            self.param_preRobotMovePatternHzPattern    = field_list[4]
            self.param_preRobotMovePatternHzPoint      = field_list[5]
            self.param_preRobotMovePatternCount        = field_list[6]
            self.param_preRobotMovePatternSizeX        = field_list[7]
            self.param_preRobotMovePatternSizeY        = field_list[8]
            self.param_preRobotMovePatternParam        = field_list[9]
            self.param_preRobotMovePatternDirection    = field_list[10]
            self.param_preRobotMoveRelTracking         = field_list[11]
            self.param_preRobotMoveRelOriginPosition   = field_list[12]
            self.param_preRobotMoveRelOriginAngle      = field_list[13]
            self.param_preRobotMoveRelDistance         = field_list[14]
            self.param_preRobotMoveRelAngleOffset      = field_list[15]
            self.param_preRobotMoveRelAngleType        = field_list[16]
            self.param_preRobotMoveRelSpeed            = field_list[17]
            self.param_preRobotMoveRelSpeedType        = field_list[18]
            self.param_preRobotMoveRelTolerance        = field_list[19]
        if (versionIn in ['2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreRobot)
            self.param_preRobotEnabled                 = field_list[0]
            self.param_preRobotMovePatternFramePosition= field_list[1]
            self.param_preRobotMovePatternFrameAngle   = field_list[2]
            self.param_preRobotMovePatternShape        = field_list[3]
            self.param_preRobotMovePatternHzPattern    = field_list[4]
            self.param_preRobotMovePatternHzPoint      = field_list[5]
            self.param_preRobotMovePatternCount        = field_list[6]
            self.param_preRobotMovePatternSizeX        = field_list[7]
            self.param_preRobotMovePatternSizeY        = field_list[8]
            self.param_preRobotMovePatternParam        = field_list[9]
            self.param_preRobotMovePatternDirection    = field_list[10]
            self.param_preRobotMoveRelTracking         = field_list[11]
            self.param_preRobotMoveRelOriginPosition   = field_list[12]
            self.param_preRobotMoveRelOriginAngle      = field_list[13]
            self.param_preRobotMoveRelDistance         = field_list[14]
            self.param_preRobotMoveRelAngleOffset      = field_list[15]
            self.param_preRobotMoveRelAngleType        = field_list[16]
            self.param_preRobotMoveRelAngleOscMag      = field_list[17]
            self.param_preRobotMoveRelAngleOscFreq     = field_list[18]
            self.param_preRobotMoveRelSpeed            = field_list[19]
            self.param_preRobotMoveRelSpeedType        = field_list[20]
            self.param_preRobotMoveRelTolerance        = field_list[21]
    
        
        ### Pre LEDPanels.
        if (versionIn in ['2.6', '2.7', '2.8']):
            field_list = self.ListFromCsv(headerPreLEDPanels)
            self.param_preLEDPanelsEnabled              = field_list[0]
            self.param_preLEDPanelsCommand              = field_list[1]
            self.param_preLEDPanelsIdPattern            = field_list[2]
            self.param_preLEDPanelsFrameid              = field_list[3]
            self.param_preLEDPanelsStatefilterLo        = field_list[4]
            self.param_preLEDPanelsStatefilterHi        = field_list[5]
            self.param_preLEDPanelsStatefilterCriteria  = field_list[6]
            self.param_preLEDPanelsTimeout              = field_list[7]
        if (versionIn in ['2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreLEDPanels)
            self.param_preLEDPanelsEnabled              = field_list[0]
            self.param_preLEDPanelsCommand              = field_list[1]
            self.param_preLEDPanelsIdPattern            = field_list[2]
            self.param_preLEDPanelsFrameid              = field_list[3]
            self.param_preLEDPanelsStatefilterLo        = field_list[4]
            self.param_preLEDPanelsStatefilterHi        = field_list[5]
            self.param_preLEDPanelsStatefilterCriteria  = field_list[6]
            
            
        ### Pre Laser.
        if (versionIn in ['2.6', '2.7']):
            field_list = self.ListFromCsv(headerPreLaser)
            self.param_preLaserEnabled                 = field_list[0]
            self.param_preLaserPatternShape            = field_list[1]
            self.param_preLaserPatternHzPattern        = field_list[2]
            self.param_preLaserPatternHzPoint          = field_list[3]
            self.param_preLaserPatternCount            = field_list[4]
            self.param_preLaserPatternSizeX            = field_list[5]
            self.param_preLaserPatternSizeY            = field_list[6]
            self.param_preLaserPatternParam            = field_list[7]
            self.param_preLaserStatefilterLo           = field_list[8]
            self.param_preLaserStatefilterHi           = field_list[9]
            self.param_preLaserStatefilterCriteria     = field_list[10]
            self.param_preLaserTimeout                 = field_list[11]
        if (versionIn in ['2.8']):
            field_list = self.ListFromCsv(headerPreLaser)
            self.param_preLaserEnabled                 = field_list[0]
            self.param_preLaserPatternFramePosition    = field_list[1]
            self.param_preLaserPatternFrameAngle       = field_list[2]
            self.param_preLaserPatternShape            = field_list[3]
            self.param_preLaserPatternHzPattern        = field_list[4]
            self.param_preLaserPatternHzPoint          = field_list[5]
            self.param_preLaserPatternCount            = field_list[6]
            self.param_preLaserPatternSizeX            = field_list[7]
            self.param_preLaserPatternSizeY            = field_list[8]
            self.param_preLaserPatternParam            = field_list[9]
            self.param_preLaserPatternDirection        = field_list[10]
            self.param_preLaserStatefilterLo           = field_list[11]
            self.param_preLaserStatefilterHi           = field_list[12]
            self.param_preLaserStatefilterCriteria     = field_list[13]
            self.param_preLaserTimeout                 = field_list[14]
        if (versionIn in ['2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreLaser)
            self.param_preLaserEnabled                 = field_list[0]
            self.param_preLaserPatternFramePosition    = field_list[1]
            self.param_preLaserPatternFrameAngle       = field_list[2]
            self.param_preLaserPatternShape            = field_list[3]
            self.param_preLaserPatternHzPattern        = field_list[4]
            self.param_preLaserPatternHzPoint          = field_list[5]
            self.param_preLaserPatternCount            = field_list[6]
            self.param_preLaserPatternSizeX            = field_list[7]
            self.param_preLaserPatternSizeY            = field_list[8]
            self.param_preLaserPatternParam            = field_list[9]
            self.param_preLaserPatternDirection        = field_list[10]
            self.param_preLaserStatefilterLo           = field_list[11]
            self.param_preLaserStatefilterHi           = field_list[12]
            self.param_preLaserStatefilterCriteria     = field_list[13]

        
        ### Pre Triggering.
        if (versionIn in ['2.0', '2.1']):
            field_list = self.ListFromCsv(headerPreTrigger)
            self.param_preTriggerEnabled              = field_list[0]
            self.param_preTriggerFrameidParent        = field_list[1]
            self.param_preTriggerFrameidChild         = field_list[2]
            self.param_preTriggerSpeedAbsParentMin    = field_list[3]
            self.param_preTriggerSpeedAbsParentMax    = field_list[4]
            self.param_preTriggerSpeedAbsChildMin     = field_list[5]
            self.param_preTriggerSpeedAbsChildMax     = field_list[6]
            self.param_preTriggerDistanceMin          = field_list[7]
            self.param_preTriggerDistanceMax          = field_list[8]
            self.param_preTriggerAngleMin             = field_list[9]
            self.param_preTriggerAngleMax             = field_list[10]
            self.param_preTriggerAngleTest            = field_list[11]
            self.param_preTriggerAngleTestBilateral   = field_list[12]
            self.param_preTriggerTimeHold             = field_list[13]
            self.param_preTriggerTimeout              = field_list[14]
        if (versionIn in ['2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreTrigger)
            self.param_preTriggerEnabled              = field_list[0]
            self.param_preTriggerFrameidParent        = field_list[1]
            self.param_preTriggerFrameidChild         = field_list[2]
            self.param_preTriggerSpeedAbsParentMin    = field_list[3]
            self.param_preTriggerSpeedAbsParentMax    = field_list[4]
            self.param_preTriggerSpeedAbsChildMin     = field_list[5]
            self.param_preTriggerSpeedAbsChildMax     = field_list[6]
            self.param_preTriggerSpeedRelMin          = field_list[7]
            self.param_preTriggerSpeedRelMax          = field_list[8]
            self.param_preTriggerDistanceMin          = field_list[9]
            self.param_preTriggerDistanceMax          = field_list[10]
            self.param_preTriggerAngleMin             = field_list[11]
            self.param_preTriggerAngleMax             = field_list[12]
            self.param_preTriggerAngleTest            = field_list[13]
            self.param_preTriggerAngleTestBilateral   = field_list[14]
            self.param_preTriggerTimeHold             = field_list[15]
            self.param_preTriggerTimeout              = field_list[16]


        ### Pre Wait2.
        if (versionIn in ['2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPreWait2)
            self.param_preWait2                         = field_list[0]

        
        ### Trial Robot Move.
        if (versionIn in ['2.0']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternShape        = field_list[1]
            self.param_trialRobotMovePatternHzPattern    = field_list[2]
            self.param_trialRobotMovePatternHzPoint      = field_list[3]
            self.param_trialRobotMovePatternCount        = field_list[4]
            self.param_trialRobotMovePatternSizeX        = field_list[5]
            self.param_trialRobotMovePatternSizeY        = field_list[6]
            self.param_trialRobotMoveRelTracking         = field_list[7]
            self.param_trialRobotMoveRelOriginPosition   = field_list[8]
            self.param_trialRobotMoveRelOriginAngle      = field_list[9]
            self.param_trialRobotMoveRelDistance         = field_list[10]
            self.param_trialRobotMoveRelAngleOffset      = field_list[11]
            self.param_trialRobotMoveRelAngleType        = field_list[12]
            self.param_trialRobotMoveRelSpeed            = field_list[13]
            self.param_trialRobotMoveRelSpeedType        = field_list[14]
            self.param_trialRobotMoveRelTolerance        = field_list[15]
        if (versionIn in ['2.1', '2.2']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternShape        = field_list[1]
            self.param_trialRobotMovePatternHzPattern    = field_list[2]
            self.param_trialRobotMovePatternHzPoint      = field_list[3]
            self.param_trialRobotMovePatternCount        = field_list[4]
            self.param_trialRobotMovePatternSizeX        = field_list[5]
            self.param_trialRobotMovePatternSizeY        = field_list[6]
            self.param_trialRobotMovePatternParam        = field_list[7]
            self.param_trialRobotMoveRelTracking         = field_list[8]
            self.param_trialRobotMoveRelOriginPosition   = field_list[9]
            self.param_trialRobotMoveRelOriginAngle      = field_list[10]
            self.param_trialRobotMoveRelDistance         = field_list[11]
            self.param_trialRobotMoveRelAngleOffset      = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
        if (versionIn in ['2.3', '2.4', '2.5', '2.6', '2.7']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternShape        = field_list[1]
            self.param_trialRobotMovePatternHzPattern    = field_list[2]
            self.param_trialRobotMovePatternHzPoint      = field_list[3]
            self.param_trialRobotMovePatternCount        = field_list[4]
            self.param_trialRobotMovePatternSizeX        = field_list[5]
            self.param_trialRobotMovePatternSizeY        = field_list[6]
            self.param_trialRobotMovePatternParam        = field_list[7]
            self.param_trialRobotMoveRelTracking         = field_list[8]
            self.param_trialRobotMoveRelOriginPosition   = field_list[9]
            self.param_trialRobotMoveRelOriginAngle      = field_list[10]
            self.param_trialRobotMoveRelDistance         = field_list[11]
            self.param_trialRobotMoveRelAngleOffset      = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
            self.param_trialRobotMoveTimeout             = field_list[17]
        if (versionIn in ['2.8']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternFramePosition= field_list[1]
            self.param_trialRobotMovePatternFrameAngle   = field_list[2]
            self.param_trialRobotMovePatternShape        = field_list[3]
            self.param_trialRobotMovePatternHzPattern    = field_list[4]
            self.param_trialRobotMovePatternHzPoint      = field_list[5]
            self.param_trialRobotMovePatternCount        = field_list[6]
            self.param_trialRobotMovePatternSizeX        = field_list[7]
            self.param_trialRobotMovePatternSizeY        = field_list[8]
            self.param_trialRobotMovePatternParam        = field_list[9]
            self.param_trialRobotMovePatternDirection    = field_list[10]
            self.param_trialRobotMoveRelTracking         = field_list[11]
            self.param_trialRobotMoveRelOriginPosition   = field_list[12]
            self.param_trialRobotMoveRelOriginAngle      = field_list[13]
            self.param_trialRobotMoveRelDistance         = field_list[14]
            self.param_trialRobotMoveRelAngleOffset      = field_list[15]
            self.param_trialRobotMoveRelAngleType        = field_list[16]
            self.param_trialRobotMoveRelSpeed            = field_list[17]
            self.param_trialRobotMoveRelSpeedType        = field_list[18]
            self.param_trialRobotMoveRelTolerance        = field_list[19]
            self.param_trialRobotMoveTimeout             = field_list[20]
        if (versionIn in ['2.81']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternFramePosition= field_list[1]
            self.param_trialRobotMovePatternFrameAngle   = field_list[2]
            self.param_trialRobotMovePatternShape        = field_list[3]
            self.param_trialRobotMovePatternHzPattern    = field_list[4]
            self.param_trialRobotMovePatternHzPoint      = field_list[5]
            self.param_trialRobotMovePatternCount        = field_list[6]
            self.param_trialRobotMovePatternSizeX        = field_list[7]
            self.param_trialRobotMovePatternSizeY        = field_list[8]
            self.param_trialRobotMovePatternParam        = field_list[9]
            self.param_trialRobotMovePatternDirection    = field_list[10]
            self.param_trialRobotMoveRelTracking         = field_list[11]
            self.param_trialRobotMoveRelOriginPosition   = field_list[12]
            self.param_trialRobotMoveRelOriginAngle      = field_list[13]
            self.param_trialRobotMoveRelDistance         = field_list[14]
            self.param_trialRobotMoveRelAngleOffset      = field_list[15]
            self.param_trialRobotMoveRelAngleType        = field_list[16]
            self.param_trialRobotMoveRelSpeed            = field_list[17]
            self.param_trialRobotMoveRelSpeedType        = field_list[18]
            self.param_trialRobotMoveRelTolerance        = field_list[19]
        if (versionIn in ['2.82', '2.83']):
            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled                 = field_list[0]
            self.param_trialRobotMovePatternFramePosition= field_list[1]
            self.param_trialRobotMovePatternFrameAngle   = field_list[2]
            self.param_trialRobotMovePatternShape        = field_list[3]
            self.param_trialRobotMovePatternHzPattern    = field_list[4]
            self.param_trialRobotMovePatternHzPoint      = field_list[5]
            self.param_trialRobotMovePatternCount        = field_list[6]
            self.param_trialRobotMovePatternSizeX        = field_list[7]
            self.param_trialRobotMovePatternSizeY        = field_list[8]
            self.param_trialRobotMovePatternParam        = field_list[9]
            self.param_trialRobotMovePatternDirection    = field_list[10]
            self.param_trialRobotMoveRelTracking         = field_list[11]
            self.param_trialRobotMoveRelOriginPosition   = field_list[12]
            self.param_trialRobotMoveRelOriginAngle      = field_list[13]
            self.param_trialRobotMoveRelDistance         = field_list[14]
            self.param_trialRobotMoveRelAngleOffset      = field_list[15]
            self.param_trialRobotMoveRelAngleType        = field_list[16]
            self.param_trialRobotMoveRelAngleOscMag      = field_list[17]
            self.param_trialRobotMoveRelAngleOscFreq     = field_list[18]
            self.param_trialRobotMoveRelSpeed            = field_list[19]
            self.param_trialRobotMoveRelSpeedType        = field_list[20]
            self.param_trialRobotMoveRelTolerance        = field_list[21]
    
        
        ### Trial LEDPanels.
        if (versionIn in ['2.3', '2.4', '2.5', '2.6', '2.7', '2.8']):
            field_list = self.ListFromCsv(headerTrialLEDPanels)
            self.param_trialLEDPanelsEnabled              = field_list[0]
            self.param_trialLEDPanelsCommand              = field_list[1]
            self.param_trialLEDPanelsIdPattern            = field_list[2]
            self.param_trialLEDPanelsFrameid              = field_list[3]
            self.param_trialLEDPanelsStatefilterLo        = field_list[4]
            self.param_trialLEDPanelsStatefilterHi        = field_list[5]
            self.param_trialLEDPanelsStatefilterCriteria  = field_list[6]
            self.param_trialLEDPanelsTimeout              = field_list[7]
        if (versionIn in ['2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerTrialLEDPanels)
            self.param_trialLEDPanelsEnabled              = field_list[0]
            self.param_trialLEDPanelsCommand              = field_list[1]
            self.param_trialLEDPanelsIdPattern            = field_list[2]
            self.param_trialLEDPanelsFrameid              = field_list[3]
            self.param_trialLEDPanelsStatefilterLo        = field_list[4]
            self.param_trialLEDPanelsStatefilterHi        = field_list[5]
            self.param_trialLEDPanelsStatefilterCriteria  = field_list[6]

    
        ### Trial Laser.
        if (versionIn in ['2.0']):
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
        if (versionIn in ['2.1', '2.2']):
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
            self.param_trialLaserPatternShape            = field_list[1]
            self.param_trialLaserPatternHzPattern        = field_list[2]
            self.param_trialLaserPatternHzPoint          = field_list[3]
            self.param_trialLaserPatternCount            = field_list[4]
            self.param_trialLaserPatternSizeX            = field_list[5]
            self.param_trialLaserPatternSizeY            = field_list[6]
            self.param_trialLaserPatternParam            = field_list[7]
            self.param_trialLaserStatefilterLo           = field_list[8]
            self.param_trialLaserStatefilterHi           = field_list[9]
            self.param_trialLaserStatefilterCriteria     = field_list[10]
        if (versionIn in ['2.3', '2.4', '2.5', '2.6', '2.7']):
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
            self.param_trialLaserPatternShape            = field_list[1]
            self.param_trialLaserPatternHzPattern        = field_list[2]
            self.param_trialLaserPatternHzPoint          = field_list[3]
            self.param_trialLaserPatternCount            = field_list[4]
            self.param_trialLaserPatternSizeX            = field_list[5]
            self.param_trialLaserPatternSizeY            = field_list[6]
            self.param_trialLaserPatternParam            = field_list[7]
            self.param_trialLaserStatefilterLo           = field_list[8]
            self.param_trialLaserStatefilterHi           = field_list[9]
            self.param_trialLaserStatefilterCriteria     = field_list[10]
            self.param_trialLaserTimeout                 = field_list[11]
        if (versionIn in ['2.8']):
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
            self.param_trialLaserPatternFramePosition    = field_list[1]
            self.param_trialLaserPatternFrameAngle       = field_list[2]
            self.param_trialLaserPatternShape            = field_list[3]
            self.param_trialLaserPatternHzPattern        = field_list[4]
            self.param_trialLaserPatternHzPoint          = field_list[5]
            self.param_trialLaserPatternCount            = field_list[6]
            self.param_trialLaserPatternSizeX            = field_list[7]
            self.param_trialLaserPatternSizeY            = field_list[8]
            self.param_trialLaserPatternParam            = field_list[9]
            self.param_trialLaserPatternDirection        = field_list[10]
            self.param_trialLaserStatefilterLo           = field_list[11]
            self.param_trialLaserStatefilterHi           = field_list[12]
            self.param_trialLaserStatefilterCriteria     = field_list[13]
            self.param_trialLaserTimeout                 = field_list[14]
        if (versionIn in ['2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
            self.param_trialLaserPatternFramePosition    = field_list[1]
            self.param_trialLaserPatternFrameAngle       = field_list[2]
            self.param_trialLaserPatternShape            = field_list[3]
            self.param_trialLaserPatternHzPattern        = field_list[4]
            self.param_trialLaserPatternHzPoint          = field_list[5]
            self.param_trialLaserPatternCount            = field_list[6]
            self.param_trialLaserPatternSizeX            = field_list[7]
            self.param_trialLaserPatternSizeY            = field_list[8]
            self.param_trialLaserPatternParam            = field_list[9]
            self.param_trialLaserPatternDirection        = field_list[10]
            self.param_trialLaserStatefilterLo           = field_list[11]
            self.param_trialLaserStatefilterHi           = field_list[12]
            self.param_trialLaserStatefilterCriteria     = field_list[13]

        
        ### Post Triggering.
        if (versionIn in ['2.0', '2.1']):
            field_list = self.ListFromCsv(headerPostTrigger)
            self.param_postTriggerEnabled              = field_list[0]
            self.param_postTriggerFrameidParent        = field_list[1]
            self.param_postTriggerFrameidChild         = field_list[2]
            self.param_postTriggerSpeedAbsParentMin    = field_list[3]
            self.param_postTriggerSpeedAbsParentMax    = field_list[4]
            self.param_postTriggerSpeedAbsChildMin     = field_list[5]
            self.param_postTriggerSpeedAbsChildMax     = field_list[6]
            self.param_postTriggerDistanceMin          = field_list[7]
            self.param_postTriggerDistanceMax          = field_list[8]
            self.param_postTriggerAngleMin             = field_list[9]
            self.param_postTriggerAngleMax             = field_list[10]
            self.param_postTriggerAngleTest            = field_list[11]
            self.param_postTriggerAngleTestBilateral   = field_list[12]
            self.param_postTriggerTimeHold             = field_list[13]
            self.param_postTriggerTimeout              = field_list[14]
        if (versionIn in ['2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPostTrigger)
            self.param_postTriggerEnabled              = field_list[0]
            self.param_postTriggerFrameidParent        = field_list[1]
            self.param_postTriggerFrameidChild         = field_list[2]
            self.param_postTriggerSpeedAbsParentMin    = field_list[3]
            self.param_postTriggerSpeedAbsParentMax    = field_list[4]
            self.param_postTriggerSpeedAbsChildMin     = field_list[5]
            self.param_postTriggerSpeedAbsChildMax     = field_list[6]
            self.param_postTriggerSpeedRelMin          = field_list[7]
            self.param_postTriggerSpeedRelMax          = field_list[8]
            self.param_postTriggerDistanceMin          = field_list[9]
            self.param_postTriggerDistanceMax          = field_list[10]
            self.param_postTriggerAngleMin             = field_list[11]
            self.param_postTriggerAngleMax             = field_list[12]
            self.param_postTriggerAngleTest            = field_list[13]
            self.param_postTriggerAngleTestBilateral   = field_list[14]
            self.param_postTriggerTimeHold             = field_list[15]
            self.param_postTriggerTimeout              = field_list[16]


        ### Post Wait.
        if (versionIn in ['2.0', '2.1', '2.2', '2.3', '2.4', '2.5', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83']):
            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


    # ReadHeader()
    # Given a file of any version, load all the header fields, using defaults if necessary.
    #        
    def ReadHeader (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        fVersion = float(versionIn)
        self.SetDefaultHeader()
        if fVersion < 2.0:
            self.ReadHeader_V1(filename)
        elif fVersion >= 2.0 and fVersion <3.0:
            self.ReadHeader_V2(filename)
        elif fVersion >= 3.0 and fVersion <4.0:
            self.ReadHeader_V3(filename)
            
            
        


    def WriteHeader_V22(self, filename):
        headerExperiment_V22 = self.templateExperiment_V22.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V22 = self.templateRobotSpec_V22.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        headerFlySpec_V22 = self.templateFlySpec_V22.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headerTrackingTxt_V22 = self.headerTrackingTxtA_V22
        headerTracking_V22 = self.templateTrackingA_V22.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V22 += self.headerTrackingTxtB_V22
            headerTracking_V22 += self.templateTrackingB_V22.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V22 += '\n'
        headerTracking_V22 += '\n'

        #######################################################################
        headerPreWait1_V22 = self.templatePreWait1_V22.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V22 = self.templatePreTrigger_V22.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        #######################################################################
        headerTrialRobot_V22 = self.templateTrialRobot_V22.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        headerTrialLaser_V22 = self.templateTrialLaser_V22.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                )
            
        #######################################################################
        headerPostTrigger_V22 = self.templatePostTrigger_V22.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V22 = self.templatePostWait_V22.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerExperimentTxt_V22)
            fid.write(headerExperiment_V22)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V22)
            fid.write(headerRobotSpec_V22)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V22)
            fid.write(headerFlySpec_V22)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V22)
            fid.write(headerTracking_V22)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V22)
            fid.write(headerPreWait1_V22)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V22)
            fid.write(headerPreTrigger_V22)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V22)
            fid.write(headerTrialRobot_V22)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V22)
            fid.write(headerTrialLaser_V22)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V22)
            fid.write(headerPostTrigger_V22)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V22)
            fid.write(headerPostWait_V22)
            fid.write('\n')
            
            fid.write('\n')
            fid.write('\n')
            fid.write('\n')

            fid.write('\n')
            fid.write('\n')
            fid.write('\n')


    def WriteHeader_V26(self, filename):
        headerVersionFile_V26 = self.templateVersionFile_V26.format(
                                                versionFile              = '2.6',
                                                ) 
        headerExperiment_V26 = self.templateExperiment_V26.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V26 = self.templateRobotSpec_V26.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        headerFlySpec_V26 = self.templateFlySpec_V26.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headerTrackingTxt_V26 = self.headerTrackingTxtA_V26
        headerTracking_V26 = self.templateTrackingA_V26.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V26 += self.headerTrackingTxtB_V26
            headerTracking_V26 += self.templateTrackingB_V26.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V26 += '\n'
        headerTracking_V26 += '\n'

        #######################################################################
        headerPreRobot_V26 = self.templatePreRobot_V26.format(
                                                preRobotEnabled               = self.param_preRobotEnabled,
                                                preRobotMovePatternShape      = self.param_preRobotMovePatternShape,
                                                preRobotMovePatternHzPattern  = self.param_preRobotMovePatternHzPattern,
                                                preRobotMovePatternHzPoint    = self.param_preRobotMovePatternHzPoint,
                                                preRobotMovePatternCount      = self.param_preRobotMovePatternCount,
                                                preRobotMovePatternSizeX      = self.param_preRobotMovePatternSizeX,
                                                preRobotMovePatternSizeY      = self.param_preRobotMovePatternSizeY,
                                                preRobotMovePatternParam      = self.param_preRobotMovePatternParam,
                                                preRobotMoveRelTracking       = self.param_preRobotMoveRelTracking,
                                                preRobotMoveRelOriginPosition = self.param_preRobotMoveRelOriginPosition,
                                                preRobotMoveRelOriginAngle    = self.param_preRobotMoveRelOriginAngle,
                                                preRobotMoveRelDistance       = self.param_preRobotMoveRelDistance,
                                                preRobotMoveRelAngle          = self.param_preRobotMoveRelAngleOffset,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                preRobotMoveTimeout           = self.param_preRobotMoveTimeout,
                                                )
        headerPreLaser_V26 = self.templatePreLaser_V26.format(
                                                preLaserEnabled               = self.param_preLaserEnabled,
                                                preLaserPatternShape          = self.param_preLaserPatternShape,
                                                preLaserPatternHzPattern      = self.param_preLaserPatternHzPattern,
                                                preLaserPatternHzPoint        = self.param_preLaserPatternHzPoint,
                                                preLaserPatternCount          = self.param_preLaserPatternCount,
                                                preLaserPatternSizeX          = self.param_preLaserPatternSizeX,
                                                preLaserPatternSizeY          = self.param_preLaserPatternSizeY,
                                                preLaserPatternParam          = self.param_preLaserPatternParam,
                                                preLaserStatefilterHi         = self.param_preLaserStatefilterHi,
                                                preLaserStatefilterLo         = self.param_preLaserStatefilterLo,
                                                preLaserStatefilterCriteria   = self.param_preLaserStatefilterCriteria,
                                                preLaserTimeout               = self.param_preLaserTimeout,
                                                )
        headerPreLEDPanels_V26 = self.templatePreLEDPanels_V26.format(
                                                preLEDPanelsEnabled             = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand             = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern           = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid             = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo       = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi       = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsTimeout             = self.param_preLEDPanelsTimeout,
                                                )
        
        #######################################################################
        headerPreWait1_V26 = self.templatePreWait1_V26.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V26 = self.templatePreTrigger_V26.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        headerPreWait2_V26 = self.templatePreWait2_V26.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        headerTrialRobot_V26 = self.templateTrialRobot_V26.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                trialRobotMoveTimeout           = self.param_trialRobotMoveTimeout,
                                                )
        
        headerTrialLaser_V26 = self.templateTrialLaser_V26.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                trialLaserTimeout               = self.param_trialLaserTimeout,
                                                )
        headerTrialLEDPanels_V26 = self.templateTrialLEDPanels_V26.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsTimeout           = self.param_trialLEDPanelsTimeout,
                                                )
            
        #######################################################################
        headerPostTrigger_V26 = self.templatePostTrigger_V26.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V26 = self.templatePostWait_V26.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerVersionFileTxt_V26)
            fid.write(headerVersionFile_V26)
            fid.write('\n')
    
            fid.write(self.headerExperimentTxt_V26)
            fid.write(headerExperiment_V26)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V26)
            fid.write(headerRobotSpec_V26)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V26)
            fid.write(headerFlySpec_V26)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V26)
            fid.write(headerTracking_V26)
            fid.write('\n')
    
            fid.write(self.headerPreRobotTxt_V26)
            fid.write(headerPreRobot_V26)
            fid.write('\n')
    
            fid.write(self.headerPreLaserTxt_V26)
            fid.write(headerPreLaser_V26)
            fid.write('\n')
    
            fid.write(self.headerPreLEDPanelsTxt_V26)
            fid.write(headerPreLEDPanels_V26)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V26)
            fid.write(headerPreWait1_V26)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V26)
            fid.write(headerPreTrigger_V26)
            fid.write('\n')
    
            fid.write(self.headerPreWait2Txt_V26)
            fid.write(headerPreWait2_V26)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V26)
            fid.write(headerTrialRobot_V26)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V26)
            fid.write(headerTrialLaser_V26)
            fid.write('\n')
    
            fid.write(self.headerTrialLEDPanelsTxt_V26)
            fid.write(headerTrialLEDPanels_V26)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V26)
            fid.write(headerPostTrigger_V26)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V26)
            fid.write(headerPostWait_V26)
            fid.write('\n')
            

    def WriteHeader_V27(self, filename):
        self.WriteHeader_V26(filename)
        

    def WriteHeader_V28(self, filename):
        headerVersionFile_V28 = self.templateVersionFile_V28.format(
                                                versionFile              = '2.8',
                                                ) 
        headerExperiment_V28 = self.templateExperiment_V28.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V28 = self.templateRobotSpec_V28.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        headerFlySpec_V28 = self.templateFlySpec_V28.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headerTrackingTxt_V28 = self.headerTrackingTxtA_V28
        headerTracking_V28 = self.templateTrackingA_V28.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V28 += self.headerTrackingTxtB_V28
            headerTracking_V28 += self.templateTrackingB_V28.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V28 += '\n'
        headerTracking_V28 += '\n'

        #######################################################################
        headerPreRobot_V28 = self.templatePreRobot_V28.format(
                                                preRobotEnabled               = self.param_preRobotEnabled,
                                                preRobotMovePatternFramePosition= self.param_preRobotMovePatternFramePosition,
                                                preRobotMovePatternFrameAngle = self.param_preRobotMovePatternFrameAngle,
                                                preRobotMovePatternShape      = self.param_preRobotMovePatternShape,
                                                preRobotMovePatternHzPattern  = self.param_preRobotMovePatternHzPattern,
                                                preRobotMovePatternHzPoint    = self.param_preRobotMovePatternHzPoint,
                                                preRobotMovePatternCount      = self.param_preRobotMovePatternCount,
                                                preRobotMovePatternSizeX      = self.param_preRobotMovePatternSizeX,
                                                preRobotMovePatternSizeY      = self.param_preRobotMovePatternSizeY,
                                                preRobotMovePatternParam      = self.param_preRobotMovePatternParam,
                                                preRobotMovePatternDirection  = self.param_preRobotMovePatternDirection,
                                                preRobotMoveRelTracking       = self.param_preRobotMoveRelTracking,
                                                preRobotMoveRelOriginPosition = self.param_preRobotMoveRelOriginPosition,
                                                preRobotMoveRelOriginAngle    = self.param_preRobotMoveRelOriginAngle,
                                                preRobotMoveRelDistance       = self.param_preRobotMoveRelDistance,
                                                preRobotMoveRelAngle          = self.param_preRobotMoveRelAngleOffset,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                preRobotMoveTimeout           = self.param_preRobotMoveTimeout,
                                                )
        headerPreLaser_V28 = self.templatePreLaser_V28.format(
                                                preLaserEnabled               = self.param_preLaserEnabled,
                                                preLaserPatternFramePosition  = self.param_preLaserPatternFramePosition,
                                                preLaserPatternFrameAngle     = self.param_preLaserPatternFrameAngle,
                                                preLaserPatternShape          = self.param_preLaserPatternShape,
                                                preLaserPatternHzPattern      = self.param_preLaserPatternHzPattern,
                                                preLaserPatternHzPoint        = self.param_preLaserPatternHzPoint,
                                                preLaserPatternCount          = self.param_preLaserPatternCount,
                                                preLaserPatternSizeX          = self.param_preLaserPatternSizeX,
                                                preLaserPatternSizeY          = self.param_preLaserPatternSizeY,
                                                preLaserPatternParam          = self.param_preLaserPatternParam,
                                                preLaserPatternDirection      = self.param_preLaserPatternDirection,
                                                preLaserStatefilterHi         = self.param_preLaserStatefilterHi,
                                                preLaserStatefilterLo         = self.param_preLaserStatefilterLo,
                                                preLaserStatefilterCriteria   = self.param_preLaserStatefilterCriteria,
                                                preLaserTimeout               = self.param_preLaserTimeout,
                                                )
        headerPreLEDPanels_V28 = self.templatePreLEDPanels_V28.format(
                                                preLEDPanelsEnabled = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsTimeout = self.param_preLEDPanelsTimeout,
                                                )
        
        #######################################################################
        headerPreWait1_V28 = self.templatePreWait1_V28.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V28 = self.templatePreTrigger_V28.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        headerPreWait2_V28 = self.templatePreWait2_V28.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        headerTrialRobot_V28 = self.templateTrialRobot_V28.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternFramePosition= self.param_trialRobotMovePatternFramePosition,
                                                trialRobotMovePatternFrameAngle = self.param_trialRobotMovePatternFrameAngle,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMovePatternDirection  = self.param_trialRobotMovePatternDirection,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                trialRobotMoveTimeout           = self.param_trialRobotMoveTimeout,
                                                )
        
        headerTrialLaser_V28 = self.templateTrialLaser_V28.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternFramePosition  = self.param_trialLaserPatternFramePosition,
                                                trialLaserPatternFrameAngle     = self.param_trialLaserPatternFrameAngle,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserPatternDirection      = self.param_trialLaserPatternDirection,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                trialLaserTimeout               = self.param_trialLaserTimeout,
                                                )
        headerTrialLEDPanels_V28 = self.templateTrialLEDPanels_V28.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsTimeout           = self.param_trialLEDPanelsTimeout,
                                                )
            
        #######################################################################
        headerPostTrigger_V28 = self.templatePostTrigger_V28.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V28 = self.templatePostWait_V28.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerVersionFileTxt_V28)
            fid.write(headerVersionFile_V28)
            fid.write('\n')
    
            fid.write(self.headerExperimentTxt_V28)
            fid.write(headerExperiment_V28)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V28)
            fid.write(headerRobotSpec_V28)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V28)
            fid.write(headerFlySpec_V28)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V28)
            fid.write(headerTracking_V28)
            fid.write('\n')
    
            fid.write(self.headerPreRobotTxt_V28)
            fid.write(headerPreRobot_V28)
            fid.write('\n')
    
            fid.write(self.headerPreLaserTxt_V28)
            fid.write(headerPreLaser_V28)
            fid.write('\n')
    
            fid.write(self.headerPreLEDPanelsTxt_V28)
            fid.write(headerPreLEDPanels_V28)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V28)
            fid.write(headerPreWait1_V28)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V28)
            fid.write(headerPreTrigger_V28)
            fid.write('\n')
    
            fid.write(self.headerPreWait2Txt_V28)
            fid.write(headerPreWait2_V28)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V28)
            fid.write(headerTrialRobot_V28)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V28)
            fid.write(headerTrialLaser_V28)
            fid.write('\n')
    
            fid.write(self.headerTrialLEDPanelsTxt_V28)
            fid.write(headerTrialLEDPanels_V28)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V28)
            fid.write(headerPostTrigger_V28)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V28)
            fid.write(headerPostWait_V28)
            fid.write('\n')
            

    def WriteHeader_V281(self, filename):
        headerVersionFile_V281 = self.templateVersionFile_V281.format(
                                                versionFile              = '2.81',
                                                ) 
        headerExperiment_V281 = self.templateExperiment_V281.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V281 = self.templateRobotSpec_V281.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        headerFlySpec_V281 = self.templateFlySpec_V281.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headerTrackingTxt_V281 = self.headerTrackingTxtA_V281
        headerTracking_V281 = self.templateTrackingA_V281.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V281 += self.headerTrackingTxtB_V281
            headerTracking_V281 += self.templateTrackingB_V281.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V281 += '\n'
        headerTracking_V281 += '\n'

        #######################################################################
        headerPreRobot_V281 = self.templatePreRobot_V281.format(
                                                preRobotEnabled               = self.param_preRobotEnabled,
                                                preRobotMovePatternFramePosition= self.param_preRobotMovePatternFramePosition,
                                                preRobotMovePatternFrameAngle = self.param_preRobotMovePatternFrameAngle,
                                                preRobotMovePatternShape      = self.param_preRobotMovePatternShape,
                                                preRobotMovePatternHzPattern  = self.param_preRobotMovePatternHzPattern,
                                                preRobotMovePatternHzPoint    = self.param_preRobotMovePatternHzPoint,
                                                preRobotMovePatternCount      = self.param_preRobotMovePatternCount,
                                                preRobotMovePatternSizeX      = self.param_preRobotMovePatternSizeX,
                                                preRobotMovePatternSizeY      = self.param_preRobotMovePatternSizeY,
                                                preRobotMovePatternParam      = self.param_preRobotMovePatternParam,
                                                preRobotMovePatternDirection  = self.param_preRobotMovePatternDirection,
                                                preRobotMoveRelTracking       = self.param_preRobotMoveRelTracking,
                                                preRobotMoveRelOriginPosition = self.param_preRobotMoveRelOriginPosition,
                                                preRobotMoveRelOriginAngle    = self.param_preRobotMoveRelOriginAngle,
                                                preRobotMoveRelDistance       = self.param_preRobotMoveRelDistance,
                                                preRobotMoveRelAngle          = self.param_preRobotMoveRelAngleOffset,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                )
        headerPreLaser_V281 = self.templatePreLaser_V281.format(
                                                preLaserEnabled               = self.param_preLaserEnabled,
                                                preLaserPatternFramePosition  = self.param_preLaserPatternFramePosition,
                                                preLaserPatternFrameAngle     = self.param_preLaserPatternFrameAngle,
                                                preLaserPatternShape          = self.param_preLaserPatternShape,
                                                preLaserPatternHzPattern      = self.param_preLaserPatternHzPattern,
                                                preLaserPatternHzPoint        = self.param_preLaserPatternHzPoint,
                                                preLaserPatternCount          = self.param_preLaserPatternCount,
                                                preLaserPatternSizeX          = self.param_preLaserPatternSizeX,
                                                preLaserPatternSizeY          = self.param_preLaserPatternSizeY,
                                                preLaserPatternParam          = self.param_preLaserPatternParam,
                                                preLaserPatternDirection      = self.param_preLaserPatternDirection,
                                                preLaserStatefilterHi         = self.param_preLaserStatefilterHi,
                                                preLaserStatefilterLo         = self.param_preLaserStatefilterLo,
                                                preLaserStatefilterCriteria   = self.param_preLaserStatefilterCriteria,
                                                )
        headerPreLEDPanels_V281 = self.templatePreLEDPanels_V281.format(
                                                preLEDPanelsEnabled = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                )
        
        #######################################################################
        headerPreWait1_V281 = self.templatePreWait1_V281.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V281 = self.templatePreTrigger_V281.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        headerPreWait2_V281 = self.templatePreWait2_V281.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        headerTrialRobot_V281 = self.templateTrialRobot_V281.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternFramePosition= self.param_trialRobotMovePatternFramePosition,
                                                trialRobotMovePatternFrameAngle = self.param_trialRobotMovePatternFrameAngle,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMovePatternDirection  = self.param_trialRobotMovePatternDirection,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        headerTrialLaser_V281 = self.templateTrialLaser_V281.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternFramePosition  = self.param_trialLaserPatternFramePosition,
                                                trialLaserPatternFrameAngle     = self.param_trialLaserPatternFrameAngle,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserPatternDirection      = self.param_trialLaserPatternDirection,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                )
        headerTrialLEDPanels_V281 = self.templateTrialLEDPanels_V281.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                )
            
        #######################################################################
        headerPostTrigger_V281 = self.templatePostTrigger_V281.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V281 = self.templatePostWait_V281.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerVersionFileTxt_V281)
            fid.write(headerVersionFile_V281)
            fid.write('\n')
    
            fid.write(self.headerExperimentTxt_V281)
            fid.write(headerExperiment_V281)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V281)
            fid.write(headerRobotSpec_V281)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V281)
            fid.write(headerFlySpec_V281)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V281)
            fid.write(headerTracking_V281)
            fid.write('\n')
    
            fid.write(self.headerPreRobotTxt_V281)
            fid.write(headerPreRobot_V281)
            fid.write('\n')
    
            fid.write(self.headerPreLaserTxt_V281)
            fid.write(headerPreLaser_V281)
            fid.write('\n')
    
            fid.write(self.headerPreLEDPanelsTxt_V281)
            fid.write(headerPreLEDPanels_V281)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V281)
            fid.write(headerPreWait1_V281)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V281)
            fid.write(headerPreTrigger_V281)
            fid.write('\n')
    
            fid.write(self.headerPreWait2Txt_V281)
            fid.write(headerPreWait2_V281)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V281)
            fid.write(headerTrialRobot_V281)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V281)
            fid.write(headerTrialLaser_V281)
            fid.write('\n')
    
            fid.write(self.headerTrialLEDPanelsTxt_V281)
            fid.write(headerTrialLEDPanels_V281)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V281)
            fid.write(headerPostTrigger_V281)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V281)
            fid.write(headerPostWait_V281)
            fid.write('\n')
            

    def WriteHeader_V282(self, filename):
        headerVersionFile_V282 = self.templateVersionFile_V282.format(
                                                versionFile              = '2.82',
                                                ) 
        headerExperiment_V282 = self.templateExperiment_V282.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V282 = self.templateRobotSpec_V282.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        headerFlySpec_V282 = self.templateFlySpec_V282.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headerTrackingTxt_V282 = self.headerTrackingTxtA_V282
        headerTracking_V282 = self.templateTrackingA_V282.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V282 += self.headerTrackingTxtB_V282
            headerTracking_V282 += self.templateTrackingB_V282.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V282 += '\n'
        headerTracking_V282 += '\n'

        #######################################################################
        headerPreRobot_V282 = self.templatePreRobot_V282.format(
                                                preRobotEnabled               = self.param_preRobotEnabled,
                                                preRobotMovePatternFramePosition= self.param_preRobotMovePatternFramePosition,
                                                preRobotMovePatternFrameAngle = self.param_preRobotMovePatternFrameAngle,
                                                preRobotMovePatternShape      = self.param_preRobotMovePatternShape,
                                                preRobotMovePatternHzPattern  = self.param_preRobotMovePatternHzPattern,
                                                preRobotMovePatternHzPoint    = self.param_preRobotMovePatternHzPoint,
                                                preRobotMovePatternCount      = self.param_preRobotMovePatternCount,
                                                preRobotMovePatternSizeX      = self.param_preRobotMovePatternSizeX,
                                                preRobotMovePatternSizeY      = self.param_preRobotMovePatternSizeY,
                                                preRobotMovePatternParam      = self.param_preRobotMovePatternParam,
                                                preRobotMovePatternDirection  = self.param_preRobotMovePatternDirection,
                                                preRobotMoveRelTracking       = self.param_preRobotMoveRelTracking,
                                                preRobotMoveRelOriginPosition = self.param_preRobotMoveRelOriginPosition,
                                                preRobotMoveRelOriginAngle    = self.param_preRobotMoveRelOriginAngle,
                                                preRobotMoveRelDistance       = self.param_preRobotMoveRelDistance,
                                                preRobotMoveRelAngleOffset    = self.param_preRobotMoveRelAngleOffset,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelAngleOscMag    = self.param_preRobotMoveRelAngleOscMag,
                                                preRobotMoveRelAngleOscFreq   = self.param_preRobotMoveRelAngleOscFreq,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                )
        headerPreLaser_V282 = self.templatePreLaser_V282.format(
                                                preLaserEnabled               = self.param_preLaserEnabled,
                                                preLaserPatternFramePosition  = self.param_preLaserPatternFramePosition,
                                                preLaserPatternFrameAngle     = self.param_preLaserPatternFrameAngle,
                                                preLaserPatternShape          = self.param_preLaserPatternShape,
                                                preLaserPatternHzPattern      = self.param_preLaserPatternHzPattern,
                                                preLaserPatternHzPoint        = self.param_preLaserPatternHzPoint,
                                                preLaserPatternCount          = self.param_preLaserPatternCount,
                                                preLaserPatternSizeX          = self.param_preLaserPatternSizeX,
                                                preLaserPatternSizeY          = self.param_preLaserPatternSizeY,
                                                preLaserPatternParam          = self.param_preLaserPatternParam,
                                                preLaserPatternDirection      = self.param_preLaserPatternDirection,
                                                preLaserStatefilterHi         = self.param_preLaserStatefilterHi,
                                                preLaserStatefilterLo         = self.param_preLaserStatefilterLo,
                                                preLaserStatefilterCriteria   = self.param_preLaserStatefilterCriteria,
                                                )
        headerPreLEDPanels_V282 = self.templatePreLEDPanels_V282.format(
                                                preLEDPanelsEnabled = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                )
        
        #######################################################################
        headerPreWait1_V282 = self.templatePreWait1_V282.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V282 = self.templatePreTrigger_V282.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        headerPreWait2_V282 = self.templatePreWait2_V282.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        headerTrialRobot_V282 = self.templateTrialRobot_V282.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternFramePosition= self.param_trialRobotMovePatternFramePosition,
                                                trialRobotMovePatternFrameAngle = self.param_trialRobotMovePatternFrameAngle,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMovePatternDirection  = self.param_trialRobotMovePatternDirection,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngleOffset    = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelAngleOscMag    = self.param_trialRobotMoveRelAngleOscMag,
                                                trialRobotMoveRelAngleOscFreq   = self.param_trialRobotMoveRelAngleOscFreq,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        headerTrialLaser_V282 = self.templateTrialLaser_V282.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternFramePosition  = self.param_trialLaserPatternFramePosition,
                                                trialLaserPatternFrameAngle     = self.param_trialLaserPatternFrameAngle,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserPatternDirection      = self.param_trialLaserPatternDirection,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                )
        headerTrialLEDPanels_V282 = self.templateTrialLEDPanels_V282.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                )
            
        #######################################################################
        headerPostTrigger_V282 = self.templatePostTrigger_V282.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V282 = self.templatePostWait_V282.format(
                                                postWait                      = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerVersionFileTxt_V282)
            fid.write(headerVersionFile_V282)
            fid.write('\n')
    
            fid.write(self.headerExperimentTxt_V282)
            fid.write(headerExperiment_V282)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V282)
            fid.write(headerRobotSpec_V282)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V282)
            fid.write(headerFlySpec_V282)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V282)
            fid.write(headerTracking_V282)
            fid.write('\n')
    
            fid.write(self.headerPreRobotTxt_V282)
            fid.write(headerPreRobot_V282)
            fid.write('\n')
    
            fid.write(self.headerPreLaserTxt_V282)
            fid.write(headerPreLaser_V282)
            fid.write('\n')
    
            fid.write(self.headerPreLEDPanelsTxt_V282)
            fid.write(headerPreLEDPanels_V282)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V282)
            fid.write(headerPreWait1_V282)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V282)
            fid.write(headerPreTrigger_V282)
            fid.write('\n')
    
            fid.write(self.headerPreWait2Txt_V282)
            fid.write(headerPreWait2_V282)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V282)
            fid.write(headerTrialRobot_V282)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V282)
            fid.write(headerTrialLaser_V282)
            fid.write('\n')
    
            fid.write(self.headerTrialLEDPanelsTxt_V282)
            fid.write(headerTrialLEDPanels_V282)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V282)
            fid.write(headerPostTrigger_V282)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V282)
            fid.write(headerPostWait_V282)
            fid.write('\n')
            

    def WriteHeader_V283(self, filename):
        headerVersionFile_V283 = self.templateVersionFile_V283.format(
                                                versionFile              = '2.83',
                                                ) 
        headerExperiment_V283 = self.templateExperiment_V283.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        headerRobotSpec_V283 = self.templateRobotSpec_V283.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                descriptionRobot           = self.param_descriptionRobot,
                                                )
        headerFlySpec_V283 = self.templateFlySpec_V283.format(
                                                nFlies                     = self.param_nFlies,
                                                descriptionFlies           = self.param_descriptionFlies,
                                                )

        self.headerTrackingTxt_V283 = self.headerTrackingTxtA_V283
        headerTracking_V283 = self.templateTrackingA_V283.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headerTrackingTxt_V283 += self.headerTrackingTxtB_V283
            headerTracking_V283 += self.templateTrackingB_V283.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headerTrackingTxt_V283 += '\n'
        headerTracking_V283 += '\n'

        #######################################################################
        headerPreRobot_V283 = self.templatePreRobot_V283.format(
                                                preRobotEnabled               = self.param_preRobotEnabled,
                                                preRobotMovePatternFramePosition= self.param_preRobotMovePatternFramePosition,
                                                preRobotMovePatternFrameAngle = self.param_preRobotMovePatternFrameAngle,
                                                preRobotMovePatternShape      = self.param_preRobotMovePatternShape,
                                                preRobotMovePatternHzPattern  = self.param_preRobotMovePatternHzPattern,
                                                preRobotMovePatternHzPoint    = self.param_preRobotMovePatternHzPoint,
                                                preRobotMovePatternCount      = self.param_preRobotMovePatternCount,
                                                preRobotMovePatternSizeX      = self.param_preRobotMovePatternSizeX,
                                                preRobotMovePatternSizeY      = self.param_preRobotMovePatternSizeY,
                                                preRobotMovePatternParam      = self.param_preRobotMovePatternParam,
                                                preRobotMovePatternDirection  = self.param_preRobotMovePatternDirection,
                                                preRobotMoveRelTracking       = self.param_preRobotMoveRelTracking,
                                                preRobotMoveRelOriginPosition = self.param_preRobotMoveRelOriginPosition,
                                                preRobotMoveRelOriginAngle    = self.param_preRobotMoveRelOriginAngle,
                                                preRobotMoveRelDistance       = self.param_preRobotMoveRelDistance,
                                                preRobotMoveRelAngleOffset    = self.param_preRobotMoveRelAngleOffset,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelAngleOscMag    = self.param_preRobotMoveRelAngleOscMag,
                                                preRobotMoveRelAngleOscFreq   = self.param_preRobotMoveRelAngleOscFreq,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                )
        headerPreLaser_V283 = self.templatePreLaser_V283.format(
                                                preLaserEnabled               = self.param_preLaserEnabled,
                                                preLaserPatternFramePosition  = self.param_preLaserPatternFramePosition,
                                                preLaserPatternFrameAngle     = self.param_preLaserPatternFrameAngle,
                                                preLaserPatternShape          = self.param_preLaserPatternShape,
                                                preLaserPatternHzPattern      = self.param_preLaserPatternHzPattern,
                                                preLaserPatternHzPoint        = self.param_preLaserPatternHzPoint,
                                                preLaserPatternCount          = self.param_preLaserPatternCount,
                                                preLaserPatternSizeX          = self.param_preLaserPatternSizeX,
                                                preLaserPatternSizeY          = self.param_preLaserPatternSizeY,
                                                preLaserPatternParam          = self.param_preLaserPatternParam,
                                                preLaserPatternDirection      = self.param_preLaserPatternDirection,
                                                preLaserStatefilterHi         = self.param_preLaserStatefilterHi,
                                                preLaserStatefilterLo         = self.param_preLaserStatefilterLo,
                                                preLaserStatefilterCriteria   = self.param_preLaserStatefilterCriteria,
                                                )
        headerPreLEDPanels_V283 = self.templatePreLEDPanels_V283.format(
                                                preLEDPanelsEnabled = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                )
        
        #######################################################################
        headerPreWait1_V283 = self.templatePreWait1_V283.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        headerPreTrigger_V283 = self.templatePreTrigger_V283.format(
                                                preTriggerEnabled            = self.param_preTriggerEnabled,
                                                preTriggerFrameidParent      = self.param_preTriggerFrameidParent,
                                                preTriggerFrameidChild       = self.param_preTriggerFrameidChild,
                                                preTriggerSpeedAbsParentMin  = self.param_preTriggerSpeedAbsParentMin,
                                                preTriggerSpeedAbsParentMax  = self.param_preTriggerSpeedAbsParentMax,
                                                preTriggerSpeedAbsChildMin   = self.param_preTriggerSpeedAbsChildMin,
                                                preTriggerSpeedAbsChildMax   = self.param_preTriggerSpeedAbsChildMax,
                                                preTriggerSpeedRelMin        = self.param_preTriggerSpeedRelMin,
                                                preTriggerSpeedRelMax        = self.param_preTriggerSpeedRelMax,
                                                preTriggerDistanceMin        = self.param_preTriggerDistanceMin,
                                                preTriggerDistanceMax        = self.param_preTriggerDistanceMax,
                                                preTriggerAngleMin           = self.param_preTriggerAngleMin,
                                                preTriggerAngleMax           = self.param_preTriggerAngleMax,
                                                preTriggerAngleTest          = self.param_preTriggerAngleTest,
                                                preTriggerAngleTestBilateral = self.param_preTriggerAngleTestBilateral,
                                                preTriggerTimeHold           = self.param_preTriggerTimeHold,
                                                preTriggerTimeout            = self.param_preTriggerTimeout,
                                                )
        headerPreWait2_V283 = self.templatePreWait2_V283.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        headerTrialRobot_V283 = self.templateTrialRobot_V283.format(
                                                trialRobotEnabled               = self.param_trialRobotEnabled,
                                                trialRobotMovePatternFramePosition= self.param_trialRobotMovePatternFramePosition,
                                                trialRobotMovePatternFrameAngle = self.param_trialRobotMovePatternFrameAngle,
                                                trialRobotMovePatternShape      = self.param_trialRobotMovePatternShape,
                                                trialRobotMovePatternHzPattern  = self.param_trialRobotMovePatternHzPattern,
                                                trialRobotMovePatternHzPoint    = self.param_trialRobotMovePatternHzPoint,
                                                trialRobotMovePatternCount      = self.param_trialRobotMovePatternCount,
                                                trialRobotMovePatternSizeX      = self.param_trialRobotMovePatternSizeX,
                                                trialRobotMovePatternSizeY      = self.param_trialRobotMovePatternSizeY,
                                                trialRobotMovePatternParam      = self.param_trialRobotMovePatternParam,
                                                trialRobotMovePatternDirection  = self.param_trialRobotMovePatternDirection,
                                                trialRobotMoveRelTracking       = self.param_trialRobotMoveRelTracking,
                                                trialRobotMoveRelOriginPosition = self.param_trialRobotMoveRelOriginPosition,
                                                trialRobotMoveRelOriginAngle    = self.param_trialRobotMoveRelOriginAngle,
                                                trialRobotMoveRelDistance       = self.param_trialRobotMoveRelDistance,
                                                trialRobotMoveRelAngleOffset    = self.param_trialRobotMoveRelAngleOffset,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelAngleOscMag    = self.param_trialRobotMoveRelAngleOscMag,
                                                trialRobotMoveRelAngleOscFreq   = self.param_trialRobotMoveRelAngleOscFreq,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        headerTrialLaser_V283 = self.templateTrialLaser_V283.format(
                                                trialLaserEnabled               = self.param_trialLaserEnabled,
                                                trialLaserPatternFramePosition  = self.param_trialLaserPatternFramePosition,
                                                trialLaserPatternFrameAngle     = self.param_trialLaserPatternFrameAngle,
                                                trialLaserPatternShape          = self.param_trialLaserPatternShape,
                                                trialLaserPatternHzPattern      = self.param_trialLaserPatternHzPattern,
                                                trialLaserPatternHzPoint        = self.param_trialLaserPatternHzPoint,
                                                trialLaserPatternCount          = self.param_trialLaserPatternCount,
                                                trialLaserPatternSizeX          = self.param_trialLaserPatternSizeX,
                                                trialLaserPatternSizeY          = self.param_trialLaserPatternSizeY,
                                                trialLaserPatternParam          = self.param_trialLaserPatternParam,
                                                trialLaserPatternDirection      = self.param_trialLaserPatternDirection,
                                                trialLaserStatefilterHi         = self.param_trialLaserStatefilterHi,
                                                trialLaserStatefilterLo         = self.param_trialLaserStatefilterLo,
                                                trialLaserStatefilterCriteria   = self.param_trialLaserStatefilterCriteria,
                                                )
        headerTrialLEDPanels_V283 = self.templateTrialLEDPanels_V283.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                )
            
        #######################################################################
        headerPostTrigger_V283 = self.templatePostTrigger_V283.format(
                                                postTriggerEnabled            = str(self.param_postTriggerEnabled),
                                                postTriggerFrameidParent      = str(self.param_postTriggerFrameidParent),
                                                postTriggerFrameidChild       = str(self.param_postTriggerFrameidChild),
                                                postTriggerSpeedAbsParentMin  = str(self.param_postTriggerSpeedAbsParentMin),
                                                postTriggerSpeedAbsParentMax  = str(self.param_postTriggerSpeedAbsParentMax),
                                                postTriggerSpeedAbsChildMin   = str(self.param_postTriggerSpeedAbsChildMin),
                                                postTriggerSpeedAbsChildMax   = str(self.param_postTriggerSpeedAbsChildMax),
                                                postTriggerSpeedRelMin        = str(self.param_postTriggerSpeedRelMin),
                                                postTriggerSpeedRelMax        = str(self.param_postTriggerSpeedRelMax),
                                                postTriggerDistanceMin        = str(self.param_postTriggerDistanceMin),
                                                postTriggerDistanceMax        = str(self.param_postTriggerDistanceMax),
                                                postTriggerAngleMin           = str(self.param_postTriggerAngleMin),
                                                postTriggerAngleMax           = str(self.param_postTriggerAngleMax),
                                                postTriggerAngleTest          = str(self.param_postTriggerAngleTest),
                                                postTriggerAngleTestBilateral = str(self.param_postTriggerAngleTestBilateral),
                                                postTriggerTimeHold           = str(self.param_postTriggerTimeHold),
                                                postTriggerTimeout            = str(self.param_postTriggerTimeout),
                                                )
        headerPostWait_V283 = self.templatePostWait_V283.format(
                                                postWait                      = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headerVersionFileTxt_V283)
            fid.write(headerVersionFile_V283)
            fid.write('\n')
    
            fid.write(self.headerExperimentTxt_V283)
            fid.write(headerExperiment_V283)
            fid.write('\n')
    
            fid.write(self.headerRobotSpecTxt_V283)
            fid.write(headerRobotSpec_V283)
            fid.write('\n')
    
            fid.write(self.headerFlySpecTxt_V283)
            fid.write(headerFlySpec_V283)
            fid.write('\n')
    
            fid.write(self.headerTrackingTxt_V283)
            fid.write(headerTracking_V283)
            fid.write('\n')
    
            fid.write(self.headerPreRobotTxt_V283)
            fid.write(headerPreRobot_V283)
            fid.write('\n')
    
            fid.write(self.headerPreLaserTxt_V283)
            fid.write(headerPreLaser_V283)
            fid.write('\n')
    
            fid.write(self.headerPreLEDPanelsTxt_V283)
            fid.write(headerPreLEDPanels_V283)
            fid.write('\n')
    
            fid.write(self.headerPreWait1Txt_V283)
            fid.write(headerPreWait1_V283)
            fid.write('\n')
    
            fid.write(self.headerPreTriggerTxt_V283)
            fid.write(headerPreTrigger_V283)
            fid.write('\n')
    
            fid.write(self.headerPreWait2Txt_V283)
            fid.write(headerPreWait2_V283)
            fid.write('\n')
    
            fid.write(self.headerTrialRobotTxt_V283)
            fid.write(headerTrialRobot_V283)
            fid.write('\n')
    
            fid.write(self.headerTrialLaserTxt_V283)
            fid.write(headerTrialLaser_V283)
            fid.write('\n')
    
            fid.write(self.headerTrialLEDPanelsTxt_V283)
            fid.write(headerTrialLEDPanels_V283)
            fid.write('\n')
    
            fid.write(self.headerPostTriggerTxt_V283)
            fid.write(headerPostTrigger_V283)
            fid.write('\n')
    
            fid.write(self.headerPostWaitTxt_V283)
            fid.write(headerPostWait_V283)
            fid.write('\n')
            

    # Write the state lines from any-version input file to a pre-2.6 version output file.
    # Pre-version 2.6 doesn't include the 'triggered' column, nor the 'wings' columns.
    def WriteStateLines_Vpre26(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'time')
            tmp = fidIn.readline()   # Skip the old heading text.
            
            with open(filenameOut, 'a') as fidOut:
                
                # Write the new heading line.
                stateTxt = self.stateLeftTxt_Vpre26 + self.stateRobotTxt_Vpre26
                for i in range(nFlies):
                    stateTxt += self.stateFlyTxt_Vpre26
                stateTxt += '\n'
                fidOut.write(stateTxt)    
                
                
                # Write the state lines.
                for lineIn in fidIn:
                    fieldsIn_list = self.ListFromCsv(lineIn)


                    ############### Fields:  Time,Triggered
                    stateLeft = self.templateStateLeft_Vpre26.format(
                                                                    time = fieldsIn_list[0],
                                                                    )
                    if (float(versionIn) < 2.6):
                        iFieldIn = 1
                    else:
                        iFieldIn = 2  # Skip the 'triggered' column.
                        
                    state = stateLeft

                        
                    ############### Fields:  Robot state.
                    stateRobot = self.templateStateRobot_Vpre26.format(
                                                            xRobot = fieldsIn_list[iFieldIn+0],
                                                            yRobot = fieldsIn_list[iFieldIn+1],
                                                            aRobot = fieldsIn_list[iFieldIn+2],
                                                            vxRobot = fieldsIn_list[iFieldIn+3],
                                                            vyRobot = fieldsIn_list[iFieldIn+4],
                                                            vaRobot = fieldsIn_list[iFieldIn+5],
                                                            )
                    
                    if (float(versionIn) < 2.7):
                        iFieldIn += 6
                    else:
                        iFieldIn += 8   # Skip the wings fields.
                        
                    state += stateRobot
                    
                    
                    ############### Fields:  Fly state.
                    for iFly in range(nFlies): 
                        stateFly = self.templateStateFly_Vpre26.format(
                                                            xFly = fieldsIn_list[iFieldIn+0],
                                                            yFly = fieldsIn_list[iFieldIn+1],
                                                            aFly = fieldsIn_list[iFieldIn+2],
                                                            vxFly = fieldsIn_list[iFieldIn+3],
                                                            vyFly = fieldsIn_list[iFieldIn+4],
                                                            vaFly = fieldsIn_list[iFieldIn+5],
                                                            )
                        if (float(versionIn) < 2.7):
                            iFieldIn += 6
                        else:
                            iFieldIn += 8   # Skip the wings fields.

                        state += stateFly
                        
                        
                    ############### End of the line.
                    state += '\n'
                    fidOut.write(state)
                    

    # Write the state lines from any-version input file to a 2.6 version output file.
    # Versions >= 2.6 include the 'triggered' column in the state.
    def WriteStateLines_V26(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'time')
            tmp = fidIn.readline()   # Skip the old heading text.
            
            with open(filenameOut, 'a') as fidOut:
                
                # Write the new heading line.
                stateTxt = self.stateLeftTxt_V26 + self.stateRobotTxt_V26
                for i in range(nFlies):
                    stateTxt += self.stateFlyTxt_V26
                stateTxt += '\n'
                fidOut.write(stateTxt)    
                
                
                # Write the state lines.
                for lineIn in fidIn:
                    fieldsIn_list = self.ListFromCsv(lineIn)

                    ############### Fields:  Time,Triggered
                    if (float(versionIn) < 2.6):
                        stateLeft = self.templateStateLeft_V26.format(
                                                            time = fieldsIn_list[0],
                                                            triggered = '1',
                                                            )
                        iFieldIn = 1
                    else:      # Use 'triggered' value from file.
                        stateLeft = self.templateStateLeft_V26.format(
                                                            time = fieldsIn_list[0],
                                                            triggered = fieldsIn_list[1],
                                                            )
                        iFieldIn = 2  # Skip the 'triggered' column.
                        
                    state = stateLeft

                        
                    ############### Fields:  Robot state.
                    stateRobot = self.templateStateRobot_V26.format(
                                                            xRobot = fieldsIn_list[iFieldIn+0],
                                                            yRobot = fieldsIn_list[iFieldIn+1],
                                                            aRobot = fieldsIn_list[iFieldIn+2],
                                                            vxRobot = fieldsIn_list[iFieldIn+3],
                                                            vyRobot = fieldsIn_list[iFieldIn+4],
                                                            vaRobot = fieldsIn_list[iFieldIn+5],
                                                            )
                    if (float(versionIn) < 2.7):    # Pre-2.7 had no 'wing' columns.
                        iFieldIn += 6
                    else:
                        iFieldIn += 8
                        
                    state += stateRobot

                    
                    ############### Fields:  Fly state.
                    for iFly in range(nFlies): 
                        stateFly = self.templateStateFly_V26.format(
                                                            xFly = fieldsIn_list[iFieldIn+0],
                                                            yFly = fieldsIn_list[iFieldIn+1],
                                                            aFly = fieldsIn_list[iFieldIn+2],
                                                            vxFly = fieldsIn_list[iFieldIn+3],
                                                            vyFly = fieldsIn_list[iFieldIn+4],
                                                            vaFly = fieldsIn_list[iFieldIn+5],
                                                            )
                        if (float(versionIn) < 2.7):    # Pre-2.7 had no 'wing angle' columns.
                            iFieldIn += 6
                        else:
                            iFieldIn += 8
                            
                        state += stateFly

                        
                    ############### End of the line.
                    state += '\n'
                    fidOut.write(state)


    # Write the state lines from any-version input file to a 2.7 version output file.
    # Versions >= 2.7 include wing angles.
    def WriteStateLines_V27(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'time')
            tmp = fidIn.readline()   # Skip the old heading text.
            
            with open(filenameOut, 'a') as fidOut:
                
                # Write the new heading line.
                stateTxt = self.stateLeftTxt_V27 + self.stateRobotTxt_V27
                for i in range(nFlies):
                    stateTxt += self.stateFlyTxt_V27
                stateTxt += '\n'
                fidOut.write(stateTxt)    
                
                
                # Write the state lines.
                for lineIn in fidIn:
                    fieldsIn_list = self.ListFromCsv(lineIn)

                    ############### Fields:  Time,Triggered
                    if (float(versionIn) < 2.6):    # Pre-2.6 had no 'triggered' column.  Use default value.
                        stateLeft = self.templateStateLeft_V27.format(
                                                                        time      = fieldsIn_list[0],
                                                                        triggered = '1',
                                                                        )
                        iFieldIn = 1
                    
                    else:      # Use 'triggered' value from file.
                        stateLeft = self.templateStateLeft_V27.format(
                                                                        time      = fieldsIn_list[0],
                                                                        triggered = fieldsIn_list[1],
                                                                        )
                        iFieldIn = 2
                        
                    state = stateLeft
                    
                    
                    ############### Fields:  Robot state.
                    if (float(versionIn) < 2.7):    # Pre-2.7 had no 'wing angle' columns.  Use default values.
                        stateRobot = self.templateStateRobot_V27.format(
                                                                xRobot          = fieldsIn_list[iFieldIn+0],
                                                                yRobot          = fieldsIn_list[iFieldIn+1],
                                                                aRobot          = fieldsIn_list[iFieldIn+2],
                                                                vxRobot         = fieldsIn_list[iFieldIn+3],
                                                                vyRobot         = fieldsIn_list[iFieldIn+4],
                                                                vaRobot         = fieldsIn_list[iFieldIn+5],
                                                                aRobotWingLeft  = '%5.3f' % N.pi,
                                                                aRobotWingRight = '%5.3f' % N.pi,
                                                                )
                        iFieldIn += 6
                        
                    else:      # Use 'wing angle' values from file.
                        stateRobot = self.templateStateRobot_V27.format(
                                                                xRobot          = fieldsIn_list[iFieldIn+0],
                                                                yRobot          = fieldsIn_list[iFieldIn+1],
                                                                aRobot          = fieldsIn_list[iFieldIn+2],
                                                                vxRobot         = fieldsIn_list[iFieldIn+3],
                                                                vyRobot         = fieldsIn_list[iFieldIn+4],
                                                                vaRobot         = fieldsIn_list[iFieldIn+5],
                                                                aRobotWingLeft  = fieldsIn_list[iFieldIn+6],
                                                                aRobotWingRight = fieldsIn_list[iFieldIn+7],
                                                                )
                        iFieldIn += 8
                        
                        
                    state += stateRobot
                    

                    ############### Fields:  Fly state.
                    for iFly in range(nFlies): 
                        if (float(versionIn) < 2.7):    # Pre-2.7 had no 'wing' columns.  Use default values.
                            stateFly = self.templateStateFly_V27.format(
                                                                xFly        = fieldsIn_list[iFieldIn+0],
                                                                yFly        = fieldsIn_list[iFieldIn+1],
                                                                aFly        = fieldsIn_list[iFieldIn+2],
                                                                vxFly       = fieldsIn_list[iFieldIn+3],
                                                                vyFly       = fieldsIn_list[iFieldIn+4],
                                                                vaFly       = fieldsIn_list[iFieldIn+5],
                                                                aFlyWingLeft  = '%5.3f' % N.pi,
                                                                aFlyWingRight = '%5.3f' % N.pi,
                                                                )
                            iFieldIn += 6
                        else:      # Use 'wing angle' values from file.
                            stateFly = self.templateStateFly_V27.format(
                                                                xFly          = fieldsIn_list[iFieldIn+0],
                                                                yFly          = fieldsIn_list[iFieldIn+1],
                                                                aFly          = fieldsIn_list[iFieldIn+2],
                                                                vxFly         = fieldsIn_list[iFieldIn+3],
                                                                vyFly         = fieldsIn_list[iFieldIn+4],
                                                                vaFly         = fieldsIn_list[iFieldIn+5],
                                                                aFlyWingLeft  = fieldsIn_list[iFieldIn+6],
                                                                aFlyWingRight = fieldsIn_list[iFieldIn+7],
                                                                )
                            iFieldIn += 8
                                
                        state += stateFly

                    ############### End of the line.
                    state += '\n'
                    fidOut.write(state)


    def WriteStateLines_V28(self, filenameIn, filenameOut):
        self.WriteStateLines_V27(filenameIn, filenameOut)


    def WriteStateLines_V281(self, filenameIn, filenameOut):
        self.WriteStateLines_V27(filenameIn, filenameOut)


    def WriteStateLines_V282(self, filenameIn, filenameOut):
        self.WriteStateLines_V27(filenameIn, filenameOut)


    def WriteStateLines_V283(self, filenameIn, filenameOut):
        self.WriteStateLines_V27(filenameIn, filenameOut)


    # ConvertFile()
    # Convert a single file from one version to another.
    #
    def ConvertFile(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        print '%s  ->  %s:  (ver %s -> %s)' % (filenameIn, filenameOut, versionIn, self.versionToWrite)
        #filenameLeaf = os.path.split(filenameIn)[1]
        #filenameOut = dirOut + '/' + filenameLeaf
        self.ReadHeader(filenameIn)
        
        if (self.versionToWrite=='2.2'):
            self.WriteHeader_V22(filenameOut)
            self.WriteStateLines_Vpre26(filenameIn, filenameOut)
            
        elif (self.versionToWrite=='2.6'):
            self.WriteHeader_V26(filenameOut)
            self.WriteStateLines_V26(filenameIn, filenameOut)
        
        elif (self.versionToWrite=='2.7'):
            self.WriteHeader_V27(filenameOut)
            self.WriteStateLines_V27(filenameIn, filenameOut)
        
        elif (self.versionToWrite=='2.8'):
            self.WriteHeader_V28(filenameOut)
            self.WriteStateLines_V28(filenameIn, filenameOut)
        
        elif (self.versionToWrite=='2.81'):
            self.WriteHeader_V281(filenameOut)
            self.WriteStateLines_V281(filenameIn, filenameOut)
        
        elif (self.versionToWrite=='2.82'):
            self.WriteHeader_V282(filenameOut)
            self.WriteStateLines_V282(filenameIn, filenameOut)
        
        elif (self.versionToWrite=='2.83') or (self.versionToWrite=='latest'):
            self.WriteHeader_V283(filenameOut)
            self.WriteStateLines_V283(filenameIn, filenameOut)
        

    # Convert all the .csv files in the input directory to a given version in the output directory.
    #
    def ConvertDirToDir(self, dirInBase, dirOutBase):
        
        if (self.versionToWrite in ['2.2', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83', 'latest']): 
            dirsIn = glob.glob(dirInBase+'/*')
            for d in dirsIn:
                dirLeaf = d.split('/')[-1]
                dirOut = dirOutBase + '/' + dirLeaf
                try:
                    os.makedirs(dirOut)
                except OSError:
                    pass
                
                filenameIn_list = glob.glob(d+'/*.csv')
                for filenameIn in filenameIn_list:
                    self.ConvertFile(filenameIn, dirOut)
                        
        else:
            print ('Only versions "2.2", "2.6", "2.7", "2.8", "2.81", "2.82", "2.83", and "latest" are supported for writing.')
            

    def ConvertTree(self, dirIn, dirOut, whatToCopy):
        if (self.versionToWrite in ['2.2', '2.6', '2.7', '2.8', '2.81', '2.82', '2.83', 'latest']): 
            if (dirIn != dirOut):
                names = os.listdir(dirIn)
            
                try:
                    print 'mkdir %s' % dirOut
                    os.makedirs(dirOut)
                except OSError: # Directory already exists.
                    pass
                
                errors = []
                for name in names:
                    filespecIn = os.path.join(dirIn, name)
                    filespecOut = os.path.join(dirOut, name)
                    try:
                        if os.path.isdir(filespecIn):
                            #print 'ConvertTree:  %s -> %s' % (filespecIn, filespecOut)
                            self.ConvertTree(filespecIn, filespecOut, whatToCopy)
                        else:
                            if (os.path.splitext(filespecIn)[1]=='.csv'):
                                #print 'ConvertFile:  %s -> %s' % (filespecIn, filespecOut)
                                self.ConvertFile(filespecIn, filespecOut)
                            else:
                                if (whatToCopy=='everything'):
                                    print '%s  ->  %s:  (copied)' % (filespecIn, filespecOut)
                                    shutil.copy2(filespecIn, filespecOut)
                                
                    except (IOError, os.error) as why:
                        errors.append((filespecIn, filespecOut, str(why)))
                    # catch the Error from the recursive ConvertTree() so that we can continue with other files
                    #except Error as err:
                    #    errors.extend(err.args[0])
                try:
                    shutil.copystat(dirIn, dirOut)
                except WindowsError:
                    # can't copy file access times on Windows
                    pass
                except OSError as why:
                    errors.extend((dirIn, dirOut, str(why)))
                if errors:
                    raise Error(errors)
            else:
                print "Source and Dest dirs must be different."
        else:
            print ('Only versions "2.2", "2.6", "2.7", "2.8", "2.81", "2.82", "2.83", and "latest" are supported for writing.')
            
        
if __name__ == '__main__':
    convert = ConvertCsv()
    
    
    ###############################################################################################
    ###############################################################################################
    convert.versionToWrite = 'latest'  # '2.2' or '2.6' or '2.7' or '2.8' or '2.81' or '2.82' or '2.83' or 'latest'
    dirIn   = '/home/ssafarik/FlylabData_oldversions'
    dirOut  = '/home/ssafarik/FlylabData_converted'
    
    
    #whatToCopy = 'everything'           # Choose one of these two options.
    whatToCopy = 'csvonly'              # Choose one of these two options.
    
    ###############################################################################################
    ###############################################################################################

    
    convert.ConvertTree(dirIn, dirOut, whatToCopy)
    
    
            
        
