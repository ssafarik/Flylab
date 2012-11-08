#!/usr/bin/env python
from __future__ import division
import sys
import os
import glob
import shutil


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
        self.headingsExperiment_V22 =  'date_time, '\
                                        'description, '\
                                        'maxTrials, '\
                                        'trial\n'
        self.templateExperiment_V22 =  '{date_time:s}, '\
                                        '{description:s}, '\
                                        '{maxTrials:s}, '\
                                        '{trial:s}\n'
                                  
        self.headingsRobot_V22 =       'nRobots, '\
                                        'widthRobot, '\
                                        'heightRobot, '\
                                        'visibleRobot, '\
                                        'paintRobot, '\
                                        'scentRobot\n'
        self.templateRobots_V22 =       '{nRobots:s}, '\
                                        '{widthRobot:s}, '\
                                        '{heightRobot:s}, '\
                                        '{visibleRobot:s}, '\
                                        '{paintRobot:s}, '\
                                        '{scentRobot:s}\n'
                                  
        self.headingsFlies_V22 =        'nFlies, '\
                                        'typeFlies, '\
                                        'genderFlies\n'
        self.templateFlies_V22 =        '{nFlies:s}, '\
                                        '{typeFlies:s}, '\
                                        '{genderFlies:s}\n'

        self.headingsTrackingA_V22 =    'trackingExclusionzoneEnabled'
        self.headingsTrackingB_V22 =    ', trackingExclusionzoneX, '\
                                        'trackingExclusionzoneY, '\
                                        'trackingExclusionzoneRadius'
        self.templateTrackingA_V22 =    '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB_V22 =    ', {trackingExclusionzoneX:s}, '\
                                        '{trackingExclusionzoneY:s}, '\
                                        '{trackingExclusionzoneRadius:s}'

        self.headingsPreWait1_V22 =     'waitEntry\n'
        self.templatePreWait1_V22 =     '{preWait1:s}\n'
        
        self.headingsPreTrigger_V22 =   'trigger1Enabled, '\
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
                                    
        self.headingsTrialRobot_V22 =   'moverobotEnabled, '\
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
                                    
        self.headingsTrialLaser_V22 =   'laserEnabled, '\
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
        self.templateTrialLaser_V22 =  '{trialLaserEnabled:s}, '\
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
        
        self.headingsPostTrigger_V22 =  'trigger2Enabled, '\
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
        self.templatePostTrigger_V22 = '{postTriggerEnabled:s}, '\
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
                                  
        self.headingsPostWait_V22 =    'waitExit\n'
        self.templatePostWait_V22 =    '{postWait:s}\n'


        #######################################################################
        self.headingsVersionFile_V26 = 'versionFile\n'
        self.templateVersionFile_V26 = '{versionFile:s}\n'
        
        self.headingsExperiment_V26 =  'date_time, '\
                                        'description, '\
                                        'maxTrials, '\
                                        'trial\n'
        self.templateExperiment_V26 =  '{date_time:s}, '\
                                        '{description:s}, '\
                                        '{maxTrials:s}, '\
                                        '{trial:s}\n'
                                  
        self.headingsRobot_V26 =       'nRobots, '\
                                        'widthRobot, '\
                                        'heightRobot, '\
                                        'visibleRobot, '\
                                        'paintRobot, '\
                                        'scentRobot\n'
        self.templateRobots_V26 =       '{nRobots:s}, '\
                                        '{widthRobot:s}, '\
                                        '{heightRobot:s}, '\
                                        '{visibleRobot:s}, '\
                                        '{paintRobot:s}, '\
                                        '{scentRobot:s}\n'
                                  
        self.headingsFlies_V26 =       'nFlies, '\
                                        'typeFlies, '\
                                        'genderFlies\n'
        self.templateFlies_V26 =       '{nFlies:s}, '\
                                        '{typeFlies:s}, '\
                                        '{genderFlies:s}\n'

        self.headingsTrackingA_V26 =   'trackingExclusionzoneEnabled'
        self.headingsTrackingB_V26 =   ', trackingExclusionzoneX, '\
                                        'trackingExclusionzoneY, '\
                                        'trackingExclusionzoneRadius'
        self.templateTrackingA_V26 =   '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB_V26 =   ', {trackingExclusionzoneX:s}, '\
                                        '{trackingExclusionzoneY:s}, '\
                                        '{trackingExclusionzoneRadius:s}'

        self.headingsPreRobot_V26 =    'preRobotEnabled, '\
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
                                        'preRobotMoveRelTolerance\n'
        self.templatePreRobot_V26 =    '{preRobotEnabled:s}, '\
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
                                        '{preRobotMoveRelTolerance:s}\n'
                                    
        self.headingsPreLaser_V26 =    'preLaserEnabled, '\
                                        'preLaserPatternShape, '\
                                        'preLaserPatternHzPattern, '\
                                        'preLaserPatternHzPoint, '\
                                        'preLaserPatternCount, '\
                                        'preLaserPatternSizeX, '\
                                        'preLaserPatternSizeY, '\
                                        'preLaserPatternParam, '\
                                        'preLaserStatefilterLo, '\
                                        'preLaserStatefilterHi, '\
                                        'preLaserStatefilterCriteria\n'
        self.templatePreLaser_V26 =    '{preLaserEnabled:s}, '\
                                        '{preLaserPatternShape:s}, '\
                                        '{preLaserPatternHzPattern:s}, '\
                                        '{preLaserPatternHzPoint:s}, '\
                                        '{preLaserPatternCount:s}, '\
                                        '{preLaserPatternSizeX:s}, '\
                                        '{preLaserPatternSizeY:s}, '\
                                        '{preLaserPatternParam:s}, '\
                                        '\"{preLaserStatefilterLo:s}\", '\
                                        '\"{preLaserStatefilterHi:s}\", '\
                                        '{preLaserStatefilterCriteria:s}\n'
        
        self.headingsPreLEDPanels_V26 ='preLEDPanelsEnabled, '\
                                        'preLEDPanelsCommand, '\
                                        'preLEDPanelsIdPattern, '\
                                        'preLEDPanelsFrameid, '\
                                        'preLEDPanelsStatefilterLo, '\
                                        'preLEDPanelsStatefilterHi, '\
                                        'preLEDPanelsStatefilterCriteria\n'
        self.templatePreLEDPanels_V26 ='{preLEDPanelsEnabled:s}, '\
                                        '{preLEDPanelsCommand:s}, '\
                                        '{preLEDPanelsIdPattern:s}, '\
                                        '{preLEDPanelsFrameid:s}, '\
                                        '{preLEDPanelsStatefilterLo:s}, '\
                                        '{preLEDPanelsStatefilterHi:s}, '\
                                        '{preLEDPanelsStatefilterCriteria:s}\n'

        self.headingsPreWait1_V26 =    'preWait1\n'
        self.templatePreWait1_V26 =    '{preWait1:s}\n'
        
        self.headingsPreTrigger_V26 =  'preTriggerEnabled, '\
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
        self.templatePreTrigger_V26 =  '{preTriggerEnabled:s}, '\
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
                                    
        self.headingsPreWait2_V26 =    'preWait2\n'
        self.templatePreWait2_V26 =    '{preWait2:s}\n'
        
        self.headingsTrialRobot_V26 =  'trialRobotEnabled, '\
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
                                        'trialRobotMoveRelTolerance\n'
        self.templateTrialRobot_V26 =  '{trialRobotEnabled:s}, '\
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
                                    
        self.headingsTrialLaser_V26 =  'trialLaserEnabled, '\
                                        'trialLaserPatternShape, '\
                                        'trialLaserPatternHzPattern, '\
                                        'trialLaserPatternHzPoint, '\
                                        'trialLaserPatternCount, '\
                                        'trialLaserPatternSizeX, '\
                                        'trialLaserPatternSizeY, '\
                                        'trialLaserPatternParam, '\
                                        'trialLaserStatefilterLo, '\
                                        'trialLaserStatefilterHi, '\
                                        'trialLaserStatefilterCriteria\n'
        self.templateTrialLaser_V26 =  '{trialLaserEnabled:s}, '\
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
        
        self.headingsTrialLEDPanels_V26 = 'trialLEDPanelsEnabled, '\
                                        'trialLEDPanelsCommand, '\
                                        'trialLEDPanelsIdPattern, '\
                                        'trialLEDPanelsFrameid, '\
                                        'trialLEDPanelsStatefilterLo, '\
                                        'trialLEDPanelsStatefilterHi, '\
                                        'trialLEDPanelsStatefilterCriteria\n'
        self.templateTrialLEDPanels_V26 = '{trialLEDPanelsEnabled:s}, '\
                                        '{trialLEDPanelsCommand:s}, '\
                                        '{trialLEDPanelsIdPattern:s}, '\
                                        '{trialLEDPanelsFrameid:s}, '\
                                        '{trialLEDPanelsStatefilterLo:s}, '\
                                        '{trialLEDPanelsStatefilterHi:s}, '\
                                        '{trialLEDPanelsStatefilterCriteria:s}\n'
        
        self.headingsPostTrigger_V26 = 'postTriggerEnabled, '\
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
        self.templatePostTrigger_V26 = '{postTriggerEnabled:s}, '\
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
                                  
        self.headingsPostWait_V26 =    'postWait\n'
        self.templatePostWait_V26 =    '{postWait:s}\n'


        #######################################################################
        self.headingsDataLeft_Pre26     = 'time'
        self.templateDataLeft_Pre26     = '{time:s}'
        self.headingsDataRobot_Pre26    = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot'
        self.templateDataRobot_Pre26    = ', {xRobot:s}, {yRobot:s}, {aRobot:s}, {vxRobot:s}, {vyRobot:s}, {vaRobot:s}'
        self.headingsDataFly_Pre26      = ', xFly, yFly, aFly, vxFly, vyFly, vaFly'
        self.templateDataFly_Pre26      = ', {xFly:s}, {yFly:s}, {aFly:s}, {vxFly:s}, {vyFly:s}, {vaFly:s}'

        self.headingsDataLeft_V26       = 'time, triggered'
        self.templateDataLeft_V26       = '{time:s}, {triggered:s}'
        self.headingsDataRobot_V26      = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot'
        self.templateDataRobot_V26      = ', {xRobot:s}, {yRobot:s}, {aRobot:s}, {vxRobot:s}, {vyRobot:s}, {vaRobot:s}'
        self.headingsDataFly_V26        = ', xFly, yFly, aFly, vxFly, vyFly, vaFly'
        self.templateDataFly_V26        = ', {xFly:s}, {yFly:s}, {aFly:s}, {vxFly:s}, {vyFly:s}, {vaFly:s}'
        
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
                
            # Read the data header line.
            lineHeaderData = fid.readline()
            
            
            # Determine the version from which headings are on which lines, or from the version number.
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
            nFields = len(lineHeaderData.split(','))
            nFields -= 1  # For the 'time' field.
                        
            field = lineHeaderData.split(',')[1].strip(' \n')
            if field=='triggered':
                nFields -= 1  # For the 'triggered' field.

            # Now nFields should be a multiple of 6:  (x,y,a,vx,vy,va) for each robot or fly.
            nObjects = int(nFields/6)
            
            nRobots = 1 # There's always a robot in the file.
            nFlies = nObjects-nRobots
            
                        
        return (version, nLinesHeader, nRobots, nFlies)


    # Given a comma separated line of text, return a list of cleaned-up fields.
    def ListFromCsv (self, csv):
        field_list = csv.split(',')
        for i in range(len(field_list)):
            field_list[i] = field_list[i].strip(' \n\"')
            
        return field_list
                 

    def ReadHeader_V1 (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        #self.nHeader = 4
        
        with open(filename, 'r') as fid:
            self.AdvanceUntilField(fid, 'date_time')
            headerTxt = fid.readline()
            header = fid.readline()

        fieldPre_list = header.split(',')
        field_list = []
        for field in fieldPre_list:
            field_list.append(field.strip(' \n'))
            
            
        # Set default values for PreRobot.
        self.param_preRobotEnabled             = 'False'
        self.param_preRobotMovePatternShape        = ''
        self.param_preRobotMovePatternHzPattern    = '0'
        self.param_preRobotMovePatternHzPoint      = '0'
        self.param_preRobotMovePatternCount        = '0'
        self.param_preRobotMovePatternSizeX        = '0'
        self.param_preRobotMovePatternSizeY        = '0'
        self.param_preRobotMovePatternParam        = '0'
        self.param_preRobotMoveRelTracking         = 'False'
        self.param_preRobotMoveRelOriginPosition   = ''
        self.param_preRobotMoveRelOriginAngle      = ''
        self.param_preRobotMoveRelDistance         = '0'
        self.param_preRobotMoveRelAngle            = '0'
        self.param_preRobotMoveRelAngleType        = ''
        self.param_preRobotMoveRelSpeed            = '0'
        self.param_preRobotMoveRelSpeedType        = ''
        self.param_preRobotMoveRelTolerance        = '0'

        # Set default values for PreLaser.
        self.param_preLaserEnabled                 = 'False'
        self.param_preLaserPatternShape            = ''
        self.param_preLaserPatternHzPattern        = '0'
        self.param_preLaserPatternHzPoint          = '0'
        self.param_preLaserPatternCount            = '0'
        self.param_preLaserPatternSizeX            = '0'
        self.param_preLaserPatternSizeY            = '0'
        self.param_preLaserPatternParam            = '0'
        self.param_preLaserStatefilterLo           = ''
        self.param_preLaserStatefilterHi           = ''
        self.param_preLaserStatefilterCriteria     = ''
        
        # Set default values for PreLEDPanels.
        self.param_preLEDPanelsEnabled              = 'False'
        self.param_preLEDPanelsCommand              = ''
        self.param_preLEDPanelsIdPattern            = '0'
        self.param_preLEDPanelsFrameid              = ''
        self.param_preLEDPanelsStatefilterLo        = ''
        self.param_preLEDPanelsStatefilterHi        = ''
        self.param_preLEDPanelsStatefilterCriteria  = ''
    
        self.param_trackingExclusionzoneEnabled     = 'False'
        self.param_trackingExclusionzoneX_list      = []
        self.param_trackingExclusionzoneY_list      = []
        self.param_trackingExclusionzoneRadius_list = []

        # Set default values for TrialLEDPanels.
        self.param_trialLEDPanelsEnabled              = 'False'
        self.param_trialLEDPanelsCommand              = ''
        self.param_trialLEDPanelsIdPattern            = '0'
        self.param_trialLEDPanelsFrameid              = ''
        self.param_trialLEDPanelsStatefilterLo        = ''
        self.param_trialLEDPanelsStatefilterHi        = ''
        self.param_trialLEDPanelsStatefilterCriteria  = ''


        if versionIn=='1.0': 
            self.param_date_time = field_list[0]
            self.param_description = field_list[1]
            self.param_maxTrials = field_list[2]
            self.param_trial = field_list[3]
            
            self.param_nFlies = 'unspecified'
            self.param_typeFlies = 'unspecified'
            self.param_genderFlies = 'unspecified'
            
            self.param_trackingExclusionzoneEnabled = 'False'
            self.param_trackingExclusionzoneX_list = []
            self.param_trackingExclusionzoneY_list = []
            self.param_trackingExclusionzoneRadius_list = []

            self.param_preWait1 = field_list[4]
            
            self.param_preTriggerEnabled = 'unspecified'
            self.param_preTriggerFrameidParent = 'unspecified'
            self.param_preTriggerFrameidChild = 'unspecified'
            self.param_preTriggerDistanceMin = field_list[5]
            self.param_preTriggerDistanceMax = field_list[6]
            self.param_preTriggerSpeedAbsParentMin = 'unspecified'
            self.param_preTriggerSpeedAbsParentMax = 'unspecified'
            self.param_preTriggerSpeedAbsChildMin = 'unspecified'
            self.param_preTriggerSpeedAbsChildMax = 'unspecified'
            self.param_preTriggerSpeedRelMin = field_list[7]
            self.param_preTriggerSpeedRelMax = field_list[8]
            self.param_preTriggerAngleMin = field_list[9]
            self.param_preTriggerAngleMax = field_list[10]
            self.param_preTriggerAngleTest = field_list[11]
            self.param_preTriggerAngleTestBilateral = field_list[12]
            self.param_preTriggerTimeHold = field_list[13]
            self.param_preTriggerTimeout = 'unspecified'

            self.param_preWait2 = '0.0'

            self.param_postTriggerEnabled = 'unspecified'
            self.param_postTriggerFrameidParent = 'unspecified'
            self.param_postTriggerFrameidChild = 'unspecified'
            self.param_postTriggerDistanceMin = field_list[14]
            self.param_postTriggerDistanceMax = field_list[15]
            self.param_postTriggerSpeedAbsParentMin = 'unspecified'
            self.param_postTriggerSpeedAbsParentMax = 'unspecified'
            self.param_postTriggerSpeedAbsChildMin = 'unspecified'
            self.param_postTriggerSpeedAbsChildMax = 'unspecified'
            self.param_postTriggerSpeedRelMin = field_list[16]
            self.param_postTriggerSpeedRelMax = field_list[17]
            self.param_postTriggerAngleMin = field_list[18]
            self.param_postTriggerAngleMax = field_list[19]
            self.param_postTriggerAngleTest = field_list[20]
            self.param_postTriggerAngleTestBilateral = field_list[21]
            self.param_postTriggerTimeHold = field_list[22]
            self.param_postTriggerTimeout = 'unspecified'

            self.param_nRobots = 'unspecified'
            self.param_widthRobot = field_list[23]
            self.param_heightRobot = field_list[24]
            self.param_visibleRobot = field_list[25]
            self.param_paintRobot = field_list[26]
            self.param_scentRobot = field_list[27]

            self.param_trialRobotEnabled = 'unspecified'
            self.param_trialRobotMovePatternShape = field_list[28]
            self.param_trialRobotMovePatternHzPattern = field_list[29]
            self.param_trialRobotMovePatternHzPoint = field_list[30]
            self.param_trialRobotMovePatternCount = field_list[31]
            self.param_trialRobotMovePatternSizeX = field_list[32]
            self.param_trialRobotMovePatternSizeY = field_list[33]
            self.param_trialRobotMovePatternParam = '0.0'
            self.param_trialRobotMoveRelTracking = field_list[34]
            self.param_trialRobotMoveRelOriginPosition = field_list[35]
            self.param_trialRobotMoveRelOriginAngle = field_list[36]
            self.param_trialRobotMoveRelDistance = field_list[37]
            self.param_trialRobotMoveRelAngle = field_list[38]
            self.param_trialRobotMoveRelAngleType = field_list[39]
            self.param_trialRobotMoveRelSpeed = field_list[40]
            self.param_trialRobotMoveRelSpeedType = field_list[41]
            self.param_trialRobotMoveRelTolerance = field_list[42]
    
            self.param_trialLaserEnabled = 'False'
            self.param_trialLaserPatternShape = 'unspecified'
            self.param_trialLaserPatternHzPattern = 'unspecified'
            self.param_trialLaserPatternHzPoint = 'unspecified'
            self.param_trialLaserPatternCount = 'unspecified'
            self.param_trialLaserPatternSizeX = 'unspecified'
            self.param_trialLaserPatternSizeY = 'unspecified'
            self.param_trialLaserPatternParam = 'unspecified'
            self.param_trialLaserStatefilterLo = 'unspecified'
            self.param_trialLaserStatefilterHi = 'unspecified'
            self.param_trialLaserStatefilterCriteria = 'unspecified'
        
            self.param_postWait = '0.0'


        if versionIn=='1.1': 
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            self.param_nRobots                      = 'unspecified'
            self.param_widthRobot                   = field_list[4]
            self.param_heightRobot                  = field_list[5]
            self.param_visibleRobot                 = field_list[6]
            self.param_paintRobot                   = field_list[7]
            self.param_scentRobot                   = field_list[8]

            self.param_nFlies                       = 'unspecified'
            self.param_typeFlies                    = 'unspecified'
            self.param_genderFlies                  = 'unspecified'
            
            self.param_trackingExclusionzoneEnabled     = 'False'
            self.param_trackingExclusionzoneX_list      = []
            self.param_trackingExclusionzoneY_list      = []
            self.param_trackingExclusionzoneRadius_list = []

            self.param_preWait1                    = field_list[9]
            
            self.param_preTriggerEnabled              = 'unspecified'
            self.param_preTriggerFrameidParent        = field_list[10]
            self.param_preTriggerFrameidChild         = field_list[11]
            self.param_preTriggerSpeedAbsParentMin    = field_list[12]
            self.param_preTriggerSpeedAbsParentMax    = field_list[13]
            self.param_preTriggerSpeedAbsChildMin     = field_list[14]
            self.param_preTriggerSpeedAbsChildMax     = field_list[15]
            self.param_preTriggerSpeedRelMin          = 'unspecified'
            self.param_preTriggerSpeedRelMax          = 'unspecified'
            self.param_preTriggerDistanceMin          = field_list[16]
            self.param_preTriggerDistanceMax          = field_list[17]
            self.param_preTriggerAngleMin             = field_list[18]
            self.param_preTriggerAngleMax             = field_list[19]
            self.param_preTriggerAngleTest            = field_list[20]
            self.param_preTriggerAngleTestBilateral   = field_list[21]
            self.param_preTriggerTimeHold             = field_list[22]
            self.param_preTriggerTimeout              = 'unspecified'

            self.param_preWait2                       = '0.0'

            self.param_trialRobotEnabled             = 'unspecified'
            self.param_trialRobotMovePatternShape        = field_list[23]
            self.param_trialRobotMovePatternHzPattern    = field_list[24]
            self.param_trialRobotMovePatternHzPoint      = field_list[25]
            self.param_trialRobotMovePatternCount        = field_list[26]
            self.param_trialRobotMovePatternSizeX        = field_list[27]
            self.param_trialRobotMovePatternSizeY        = field_list[28]
            self.param_trialRobotMovePatternParam        = '0.0'
            self.param_trialRobotMoveRelTracking         = field_list[29]
            self.param_trialRobotMoveRelOriginPosition   = field_list[30]
            self.param_trialRobotMoveRelOriginAngle      = field_list[31]
            self.param_trialRobotMoveRelDistance         = field_list[32]
            self.param_trialRobotMoveRelAngle            = field_list[33]
            self.param_trialRobotMoveRelAngleType        = field_list[34]
            self.param_trialRobotMoveRelSpeed            = field_list[35]
            self.param_trialRobotMoveRelSpeedType        = field_list[36]
            self.param_trialRobotMoveRelTolerance        = field_list[37]
    
            self.param_trialLaserEnabled                 = field_list[38]
            self.param_trialLaserPatternShape            = 'unspecified'
            self.param_trialLaserPatternHzPattern        = 'unspecified'
            self.param_trialLaserPatternHzPoint          = 'unspecified'
            self.param_trialLaserPatternCount            = 'unspecified'
            self.param_trialLaserPatternSizeX            = 'unspecified'
            self.param_trialLaserPatternSizeY            = 'unspecified'
            self.param_trialLaserPatternParam            = 'unspecified'
            self.param_trialLaserStatefilterLo           = 'unspecified'
            self.param_trialLaserStatefilterHi           = 'unspecified'
            self.param_trialLaserStatefilterCriteria     = 'unspecified'
        
            self.param_postTriggerEnabled              = 'unspecified'
            self.param_postTriggerFrameidParent        = field_list[39]
            self.param_postTriggerFrameidChild         = field_list[40]
            self.param_postTriggerSpeedAbsParentMin    = field_list[41]
            self.param_postTriggerSpeedAbsParentMax    = field_list[42]
            self.param_postTriggerSpeedAbsChildMin     = field_list[43]
            self.param_postTriggerSpeedAbsChildMax     = field_list[44]
            self.param_postTriggerSpeedRelMin          = 'unspecified'
            self.param_postTriggerSpeedRelMax          = 'unspecified'
            self.param_postTriggerDistanceMin          = field_list[45]
            self.param_postTriggerDistanceMax          = field_list[46]
            self.param_postTriggerAngleMin             = field_list[47]
            self.param_postTriggerAngleMax             = field_list[48]
            self.param_postTriggerAngleTest            = field_list[49]
            self.param_postTriggerAngleTestBilateral   = field_list[50]
            self.param_postTriggerTimeHold             = field_list[51]
            self.param_postTriggerTimeout              = 'unspecified'

            self.param_postWait = '0.0'


    def ReadHeader_V2 (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        #self.nHeader = 37
        
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
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostWaitTxt   = line[i]; i=i+1
                headerPostWait      = line[i]; i=i+1
                blank               = line[i]; i=i+1


            if versionIn=='2.1':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerTrialRobotTxt      = line[i]; i=i+1
                headerTrialRobot         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt      = line[i]; i=i+1
                headerTrialLaser         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerPostTriggerTxt = line[i]; i=i+1
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
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerTrialRobotTxt      = line[i]; i=i+1
                headerTrialRobot         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt      = line[i]; i=i+1
                headerTrialLaser         = line[i]; i=i+1
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


            if versionIn=='2.4':
                i = 0;
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerTrialRobotTxt      = line[i]; i=i+1
                headerTrialRobot         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt      = line[i]; i=i+1
                headerTrialLaser         = line[i]; i=i+1
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
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerTrialRobotTxt      = line[i]; i=i+1
                headerTrialRobot         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt      = line[i]; i=i+1
                headerTrialLaser         = line[i]; i=i+1
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


            if versionIn=='2.6':
                i = 0;
                headerVersionTxt    = line[i]; i=i+1
                headerVersion       = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerExperimentTxt = line[i]; i=i+1
                headerExperiment    = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerRobotsTxt     = line[i]; i=i+1
                headerRobots        = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerFliesTxt      = line[i]; i=i+1
                headerFlies         = line[i]; i=i+1
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
                
                headerTrialRobotTxt      = line[i]; i=i+1
                headerTrialRobot         = line[i]; i=i+1
                blank               = line[i]; i=i+1
                
                headerTrialLaserTxt      = line[i]; i=i+1
                headerTrialLaser         = line[i]; i=i+1
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


        
        # Set default values for PreRobot.
        self.param_preRobotEnabled             = 'False'
        self.param_preRobotMovePatternShape        = ''
        self.param_preRobotMovePatternHzPattern    = '0'
        self.param_preRobotMovePatternHzPoint      = '0'
        self.param_preRobotMovePatternCount        = '0'
        self.param_preRobotMovePatternSizeX        = '0'
        self.param_preRobotMovePatternSizeY        = '0'
        self.param_preRobotMovePatternParam        = '0'
        self.param_preRobotMoveRelTracking         = 'False'
        self.param_preRobotMoveRelOriginPosition   = ''
        self.param_preRobotMoveRelOriginAngle      = ''
        self.param_preRobotMoveRelDistance         = '0'
        self.param_preRobotMoveRelAngle            = '0'
        self.param_preRobotMoveRelAngleType        = ''
        self.param_preRobotMoveRelSpeed            = '0'
        self.param_preRobotMoveRelSpeedType        = ''
        self.param_preRobotMoveRelTolerance        = '0'

        # Set default values for PreLaser.
        self.param_preLaserEnabled                 = 'False'
        self.param_preLaserPatternShape            = ''
        self.param_preLaserPatternHzPattern        = '0'
        self.param_preLaserPatternHzPoint          = '0'
        self.param_preLaserPatternCount            = '0'
        self.param_preLaserPatternSizeX            = '0'
        self.param_preLaserPatternSizeY            = '0'
        self.param_preLaserPatternParam            = '0'
        self.param_preLaserStatefilterLo           = ''
        self.param_preLaserStatefilterHi           = ''
        self.param_preLaserStatefilterCriteria     = ''
        
        # Set default values for PreLEDPanels.
        self.param_preLEDPanelsEnabled              = 'False'
        self.param_preLEDPanelsCommand              = ''
        self.param_preLEDPanelsIdPattern            = '0'
        self.param_preLEDPanelsFrameid              = ''
        self.param_preLEDPanelsStatefilterLo        = ''
        self.param_preLEDPanelsStatefilterHi        = ''
        self.param_preLEDPanelsStatefilterCriteria  = ''
    
        self.param_trackingExclusionzoneEnabled     = 'False'
        self.param_trackingExclusionzoneX_list      = []
        self.param_trackingExclusionzoneY_list      = []
        self.param_trackingExclusionzoneRadius_list = []

        # Set default values for TrialLEDPanels.
        self.param_trialLEDPanelsEnabled              = 'False'
        self.param_trialLEDPanelsCommand              = ''
        self.param_trialLEDPanelsIdPattern            = '0'
        self.param_trialLEDPanelsFrameid              = ''
        self.param_trialLEDPanelsStatefilterLo        = ''
        self.param_trialLEDPanelsStatefilterHi        = ''
        self.param_trialLEDPanelsStatefilterCriteria  = ''
    

        if versionIn=='2.0':
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = 'unspecified'
            self.param_genderFlies                  = 'unspecified'
            
            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
            field_list = self.ListFromCsv(headerPreTrigger)
            self.param_preTriggerEnabled              = field_list[0]
            self.param_preTriggerFrameidParent        = field_list[1]
            self.param_preTriggerFrameidChild         = field_list[2]
            self.param_preTriggerSpeedAbsParentMin    = field_list[3]
            self.param_preTriggerSpeedAbsParentMax    = field_list[4]
            self.param_preTriggerSpeedAbsChildMin     = field_list[5]
            self.param_preTriggerSpeedAbsChildMax     = field_list[6]
            self.param_preTriggerSpeedRelMin          = 'unspecified'
            self.param_preTriggerSpeedRelMax          = 'unspecified'
            self.param_preTriggerDistanceMin          = field_list[7]
            self.param_preTriggerDistanceMax          = field_list[8]
            self.param_preTriggerAngleMin             = field_list[9]
            self.param_preTriggerAngleMax             = field_list[10]
            self.param_preTriggerAngleTest            = field_list[11]
            self.param_preTriggerAngleTestBilateral   = field_list[12]
            self.param_preTriggerTimeHold             = field_list[13]
            self.param_preTriggerTimeout              = field_list[14]

            self.param_preWait2                         = '0.0'

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
            self.param_trialRobotMovePatternShape        = field_list[1]
            self.param_trialRobotMovePatternHzPattern    = field_list[2]
            self.param_trialRobotMovePatternHzPoint      = field_list[3]
            self.param_trialRobotMovePatternCount        = field_list[4]
            self.param_trialRobotMovePatternSizeX        = field_list[5]
            self.param_trialRobotMovePatternSizeY        = field_list[6]
            self.param_trialRobotMovePatternParam        = '0.0'
            self.param_trialRobotMoveRelTracking         = field_list[7]
            self.param_trialRobotMoveRelOriginPosition   = field_list[8]
            self.param_trialRobotMoveRelOriginAngle      = field_list[9]
            self.param_trialRobotMoveRelDistance         = field_list[10]
            self.param_trialRobotMoveRelAngle            = field_list[11]
            self.param_trialRobotMoveRelAngleType        = field_list[12]
            self.param_trialRobotMoveRelSpeed            = field_list[13]
            self.param_trialRobotMoveRelSpeedType        = field_list[14]
            self.param_trialRobotMoveRelTolerance        = field_list[15]
    
            field_list = self.ListFromCsv(headerTrialLaser)
            self.param_trialLaserEnabled                 = field_list[0]
            self.param_trialLaserPatternShape            = 'unspecified'
            self.param_trialLaserPatternHzPattern        = 'unspecified'
            self.param_trialLaserPatternHzPoint          = 'unspecified'
            self.param_trialLaserPatternCount            = 'unspecified'
            self.param_trialLaserPatternSizeX            = 'unspecified'
            self.param_trialLaserPatternSizeY            = 'unspecified'
            self.param_trialLaserPatternParam            = 'unspecified'
            self.param_trialLaserStatefilterLo           = 'unspecified'
            self.param_trialLaserStatefilterHi           = 'unspecified'
            self.param_trialLaserStatefilterCriteria     = 'unspecified'
        
            field_list = self.ListFromCsv(headerPostTrigger)
            self.param_postTriggerEnabled              = field_list[0]
            self.param_postTriggerFrameidParent        = field_list[1]
            self.param_postTriggerFrameidChild         = field_list[2]
            self.param_postTriggerSpeedAbsParentMin    = field_list[3]
            self.param_postTriggerSpeedAbsParentMax    = field_list[4]
            self.param_postTriggerSpeedAbsChildMin     = field_list[5]
            self.param_postTriggerSpeedAbsChildMax     = field_list[6]
            self.param_postTriggerSpeedRelMin          = 'unspecified'
            self.param_postTriggerSpeedRelMax          = 'unspecified'
            self.param_postTriggerDistanceMin          = field_list[7]
            self.param_postTriggerDistanceMax          = field_list[8]
            self.param_postTriggerAngleMin             = field_list[9]
            self.param_postTriggerAngleMax             = field_list[10]
            self.param_postTriggerAngleTest            = field_list[11]
            self.param_postTriggerAngleTestBilateral   = field_list[12]
            self.param_postTriggerTimeHold             = field_list[13]
            self.param_postTriggerTimeout              = field_list[14]

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


        if versionIn=='2.1':
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            
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

            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
            field_list = self.ListFromCsv(headerPreTrigger)
            self.param_preTriggerEnabled              = field_list[0]
            self.param_preTriggerFrameidParent        = field_list[1]
            self.param_preTriggerFrameidChild         = field_list[2]
            self.param_preTriggerSpeedAbsParentMin    = field_list[3]
            self.param_preTriggerSpeedAbsParentMax    = field_list[4]
            self.param_preTriggerSpeedAbsChildMin     = field_list[5]
            self.param_preTriggerSpeedAbsChildMax     = field_list[6]
            self.param_preTriggerSpeedRelMin          = 'unspecified'
            self.param_preTriggerSpeedRelMax          = 'unspecified'
            self.param_preTriggerDistanceMin          = field_list[7]
            self.param_preTriggerDistanceMax          = field_list[8]
            self.param_preTriggerAngleMin             = field_list[9]
            self.param_preTriggerAngleMax             = field_list[10]
            self.param_preTriggerAngleTest            = field_list[11]
            self.param_preTriggerAngleTestBilateral   = field_list[12]
            self.param_preTriggerTimeHold             = field_list[13]
            self.param_preTriggerTimeout              = field_list[14]

            self.param_preWait2                         = '0.0'

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
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
            self.param_trialRobotMoveRelAngle            = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
    
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
        
            field_list = self.ListFromCsv(headerPostTrigger)
            self.param_postTriggerEnabled              = field_list[0]
            self.param_postTriggerFrameidParent        = field_list[1]
            self.param_postTriggerFrameidChild         = field_list[2]
            self.param_postTriggerSpeedAbsParentMin    = field_list[3]
            self.param_postTriggerSpeedAbsParentMax    = field_list[4]
            self.param_postTriggerSpeedAbsChildMin     = field_list[5]
            self.param_postTriggerSpeedAbsChildMax     = field_list[6]
            self.param_postTriggerSpeedRelMin          = 'unspecified'
            self.param_postTriggerSpeedRelMax          = 'unspecified'
            self.param_postTriggerDistanceMin          = field_list[7]
            self.param_postTriggerDistanceMax          = field_list[8]
            self.param_postTriggerAngleMin             = field_list[9]
            self.param_postTriggerAngleMax             = field_list[10]
            self.param_postTriggerAngleTest            = field_list[11]
            self.param_postTriggerAngleTestBilateral   = field_list[12]
            self.param_postTriggerTimeHold             = field_list[13]
            self.param_postTriggerTimeout              = field_list[14]

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


        if versionIn=='2.2':
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            
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

            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
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

            self.param_preWait2                         = '0.0'

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
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
            self.param_trialRobotMoveRelAngle            = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
    
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

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


        if versionIn=='2.3':
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            
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

            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
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

            self.param_preWait2                         = '0.0'

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
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
            self.param_trialRobotMoveRelAngle            = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
    
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
        
            field_list = self.ListFromCsv(headerTrialLEDPanels)
            self.param_trialLEDPanelsEnabled              = field_list[0]
            self.param_trialLEDPanelsCommand              = field_list[1]
            self.param_trialLEDPanelsIdPattern            = field_list[2]
            self.param_trialLEDPanelsFrameid              = field_list[3]
            self.param_trialLEDPanelsStatefilterLo        = field_list[4]
            self.param_trialLEDPanelsStatefilterHi        = field_list[5]
            self.param_trialLEDPanelsStatefilterCriteria  = field_list[6]

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

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


        if versionIn=='2.4' or versionIn=='2.5':    # Just the order of two sections was changed: trialLEDPanels & postTrigger.
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            
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

            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
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

            field_list = self.ListFromCsv(headerPreWait2)
            self.param_preWait2                    = field_list[0]

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
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
            self.param_trialRobotMoveRelAngle            = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
    
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
        
            field_list = self.ListFromCsv(headerTrialLEDPanels)
            self.param_trialLEDPanelsEnabled              = field_list[0]
            self.param_trialLEDPanelsCommand              = field_list[1]
            self.param_trialLEDPanelsIdPattern            = field_list[2]
            self.param_trialLEDPanelsFrameid              = field_list[3]
            self.param_trialLEDPanelsStatefilterLo        = field_list[4]
            self.param_trialLEDPanelsStatefilterHi        = field_list[5]
            self.param_trialLEDPanelsStatefilterCriteria  = field_list[6]

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

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


        if versionIn=='2.6':
            field_list = self.ListFromCsv(headerExperiment)
            self.param_date_time                    = field_list[0]
            self.param_description                  = field_list[1]
            self.param_maxTrials                    = field_list[2]
            self.param_trial                        = field_list[3]
            
            field_list = self.ListFromCsv(headerRobots)
            self.param_nRobots                      = field_list[0]
            self.param_widthRobot                   = field_list[1]
            self.param_heightRobot                  = field_list[2]
            self.param_visibleRobot                 = field_list[3]
            self.param_paintRobot                   = field_list[4]
            self.param_scentRobot                   = field_list[5]

            field_list = self.ListFromCsv(headerFlies)
            self.param_nFlies                       = field_list[0]
            self.param_typeFlies                    = field_list[1]
            self.param_genderFlies                  = field_list[2]
            
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
            self.param_preRobotMoveRelAngle            = field_list[12]
            self.param_preRobotMoveRelAngleType        = field_list[13]
            self.param_preRobotMoveRelSpeed            = field_list[14]
            self.param_preRobotMoveRelSpeedType        = field_list[15]
            self.param_preRobotMoveRelTolerance        = field_list[16]
    
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
        
            field_list = self.ListFromCsv(headerPreLEDPanels)
            self.param_preLEDPanelsEnabled              = field_list[0]
            self.param_preLEDPanelsCommand              = field_list[1]
            self.param_preLEDPanelsIdPattern            = field_list[2]
            self.param_preLEDPanelsFrameid              = field_list[3]
            self.param_preLEDPanelsStatefilterLo        = field_list[4]
            self.param_preLEDPanelsStatefilterHi        = field_list[5]
            self.param_preLEDPanelsStatefilterCriteria  = field_list[6]

            field_list = self.ListFromCsv(headerPreWait1)
            self.param_preWait1                    = field_list[0]
            
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

            field_list = self.ListFromCsv(headerPreWait2)
            self.param_preWait2                    = field_list[0]

            field_list = self.ListFromCsv(headerTrialRobot)
            self.param_trialRobotEnabled             = field_list[0]
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
            self.param_trialRobotMoveRelAngle            = field_list[12]
            self.param_trialRobotMoveRelAngleType        = field_list[13]
            self.param_trialRobotMoveRelSpeed            = field_list[14]
            self.param_trialRobotMoveRelSpeedType        = field_list[15]
            self.param_trialRobotMoveRelTolerance        = field_list[16]
    
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
        
            field_list = self.ListFromCsv(headerTrialLEDPanels)
            self.param_trialLEDPanelsEnabled              = field_list[0]
            self.param_trialLEDPanelsCommand              = field_list[1]
            self.param_trialLEDPanelsIdPattern            = field_list[2]
            self.param_trialLEDPanelsFrameid              = field_list[3]
            self.param_trialLEDPanelsStatefilterLo        = field_list[4]
            self.param_trialLEDPanelsStatefilterHi        = field_list[5]
            self.param_trialLEDPanelsStatefilterCriteria  = field_list[6]

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

            field_list = self.ListFromCsv(headerPostWait)
            self.param_postWait                     = field_list[0]


    # ReadHeader()
    # Given a file of any version, load all the header fields, using defaults if necessary.
    #        
    def ReadHeader (self, filename):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filename)
        fVersion = float(versionIn)
        if fVersion < 2.0:
            self.ReadHeader_V1(filename)
        elif fVersion >= 2.0 and fVersion <3.0:
            self.ReadHeader_V2(filename)
        elif fVersion >= 3.0 and fVersion <4.0:
            self.ReadHeader_V3(filename)
            
            
        


    def WriteHeader_V22(self, filename):
        paramsExperiment_V22 = self.templateExperiment_V22.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        paramsRobot_V22 = self.templateRobots_V22.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        paramsFlies_V22 = self.templateFlies_V22.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headingsTracking_V22 = self.headingsTrackingA_V22
        paramsTracking_V22 = self.templateTrackingA_V22.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headingsTracking_V22 += self.headingsTrackingB_V22
            paramsTracking_V22 += self.templateTrackingB_V22.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headingsTracking_V22 += '\n'
        paramsTracking_V22 += '\n'

        #######################################################################
        paramsPreWait1_V22 = self.templatePreWait1_V22.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        paramsPreTrigger_V22 = self.templatePreTrigger_V22.format(
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
        paramsTrialRobot_V22 = self.templateTrialRobot_V22.format(
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
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngle,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        paramsTrialLaser_V22 = self.templateTrialLaser_V22.format(
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
        paramsPostTrigger_V22 = self.templatePostTrigger_V22.format(
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
        paramsPostWait_V22 = self.templatePostWait_V22.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headingsExperiment_V22)
            fid.write(paramsExperiment_V22)
            fid.write('\n')
    
            fid.write(self.headingsRobot_V22)
            fid.write(paramsRobot_V22)
            fid.write('\n')
    
            fid.write(self.headingsFlies_V22)
            fid.write(paramsFlies_V22)
            fid.write('\n')
    
            fid.write(self.headingsTracking_V22)
            fid.write(paramsTracking_V22)
            fid.write('\n')
    
            fid.write(self.headingsPreWait1_V22)
            fid.write(paramsPreWait1_V22)
            fid.write('\n')
    
            fid.write(self.headingsPreTrigger_V22)
            fid.write(paramsPreTrigger_V22)
            fid.write('\n')
    
            fid.write(self.headingsTrialRobot_V22)
            fid.write(paramsTrialRobot_V22)
            fid.write('\n')
    
            fid.write(self.headingsTrialLaser_V22)
            fid.write(paramsTrialLaser_V22)
            fid.write('\n')
    
            fid.write(self.headingsPostTrigger_V22)
            fid.write(paramsPostTrigger_V22)
            fid.write('\n')
    
            fid.write(self.headingsPostWait_V22)
            fid.write(paramsPostWait_V22)
            fid.write('\n')
            
            fid.write('\n')
            fid.write('\n')
            fid.write('\n')

            fid.write('\n')
            fid.write('\n')
            fid.write('\n')


    def WriteHeader_V26(self, filename):
        paramsVersionFile_V26 = self.templateVersionFile_V26.format(
                                                versionFile              = '2.6',
                                                ) 
        paramsExperiment_V26 = self.templateExperiment_V26.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        paramsRobot_V26 = self.templateRobots_V26.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        paramsFlies_V26 = self.templateFlies_V26.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headingsTracking_V26 = self.headingsTrackingA_V26
        paramsTracking_V26 = self.templateTrackingA_V26.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headingsTracking_V26 += self.headingsTrackingB_V26
            paramsTracking_V26 += self.templateTrackingB_V26.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headingsTracking_V26 += '\n'
        paramsTracking_V26 += '\n'

        #######################################################################
        paramsPreRobot_V26 = self.templatePreRobot_V26.format(
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
                                                preRobotMoveRelAngle          = self.param_preRobotMoveRelAngle,
                                                preRobotMoveRelAngleType      = self.param_preRobotMoveRelAngleType,
                                                preRobotMoveRelSpeed          = self.param_preRobotMoveRelSpeed,
                                                preRobotMoveRelSpeedType      = self.param_preRobotMoveRelSpeedType,
                                                preRobotMoveRelTolerance      = self.param_preRobotMoveRelTolerance,
                                                )
        paramsPreLaser_V26 = self.templatePreLaser_V26.format(
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
                                                )
        paramsPreLEDPanels_V26 = self.templatePreLEDPanels_V26.format(
                                                preLEDPanelsEnabled = self.param_preLEDPanelsEnabled,
                                                preLEDPanelsCommand = self.param_preLEDPanelsCommand,
                                                preLEDPanelsIdPattern = self.param_preLEDPanelsIdPattern,
                                                preLEDPanelsFrameid = self.param_preLEDPanelsFrameid,
                                                preLEDPanelsStatefilterLo = self.param_preLEDPanelsStatefilterLo,
                                                preLEDPanelsStatefilterHi = self.param_preLEDPanelsStatefilterHi,
                                                preLEDPanelsStatefilterCriteria = self.param_preLEDPanelsStatefilterHi,
                                                )
        
        #######################################################################
        paramsPreWait1_V26 = self.templatePreWait1_V26.format(
                                                preWait1                  = self.param_preWait1,
                                                )
        paramsPreTrigger_V26 = self.templatePreTrigger_V26.format(
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
        paramsPreWait2_V26 = self.templatePreWait2_V26.format(
                                                preWait2                  = self.param_preWait2,
                                                )
        #######################################################################
        paramsTrialRobot_V26 = self.templateTrialRobot_V26.format(
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
                                                trialRobotMoveRelAngle          = self.param_trialRobotMoveRelAngle,
                                                trialRobotMoveRelAngleType      = self.param_trialRobotMoveRelAngleType,
                                                trialRobotMoveRelSpeed          = self.param_trialRobotMoveRelSpeed,
                                                trialRobotMoveRelSpeedType      = self.param_trialRobotMoveRelSpeedType,
                                                trialRobotMoveRelTolerance      = self.param_trialRobotMoveRelTolerance,
                                                )
        
        paramsTrialLaser_V26 = self.templateTrialLaser_V26.format(
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
        paramsTrialLEDPanels_V26 = self.templateTrialLEDPanels_V26.format(
                                                trialLEDPanelsEnabled           = self.param_trialLEDPanelsEnabled,
                                                trialLEDPanelsCommand           = self.param_trialLEDPanelsCommand,
                                                trialLEDPanelsIdPattern         = self.param_trialLEDPanelsIdPattern,
                                                trialLEDPanelsFrameid           = self.param_trialLEDPanelsFrameid,
                                                trialLEDPanelsStatefilterLo     = self.param_trialLEDPanelsStatefilterLo,
                                                trialLEDPanelsStatefilterHi     = self.param_trialLEDPanelsStatefilterHi,
                                                trialLEDPanelsStatefilterCriteria = self.param_trialLEDPanelsStatefilterHi,
                                                )
            
        #######################################################################
        paramsPostTrigger_V26 = self.templatePostTrigger_V26.format(
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
        paramsPostWait_V26 = self.templatePostWait_V26.format(
                                                postWait                   = str(self.param_postWait),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headingsVersionFile_V26)
            fid.write(paramsVersionFile_V26)
            fid.write('\n')
    
            fid.write(self.headingsExperiment_V26)
            fid.write(paramsExperiment_V26)
            fid.write('\n')
    
            fid.write(self.headingsRobot_V26)
            fid.write(paramsRobot_V26)
            fid.write('\n')
    
            fid.write(self.headingsFlies_V26)
            fid.write(paramsFlies_V26)
            fid.write('\n')
    
            fid.write(self.headingsTracking_V26)
            fid.write(paramsTracking_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreRobot_V26)
            fid.write(paramsPreRobot_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreLaser_V26)
            fid.write(paramsPreLaser_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreLEDPanels_V26)
            fid.write(paramsPreLEDPanels_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreWait1_V26)
            fid.write(paramsPreWait1_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreTrigger_V26)
            fid.write(paramsPreTrigger_V26)
            fid.write('\n')
    
            fid.write(self.headingsPreWait2_V26)
            fid.write(paramsPreWait2_V26)
            fid.write('\n')
    
            fid.write(self.headingsTrialRobot_V26)
            fid.write(paramsTrialRobot_V26)
            fid.write('\n')
    
            fid.write(self.headingsTrialLaser_V26)
            fid.write(paramsTrialLaser_V26)
            fid.write('\n')
    
            fid.write(self.headingsTrialLEDPanels_V26)
            fid.write(paramsTrialLEDPanels_V26)
            fid.write('\n')
    
            fid.write(self.headingsPostTrigger_V26)
            fid.write(paramsPostTrigger_V26)
            fid.write('\n')
    
            fid.write(self.headingsPostWait_V26)
            fid.write(paramsPostWait_V26)
            fid.write('\n')
            

    # Write the data lines from any-version input file to a pre-2.6 version output file.
    # Pre-version 2.6 doesn't include the 'triggered' column in the data.
    def CopyDataLines_Pre26(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'time')
            tmp = fidIn.readline()   # Skip the old heading text.
            
            with open(filenameOut, 'a') as fidOut:
                
                # Write the new heading line.
                self.headingsData_Pre26 = self.headingsDataLeft_Pre26 + self.headingsDataRobot_Pre26
                for i in range(nFlies):
                    self.headingsData_Pre26 += self.headingsDataFly_Pre26
                self.headingsData_Pre26 += '\n'
                fidOut.write(self.headingsData_Pre26)    
                
                
                # Write the data lines.
                for lineIn in fidIn:
                    field_list = self.ListFromCsv(lineIn)

                    lineOutLeft = self.templateDataLeft_Pre26.format(
                                                            time = field_list[0],
                                                            )
                    if (float(versionIn) < 2.6):
                        iField = 1
                    else:
                        iField = 2  # Skip the 'triggered' column.
                        
                    lineOutRobot = self.templateDataRobot_Pre26.format(
                                                            xRobot = field_list[iField+0],
                                                            yRobot = field_list[iField+1],
                                                            aRobot = field_list[iField+2],
                                                            vxRobot = field_list[iField+3],
                                                            vyRobot = field_list[iField+4],
                                                            vaRobot = field_list[iField+5],
                                                            )
                    iField += 6
                    lineOut = lineOutLeft + lineOutRobot
                    
                    for iFly in range(nFlies): 
                        lineOutFly = self.templateDataFly_Pre26.format(
                                                            xFly = field_list[iField+0],
                                                            yFly = field_list[iField+1],
                                                            aFly = field_list[iField+2],
                                                            vxFly = field_list[iField+3],
                                                            vyFly = field_list[iField+4],
                                                            vaFly = field_list[iField+5],
                                                            )
                        iField += 6
                        lineOut += lineOutFly
                        
                    lineOut += '\n'
                    fidOut.write(lineOut)
                    

    # Write the data lines from any-version input file to a post-2.6 version output file.
    # Post-version 2.6 includes the 'triggered' column in the data.
    def CopyDataLines_V26(self, filenameIn, filenameOut):
        (versionIn, nLinesHeader, nRobots, nFlies) = self.GetFileInfo(filenameIn)
        
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'time')
            tmp = fidIn.readline()   # Skip the old heading text.
            
            with open(filenameOut, 'a') as fidOut:
                
                # Write the new heading line.
                self.headingsData_V26 = self.headingsDataLeft_V26 + self.headingsDataRobot_V26
                for i in range(nFlies):
                    self.headingsData_V26 += self.headingsDataFly_V26
                self.headingsData_V26 += '\n'
                fidOut.write(self.headingsData_V26)    
                
                
                # Write the data lines.
                for lineIn in fidIn:
                    field_list = self.ListFromCsv(lineIn)

                    if (float(versionIn) < 2.6):
                        lineOutLeft = self.templateDataLeft_V26.format(
                                                            time = field_list[0],
                                                            triggered = 1,
                                                            )
                        iField = 1
                    else:
                        lineOutLeft = self.templateDataLeft_V26.format(
                                                            time = field_list[0],
                                                            triggered = field_list[1],
                                                            )
                        iField = 2  # Skip the 'triggered' column.
                        
                    lineOutRobot = self.templateDataRobot_V26.format(
                                                            xRobot = field_list[iField+0],
                                                            yRobot = field_list[iField+1],
                                                            aRobot = field_list[iField+2],
                                                            vxRobot = field_list[iField+3],
                                                            vyRobot = field_list[iField+4],
                                                            vaRobot = field_list[iField+5],
                                                            )
                    iField += 6
                    lineOut = lineOutLeft + lineOutRobot
                    
                    for iFly in range(nFlies): 
                        lineOutFly = self.templateDataFly_V26.format(
                                                            xFly = field_list[iField+0],
                                                            yFly = field_list[iField+1],
                                                            aFly = field_list[iField+2],
                                                            vxFly = field_list[iField+3],
                                                            vyFly = field_list[iField+4],
                                                            vaFly = field_list[iField+5],
                                                            )
                        iField += 6
                        lineOut += lineOutFly
                        
                    lineOut += '\n'
                    fidOut.write(lineOut)


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
            self.CopyDataLines_Pre26(filenameIn, filenameOut)
            
        elif (self.versionToWrite=='2.6') or (self.versionToWrite=='latest'):
            self.WriteHeader_V26(filenameOut)
            self.CopyDataLines_V26(filenameIn, filenameOut)
        

    # Convert all the .csv files in the input directory to a given version in the output directory.
    #
    def ConvertDirToDir(self, dirInBase, dirOutBase):
        
        if (self.versionToWrite=='2.2') or (self.versionToWrite=='2.6') or (self.versionToWrite=='latest'): 
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
            print ('Only versions "2.2", "2.6", and "latest" are supported for writing.')
            

    def ConvertTree(self, dirIn, dirOut):
        if (self.versionToWrite=='2.2') or (self.versionToWrite=='2.6') or (self.versionToWrite=='latest'): 
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
                            self.ConvertTree(filespecIn, filespecOut)
                        else:
                            if (os.path.splitext(filespecIn)[1]=='.csv'):
                                #print 'ConvertFile:  %s -> %s' % (filespecIn, filespecOut)
                                self.ConvertFile(filespecIn, filespecOut)
                            else:
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
            print ('Only versions "2.2", "2.6", and "latest" are supported for writing.')
            
        
if __name__ == '__main__':
    convert = ConvertCsv()
    
    
    ###############################################################################################
    ###############################################################################################
    convert.versionToWrite = '2.2'  # '2.2' or '2.6' or 'latest'
    dirIn   = '/home/ssafarik/FlylabData/2012_11_08'
    dirOut  = '/home/ssafarik/FlylabData_converted'
    ###############################################################################################
    ###############################################################################################

    
    convert.ConvertTree(dirIn, dirOut)
    
    
            
        
