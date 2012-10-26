#!/usr/bin/env python
from __future__ import division
import sys
import os
import glob


def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


class ConvertCsv:
    def __init__(self):
        self.headingsExperiment_V2 =   'date_time, '\
                                    'description, '\
                                    'maxTrials, '\
                                    'trial\n'
        self.templateExperiment_V2 =   '{date_time:s}, '\
                                    '{description:s}, '\
                                    '{maxTrials:s}, '\
                                    '{trial:s}\n'
                                  
        self.headingsRobot_V2 =        'nRobots, '\
                                    'widthRobot, '\
                                    'heightRobot, '\
                                    'visibleRobot, '\
                                    'paintRobot, '\
                                    'scentRobot\n'
        self.templateRobot_V2 =        '{nRobots:s}, '\
                                    '{widthRobot:s}, '\
                                    '{heightRobot:s}, '\
                                    '{visibleRobot:s}, '\
                                    '{paintRobot:s}, '\
                                    '{scentRobot:s}\n'
                                  
        self.headingsFlies_V2 =        'nFlies, '\
                                    'typeFlies, '\
                                    'genderFlies\n'
        self.templateFlies_V2 =        '{nFlies:s}, '\
                                    '{typeFlies:s}, '\
                                    '{genderFlies:s}\n'

        self.headingsTrackingA_V2 =    'trackingExclusionzoneEnabled'
        self.headingsTrackingB_V2 =    ', trackingExclusionzoneX, '\
                                    'trackingExclusionzoneY, '\
                                    'trackingExclusionzoneRadius'
        self.templateTrackingA_V2 =    '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB_V2 =    ', {trackingExclusionzoneX:s}, '\
                                    '{trackingExclusionzoneY:s}, '\
                                    '{trackingExclusionzoneRadius:s}'

        self.headingsWaitEntry_V2 =    'waitEntry\n'
        self.templateWaitEntry_V2 =    '{waitEntry:s}\n'
        
        self.headingsTriggerEntry_V2 = 'trigger1Enabled, '\
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
        self.templateTriggerEntry_V2 = '{trigger1Enabled:s}, '\
                                    '{trigger1FrameidParent:s}, '\
                                    '{trigger1FrameidChild:s}, '\
                                    '{trigger1SpeedAbsParentMin:s}, '\
                                    '{trigger1SpeedAbsParentMax:s}, '\
                                    '{trigger1SpeedAbsChildMin:s}, '\
                                    '{trigger1SpeedAbsChildMax:s}, '\
                                    '{trigger1SpeedRelMin:s}, '\
                                    '{trigger1SpeedRelMax:s}, '\
                                    '{trigger1DistanceMin:s}, '\
                                    '{trigger1DistanceMax:s}, '\
                                    '{trigger1AngleMin:s}, '\
                                    '{trigger1AngleMax:s}, '\
                                    '{trigger1AngleTest:s}, '\
                                    '{trigger1AngleTestBilateral:s}, '\
                                    '{trigger1TimeHold:s}, '\
                                    '{trigger1Timeout:s}\n'
                                    
        self.headingsMoveRobot_V2 =    'moverobotEnabled, '\
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
        self.templateMoveRobot_V2 =    '{moverobotEnabled:s}, '\
                                    '{moverobotPatternShape:s}, '\
                                    '{moverobotPatternHzPattern:s}, '\
                                    '{moverobotPatternHzPoint:s}, '\
                                    '{moverobotPatternCount:s}, '\
                                    '{moverobotPatternSizeX:s}, '\
                                    '{moverobotPatternSizeY:s}, '\
                                    '{moverobotPatternParam:s}, '\
                                    '{moverobotRelTracking:s}, '\
                                    '{moverobotRelOriginPosition:s}, '\
                                    '{moverobotRelOriginAngle:s}, '\
                                    '{moverobotRelDistance:s}, '\
                                    '{moverobotRelAngle:s}, '\
                                    '{moverobotRelAngleType:s}, '\
                                    '{moverobotRelSpeed:s}, '\
                                    '{moverobotRelSpeedType:s}, '\
                                    '{moverobotRelTolerance:s}\n'
                                    
        self.headingsLasertrack_V2 =   'laserEnabled, '\
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
        self.templateLasertrack_V2 =   '{laserEnabled:s}, '\
                                    '{laserPatternShape:s}, '\
                                    '{laserPatternHzPattern:s}, '\
                                    '{laserPatternHzPoint:s}, '\
                                    '{laserPatternCount:s}, '\
                                    '{laserPatternSizeX:s}, '\
                                    '{laserPatternSizeY:s}, '\
                                    '{laserPatternParam:s}, '\
                                    '{laserStatefilterLo:s}, '\
                                    '{laserStatefilterHi:s}, '\
                                    '{laserStatefilterCriteria:s}\n'
        
        self.headingsTriggerExit_V2 =  'trigger2Enabled, '\
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
        self.templateTriggerExit_V2 =  '{trigger2Enabled:s}, '\
                                    '{trigger2FrameidParent:s}, '\
                                    '{trigger2FrameidChild:s}, '\
                                    '{trigger2SpeedAbsParentMin:s}, '\
                                    '{trigger2SpeedAbsParentMax:s}, '\
                                    '{trigger2SpeedAbsChildMin:s}, '\
                                    '{trigger2SpeedAbsChildMax:s}, '\
                                    '{trigger2SpeedRelMin:s}, '\
                                    '{trigger2SpeedRelMax:s}, '\
                                    '{trigger2DistanceMin:s}, '\
                                    '{trigger2DistanceMax:s}, '\
                                    '{trigger2AngleMin:s}, '\
                                    '{trigger2AngleMax:s}, '\
                                    '{trigger2AngleTest:s}, '\
                                    '{trigger2AngleTestBilateral:s}, '\
                                    '{trigger2TimeHold:s}, '\
                                    '{trigger2Timeout:s}\n'
                                  
        self.headingsWaitExit_V2 =     'waitExit\n'
        self.templateWaitExit_V2 =     '{waitExit:s}\n'

        self.headingsData_V2 = 'time, xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, xFly, yFly, aFly, vxFly, vyFly, vaFly\n'
        self.templateData_V2 = '{time:0.4f}, {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}\n'

        


    # Advance the file seek position until the next line to be read has fieldGoal as the first csv field.
    #
    def AdvanceUntilField(self, fid, fieldGoal):
        field = 'bogus'
        while field != fieldGoal:
            pos = fid.tell()
            line = fid.readline()
            field = line.split(',')[0].strip(' \n')
        fid.seek(pos)
        

    # Return the version of a Flylab .csv file.
    #                
    def GetVersion(self, filename):
        with open(filename, 'r') as fid:
            self.AdvanceUntilField(fid, 'date_time')
            
            line = []
            for i in range(36):
                line.append(fid.readline())
            
            field = line[3].split(',')[0].strip(' \n')
            if field=='time':
                version = 1.0
                field = line[0].split(',')[4].strip(' \n')
                if field=='waitEntry':
                    version += 0.0
                elif field=='robot_width':
                    version += 0.1

            elif field=='nRobots':
                version = 2.0
                field = line[9].split(',')[0].strip(' \n')
                if field=='waitEntry':
                    version += 0.0                                  # v2.0
                elif field=='trackingExclusionzoneEnabled':
                    version += 0.1                                  # v2.1
                    field = line[15].split(',')[7].strip(' \n')
                    if field=='trigger1SpeedRelMin':
                        version += 0.1                              # v2.2

            else:
                version = 0.0
                
                        
        return version


    # Given a comma separated line of text, return a list of cleaned-up fields.
    def ListFromCsv (self, csv):
        field_list = csv.split(',')
        for i in range(len(field_list)):
            field_list[i] = field_list[i].strip(' \n')
            
        return field_list
                 

    def ReadHeader_V1 (self, filename):
        version = self.GetVersion(filename)
        self.nHeader = 4
        
        with open(filename, 'r') as fid:
            self.AdvanceUntilField(fid, 'date_time')
            headerTxt = fid.readline()
            header = fid.readline()

        fieldPre_list = header.split(',')
        field_list = []
        for field in fieldPre_list:
            field_list.append(field.strip(' \n'))
            
        if version==1.0: 
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

            self.param_waitEntry = field_list[4]
            
            self.param_trigger1Enabled = 'unspecified'
            self.param_trigger1FrameidParent = 'unspecified'
            self.param_trigger1FrameidChild = 'unspecified'
            self.param_trigger1DistanceMin = field_list[5]
            self.param_trigger1DistanceMax = field_list[6]
            self.param_trigger1SpeedAbsParentMin = 'unspecified'
            self.param_trigger1SpeedAbsParentMax = 'unspecified'
            self.param_trigger1SpeedAbsChildMin = 'unspecified'
            self.param_trigger1SpeedAbsChildMax = 'unspecified'
            self.param_trigger1SpeedRelMin = field_list[7]
            self.param_trigger1SpeedRelMax = field_list[8]
            self.param_trigger1AngleMin = field_list[9]
            self.param_trigger1AngleMax = field_list[10]
            self.param_trigger1AngleTest = field_list[11]
            self.param_trigger1AngleTestBilateral = field_list[12]
            self.param_trigger1TimeHold = field_list[13]
            self.param_trigger1Timeout = 'unspecified'

            self.param_trigger2Enabled = 'unspecified'
            self.param_trigger2FrameidParent = 'unspecified'
            self.param_trigger2FrameidChild = 'unspecified'
            self.param_trigger2DistanceMin = field_list[14]
            self.param_trigger2DistanceMax = field_list[15]
            self.param_trigger2SpeedAbsParentMin = 'unspecified'
            self.param_trigger2SpeedAbsParentMax = 'unspecified'
            self.param_trigger2SpeedAbsChildMin = 'unspecified'
            self.param_trigger2SpeedAbsChildMax = 'unspecified'
            self.param_trigger2SpeedRelMin = field_list[16]
            self.param_trigger2SpeedRelMax = field_list[17]
            self.param_trigger2AngleMin = field_list[18]
            self.param_trigger2AngleMax = field_list[19]
            self.param_trigger2AngleTest = field_list[20]
            self.param_trigger2AngleTestBilateral = field_list[21]
            self.param_trigger2TimeHold = field_list[22]
            self.param_trigger2Timeout = 'unspecified'

            self.param_nRobots = 'unspecified'
            self.param_widthRobot = field_list[23]
            self.param_heightRobot = field_list[24]
            self.param_visibleRobot = field_list[25]
            self.param_paintRobot = field_list[26]
            self.param_scentRobot = field_list[27]

            self.param_moverobotEnabled = 'unspecified'
            self.param_moverobotPatternShape = field_list[28]
            self.param_moverobotPatternHzPattern = field_list[29]
            self.param_moverobotPatternHzPoint = field_list[30]
            self.param_moverobotPatternCount = field_list[31]
            self.param_moverobotPatternSizeX = field_list[32]
            self.param_moverobotPatternSizeY = field_list[33]
            self.param_moverobotPatternParam = '0.0'
            self.param_moverobotRelTracking = field_list[34]
            self.param_moverobotRelOriginPosition = field_list[35]
            self.param_moverobotRelOriginAngle = field_list[36]
            self.param_moverobotRelDistance = field_list[37]
            self.param_moverobotRelAngle = field_list[38]
            self.param_moverobotRelAngleType = field_list[39]
            self.param_moverobotRelSpeed = field_list[40]
            self.param_moverobotRelSpeedType = field_list[41]
            self.param_moverobotRelTolerance = field_list[42]
    
            self.param_laserEnabled = 'False'
            self.param_laserPatternShape = 'unspecified'
            self.param_laserPatternHzPattern = 'unspecified'
            self.param_laserPatternHzPoint = 'unspecified'
            self.param_laserPatternCount = 'unspecified'
            self.param_laserPatternSizeX = 'unspecified'
            self.param_laserPatternSizeY = 'unspecified'
            self.param_laserPatternParam = 'unspecified'
            self.param_laserStatefilterLo = 'unspecified'
            self.param_laserStatefilterHi = 'unspecified'
            self.param_laserStatefilterCriteria = 'unspecified'
        
            self.param_waitExit = '0.0'


        if version==1.1: 
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

            self.param_waitEntry                    = field_list[9]
            
            self.param_trigger1Enabled              = 'unspecified'
            self.param_trigger1FrameidParent        = field_list[10]
            self.param_trigger1FrameidChild         = field_list[11]
            self.param_trigger1SpeedAbsParentMin    = field_list[12]
            self.param_trigger1SpeedAbsParentMax    = field_list[13]
            self.param_trigger1SpeedAbsChildMin     = field_list[14]
            self.param_trigger1SpeedAbsChildMax     = field_list[15]
            self.param_trigger1SpeedRelMin          = 'unspecified'
            self.param_trigger1SpeedRelMax          = 'unspecified'
            self.param_trigger1DistanceMin          = field_list[16]
            self.param_trigger1DistanceMax          = field_list[17]
            self.param_trigger1AngleMin             = field_list[18]
            self.param_trigger1AngleMax             = field_list[19]
            self.param_trigger1AngleTest            = field_list[20]
            self.param_trigger1AngleTestBilateral   = field_list[21]
            self.param_trigger1TimeHold             = field_list[22]
            self.param_trigger1Timeout              = 'unspecified'

            self.param_moverobotEnabled             = 'unspecified'
            self.param_moverobotPatternShape        = field_list[23]
            self.param_moverobotPatternHzPattern    = field_list[24]
            self.param_moverobotPatternHzPoint      = field_list[25]
            self.param_moverobotPatternCount        = field_list[26]
            self.param_moverobotPatternSizeX        = field_list[27]
            self.param_moverobotPatternSizeY        = field_list[28]
            self.param_moverobotPatternParam        = '0.0'
            self.param_moverobotRelTracking         = field_list[29]
            self.param_moverobotRelOriginPosition   = field_list[30]
            self.param_moverobotRelOriginAngle      = field_list[31]
            self.param_moverobotRelDistance         = field_list[32]
            self.param_moverobotRelAngle            = field_list[33]
            self.param_moverobotRelAngleType        = field_list[34]
            self.param_moverobotRelSpeed            = field_list[35]
            self.param_moverobotRelSpeedType        = field_list[36]
            self.param_moverobotRelTolerance        = field_list[37]
    
            self.param_laserEnabled                 = field_list[38]
            self.param_laserPatternShape            = 'unspecified'
            self.param_laserPatternHzPattern        = 'unspecified'
            self.param_laserPatternHzPoint          = 'unspecified'
            self.param_laserPatternCount            = 'unspecified'
            self.param_laserPatternSizeX            = 'unspecified'
            self.param_laserPatternSizeY            = 'unspecified'
            self.param_laserPatternParam            = 'unspecified'
            self.param_laserStatefilterLo           = 'unspecified'
            self.param_laserStatefilterHi           = 'unspecified'
            self.param_laserStatefilterCriteria     = 'unspecified'
        
            self.param_trigger2Enabled              = 'unspecified'
            self.param_trigger2FrameidParent        = field_list[39]
            self.param_trigger2FrameidChild         = field_list[40]
            self.param_trigger2SpeedAbsParentMin    = field_list[41]
            self.param_trigger2SpeedAbsParentMax    = field_list[42]
            self.param_trigger2SpeedAbsChildMin     = field_list[43]
            self.param_trigger2SpeedAbsChildMax     = field_list[44]
            self.param_trigger2SpeedRelMin          = 'unspecified'
            self.param_trigger2SpeedRelMax          = 'unspecified'
            self.param_trigger2DistanceMin          = field_list[45]
            self.param_trigger2DistanceMax          = field_list[46]
            self.param_trigger2AngleMin             = field_list[47]
            self.param_trigger2AngleMax             = field_list[48]
            self.param_trigger2AngleTest            = field_list[49]
            self.param_trigger2AngleTestBilateral   = field_list[50]
            self.param_trigger2TimeHold             = field_list[51]
            self.param_trigger2Timeout              = 'unspecified'

            self.param_waitExit = '0.0'


    def ReadHeader_V2 (self, filename):
        version = self.GetVersion(filename)
        self.nHeader = 37
        
        with open(filename, 'r') as fid:
            self.AdvanceUntilField(fid, 'date_time')
            
            headerExperimentTxt = fid.readline()
            headerExperiment = fid.readline()
            blank = fid.readline()
            
            headerRobotsTxt = fid.readline()
            headerRobots = fid.readline()
            blank = fid.readline()
            
            headerFliesTxt = fid.readline()
            headerFlies = fid.readline()
            blank = fid.readline()
            
            if version >= 2.1:
                headerTrackingTxt = fid.readline()
                headerTracking = fid.readline()
                blank = fid.readline()
            
            headerWaitEntryTxt = fid.readline()
            headerWaitEntry = fid.readline()
            blank = fid.readline()
            
            headerTriggerEntryTxt = fid.readline()
            headerTriggerEntry = fid.readline()
            blank = fid.readline()
            
            headerMoverobotTxt = fid.readline()
            headerMoverobot = fid.readline()
            blank = fid.readline()
            
            headerLaserTxt = fid.readline()
            headerLaser = fid.readline()
            blank = fid.readline()
            
            headerTriggerExitTxt = fid.readline()
            headerTriggerExit = fid.readline()
            blank = fid.readline()
            
            headerWaitExitTxt = fid.readline()
            headerWaitExit = fid.readline()
            blank = fid.readline()
        

        if version==2.0:
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
            
            self.param_trackingExclusionzoneEnabled     = 'False'
            self.param_trackingExclusionzoneX_list      = []
            self.param_trackingExclusionzoneY_list      = []
            self.param_trackingExclusionzoneRadius_list = []

            field_list = self.ListFromCsv(headerWaitEntry)
            self.param_waitEntry                    = field_list[0]
            
            field_list = self.ListFromCsv(headerTriggerEntry)
            self.param_trigger1Enabled              = field_list[0]
            self.param_trigger1FrameidParent        = field_list[1]
            self.param_trigger1FrameidChild         = field_list[2]
            self.param_trigger1SpeedAbsParentMin    = field_list[3]
            self.param_trigger1SpeedAbsParentMax    = field_list[4]
            self.param_trigger1SpeedAbsChildMin     = field_list[5]
            self.param_trigger1SpeedAbsChildMax     = field_list[6]
            self.param_trigger1SpeedRelMin          = 'unspecified'
            self.param_trigger1SpeedRelMax          = 'unspecified'
            self.param_trigger1DistanceMin          = field_list[7]
            self.param_trigger1DistanceMax          = field_list[8]
            self.param_trigger1AngleMin             = field_list[9]
            self.param_trigger1AngleMax             = field_list[10]
            self.param_trigger1AngleTest            = field_list[11]
            self.param_trigger1AngleTestBilateral   = field_list[12]
            self.param_trigger1TimeHold             = field_list[13]
            self.param_trigger1Timeout              = field_list[14]

            field_list = self.ListFromCsv(headerMoverobot)
            self.param_moverobotEnabled             = field_list[0]
            self.param_moverobotPatternShape        = field_list[1]
            self.param_moverobotPatternHzPattern    = field_list[2]
            self.param_moverobotPatternHzPoint      = field_list[3]
            self.param_moverobotPatternCount        = field_list[4]
            self.param_moverobotPatternSizeX        = field_list[5]
            self.param_moverobotPatternSizeY        = field_list[6]
            self.param_moverobotPatternParam        = '0.0'
            self.param_moverobotRelTracking         = field_list[7]
            self.param_moverobotRelOriginPosition   = field_list[8]
            self.param_moverobotRelOriginAngle      = field_list[9]
            self.param_moverobotRelDistance         = field_list[10]
            self.param_moverobotRelAngle            = field_list[11]
            self.param_moverobotRelAngleType        = field_list[12]
            self.param_moverobotRelSpeed            = field_list[13]
            self.param_moverobotRelSpeedType        = field_list[14]
            self.param_moverobotRelTolerance        = field_list[15]
    
            field_list = self.ListFromCsv(headerLaser)
            self.param_laserEnabled                 = field_list[0]
            self.param_laserPatternShape            = 'unspecified'
            self.param_laserPatternHzPattern        = 'unspecified'
            self.param_laserPatternHzPoint          = 'unspecified'
            self.param_laserPatternCount            = 'unspecified'
            self.param_laserPatternSizeX            = 'unspecified'
            self.param_laserPatternSizeY            = 'unspecified'
            self.param_laserPatternParam            = 'unspecified'
            self.param_laserStatefilterLo           = 'unspecified'
            self.param_laserStatefilterHi           = 'unspecified'
            self.param_laserStatefilterCriteria     = 'unspecified'
        
            field_list = self.ListFromCsv(headerTriggerExit)
            self.param_trigger2Enabled              = field_list[0]
            self.param_trigger2FrameidParent        = field_list[1]
            self.param_trigger2FrameidChild         = field_list[2]
            self.param_trigger2SpeedAbsParentMin    = field_list[3]
            self.param_trigger2SpeedAbsParentMax    = field_list[4]
            self.param_trigger2SpeedAbsChildMin     = field_list[5]
            self.param_trigger2SpeedAbsChildMax     = field_list[6]
            self.param_trigger2SpeedRelMin          = 'unspecified'
            self.param_trigger2SpeedRelMax          = 'unspecified'
            self.param_trigger2DistanceMin          = field_list[7]
            self.param_trigger2DistanceMax          = field_list[8]
            self.param_trigger2AngleMin             = field_list[9]
            self.param_trigger2AngleMax             = field_list[10]
            self.param_trigger2AngleTest            = field_list[11]
            self.param_trigger2AngleTestBilateral   = field_list[12]
            self.param_trigger2TimeHold             = field_list[13]
            self.param_trigger2Timeout              = field_list[14]

            field_list = self.ListFromCsv(headerWaitExit)
            self.param_waitExit                     = field_list[0]


        if version==2.1:
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

            field_list = self.ListFromCsv(headerWaitEntry)
            self.param_waitEntry                    = field_list[0]
            
            field_list = self.ListFromCsv(headerTriggerEntry)
            self.param_trigger1Enabled              = field_list[0]
            self.param_trigger1FrameidParent        = field_list[1]
            self.param_trigger1FrameidChild         = field_list[2]
            self.param_trigger1SpeedAbsParentMin    = field_list[3]
            self.param_trigger1SpeedAbsParentMax    = field_list[4]
            self.param_trigger1SpeedAbsChildMin     = field_list[5]
            self.param_trigger1SpeedAbsChildMax     = field_list[6]
            self.param_trigger1SpeedRelMin          = 'unspecified'
            self.param_trigger1SpeedRelMax          = 'unspecified'
            self.param_trigger1DistanceMin          = field_list[7]
            self.param_trigger1DistanceMax          = field_list[8]
            self.param_trigger1AngleMin             = field_list[9]
            self.param_trigger1AngleMax             = field_list[10]
            self.param_trigger1AngleTest            = field_list[11]
            self.param_trigger1AngleTestBilateral   = field_list[12]
            self.param_trigger1TimeHold             = field_list[13]
            self.param_trigger1Timeout              = field_list[14]

            field_list = self.ListFromCsv(headerMoverobot)
            self.param_moverobotEnabled             = field_list[0]
            self.param_moverobotPatternShape        = field_list[1]
            self.param_moverobotPatternHzPattern    = field_list[2]
            self.param_moverobotPatternHzPoint      = field_list[3]
            self.param_moverobotPatternCount        = field_list[4]
            self.param_moverobotPatternSizeX        = field_list[5]
            self.param_moverobotPatternSizeY        = field_list[6]
            self.param_moverobotPatternParam        = field_list[7]
            self.param_moverobotRelTracking         = field_list[8]
            self.param_moverobotRelOriginPosition   = field_list[9]
            self.param_moverobotRelOriginAngle      = field_list[10]
            self.param_moverobotRelDistance         = field_list[11]
            self.param_moverobotRelAngle            = field_list[12]
            self.param_moverobotRelAngleType        = field_list[13]
            self.param_moverobotRelSpeed            = field_list[14]
            self.param_moverobotRelSpeedType        = field_list[15]
            self.param_moverobotRelTolerance        = field_list[16]
    
            field_list = self.ListFromCsv(headerLaser)
            self.param_laserEnabled                 = field_list[0]
            self.param_laserPatternShape            = field_list[1]
            self.param_laserPatternHzPattern        = field_list[2]
            self.param_laserPatternHzPoint          = field_list[3]
            self.param_laserPatternCount            = field_list[4]
            self.param_laserPatternSizeX            = field_list[5]
            self.param_laserPatternSizeY            = field_list[6]
            self.param_laserPatternParam            = field_list[7]
            self.param_laserStatefilterLo           = field_list[8]
            self.param_laserStatefilterHi           = field_list[9]
            self.param_laserStatefilterCriteria     = field_list[10]
        
            field_list = self.ListFromCsv(headerTriggerExit)
            self.param_trigger2Enabled              = field_list[0]
            self.param_trigger2FrameidParent        = field_list[1]
            self.param_trigger2FrameidChild         = field_list[2]
            self.param_trigger2SpeedAbsParentMin    = field_list[3]
            self.param_trigger2SpeedAbsParentMax    = field_list[4]
            self.param_trigger2SpeedAbsChildMin     = field_list[5]
            self.param_trigger2SpeedAbsChildMax     = field_list[6]
            self.param_trigger2SpeedRelMin          = 'unspecified'
            self.param_trigger2SpeedRelMax          = 'unspecified'
            self.param_trigger2DistanceMin          = field_list[7]
            self.param_trigger2DistanceMax          = field_list[8]
            self.param_trigger2AngleMin             = field_list[9]
            self.param_trigger2AngleMax             = field_list[10]
            self.param_trigger2AngleTest            = field_list[11]
            self.param_trigger2AngleTestBilateral   = field_list[12]
            self.param_trigger2TimeHold             = field_list[13]
            self.param_trigger2Timeout              = field_list[14]

            field_list = self.ListFromCsv(headerWaitExit)
            self.param_waitExit                     = field_list[0]


        if version==2.2:
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

            field_list = self.ListFromCsv(headerWaitEntry)
            self.param_waitEntry                    = field_list[0]
            
            field_list = self.ListFromCsv(headerTriggerEntry)
            self.param_trigger1Enabled              = field_list[0]
            self.param_trigger1FrameidParent        = field_list[1]
            self.param_trigger1FrameidChild         = field_list[2]
            self.param_trigger1SpeedAbsParentMin    = field_list[3]
            self.param_trigger1SpeedAbsParentMax    = field_list[4]
            self.param_trigger1SpeedAbsChildMin     = field_list[5]
            self.param_trigger1SpeedAbsChildMax     = field_list[6]
            self.param_trigger1SpeedRelMin          = field_list[7]
            self.param_trigger1SpeedRelMax          = field_list[8]
            self.param_trigger1DistanceMin          = field_list[9]
            self.param_trigger1DistanceMax          = field_list[10]
            self.param_trigger1AngleMin             = field_list[11]
            self.param_trigger1AngleMax             = field_list[12]
            self.param_trigger1AngleTest            = field_list[13]
            self.param_trigger1AngleTestBilateral   = field_list[14]
            self.param_trigger1TimeHold             = field_list[15]
            self.param_trigger1Timeout              = field_list[16]

            field_list = self.ListFromCsv(headerMoverobot)
            self.param_moverobotEnabled             = field_list[0]
            self.param_moverobotPatternShape        = field_list[1]
            self.param_moverobotPatternHzPattern    = field_list[2]
            self.param_moverobotPatternHzPoint      = field_list[3]
            self.param_moverobotPatternCount        = field_list[4]
            self.param_moverobotPatternSizeX        = field_list[5]
            self.param_moverobotPatternSizeY        = field_list[6]
            self.param_moverobotPatternParam        = field_list[7]
            self.param_moverobotRelTracking         = field_list[8]
            self.param_moverobotRelOriginPosition   = field_list[9]
            self.param_moverobotRelOriginAngle      = field_list[10]
            self.param_moverobotRelDistance         = field_list[11]
            self.param_moverobotRelAngle            = field_list[12]
            self.param_moverobotRelAngleType        = field_list[13]
            self.param_moverobotRelSpeed            = field_list[14]
            self.param_moverobotRelSpeedType        = field_list[15]
            self.param_moverobotRelTolerance        = field_list[16]
    
            field_list = self.ListFromCsv(headerLaser)
            self.param_laserEnabled                 = field_list[0]
            self.param_laserPatternShape            = field_list[1]
            self.param_laserPatternHzPattern        = field_list[2]
            self.param_laserPatternHzPoint          = field_list[3]
            self.param_laserPatternCount            = field_list[4]
            self.param_laserPatternSizeX            = field_list[5]
            self.param_laserPatternSizeY            = field_list[6]
            self.param_laserPatternParam            = field_list[7]
            self.param_laserStatefilterLo           = field_list[8]
            self.param_laserStatefilterHi           = field_list[9]
            self.param_laserStatefilterCriteria     = field_list[10]
        
            field_list = self.ListFromCsv(headerTriggerExit)
            self.param_trigger2Enabled              = field_list[0]
            self.param_trigger2FrameidParent        = field_list[1]
            self.param_trigger2FrameidChild         = field_list[2]
            self.param_trigger2SpeedAbsParentMin    = field_list[3]
            self.param_trigger2SpeedAbsParentMax    = field_list[4]
            self.param_trigger2SpeedAbsChildMin     = field_list[5]
            self.param_trigger2SpeedAbsChildMax     = field_list[6]
            self.param_trigger2SpeedRelMin          = field_list[7]
            self.param_trigger2SpeedRelMax          = field_list[8]
            self.param_trigger2DistanceMin          = field_list[9]
            self.param_trigger2DistanceMax          = field_list[10]
            self.param_trigger2AngleMin             = field_list[11]
            self.param_trigger2AngleMax             = field_list[12]
            self.param_trigger2AngleTest            = field_list[13]
            self.param_trigger2AngleTestBilateral   = field_list[14]
            self.param_trigger2TimeHold             = field_list[15]
            self.param_trigger2Timeout              = field_list[16]

            field_list = self.ListFromCsv(headerWaitExit)
            self.param_waitExit                     = field_list[0]


    # ReadHeader()
    # Given a file of any version, load all the header fields, using defaults if necessary.
    #        
    def ReadHeader (self, filename):
        version = self.GetVersion(filename)
        
        if version < 2.0:
            self.ReadHeader_V1(filename)
        elif version >= 2.0 and version <3.0:
            self.ReadHeader_V2(filename)
            
            
        


    def WriteHeader_V2(self, filename):
        paramsExperiment_V2 = self.templateExperiment_V2.format(
                                                date_time                  = self.param_date_time,
                                                description                = self.param_description,
                                                maxTrials                  = self.param_maxTrials,
                                                trial                      = self.param_trial,
                                                )
        paramsRobot_V2 = self.templateRobot_V2.format(
                                                nRobots                    = self.param_nRobots,
                                                widthRobot                 = self.param_widthRobot,
                                                heightRobot                = self.param_heightRobot,
                                                visibleRobot               = self.param_visibleRobot,
                                                paintRobot                 = self.param_paintRobot,
                                                scentRobot                 = self.param_scentRobot,
                                                )
        paramsFlies_V2 = self.templateFlies_V2.format(
                                                nFlies                     = self.param_nFlies,
                                                typeFlies                  = self.param_typeFlies,
                                                genderFlies                = self.param_genderFlies,
                                                )

        self.headingsTracking_V2 = self.headingsTrackingA_V2
        paramsTracking_V2 = self.templateTrackingA_V2.format(
                                                       trackingExclusionzoneEnabled = self.param_trackingExclusionzoneEnabled
                                                       )
        for i in range(len(self.param_trackingExclusionzoneX_list)):
            self.headingsTracking_V2 += self.headingsTrackingB_V2
            paramsTracking_V2 += self.templateTrackingB_V2.format(
                                                trackingExclusionzoneX       = str(self.param_trackingExclusionzoneX_list[i]),
                                                trackingExclusionzoneY       = str(self.param_trackingExclusionzoneY_list[i]),
                                                trackingExclusionzoneRadius  = str(self.param_trackingExclusionzoneRadius_list[i]),
                                                )
        self.headingsTracking_V2 += '\n'
        paramsTracking_V2 += '\n'

        paramsWaitEntry_V2 = self.templateWaitEntry_V2.format(
                                                waitEntry                  = self.param_waitEntry,
                                                )
        paramsTriggerEntry_V2 = self.templateTriggerEntry_V2.format(
                                                trigger1Enabled            = self.param_trigger1Enabled,
                                                trigger1FrameidParent      = self.param_trigger1FrameidParent,
                                                trigger1FrameidChild       = self.param_trigger1FrameidChild,
                                                trigger1SpeedAbsParentMin  = self.param_trigger1SpeedAbsParentMin,
                                                trigger1SpeedAbsParentMax  = self.param_trigger1SpeedAbsParentMax,
                                                trigger1SpeedAbsChildMin   = self.param_trigger1SpeedAbsChildMin,
                                                trigger1SpeedAbsChildMax   = self.param_trigger1SpeedAbsChildMax,
                                                trigger1SpeedRelMin        = self.param_trigger1SpeedRelMin,
                                                trigger1SpeedRelMax        = self.param_trigger1SpeedRelMax,
                                                trigger1DistanceMin        = self.param_trigger1DistanceMin,
                                                trigger1DistanceMax        = self.param_trigger1DistanceMax,
                                                trigger1AngleMin           = self.param_trigger1AngleMin,
                                                trigger1AngleMax           = self.param_trigger1AngleMax,
                                                trigger1AngleTest          = self.param_trigger1AngleTest,
                                                trigger1AngleTestBilateral = self.param_trigger1AngleTestBilateral,
                                                trigger1TimeHold           = self.param_trigger1TimeHold,
                                                trigger1Timeout            = self.param_trigger1Timeout,
                                                )
        paramsMoveRobot_V2 = self.templateMoveRobot_V2.format(
                                                moverobotEnabled           = self.param_moverobotEnabled,
                                                moverobotPatternShape      = self.param_moverobotPatternShape,
                                                moverobotPatternHzPattern  = self.param_moverobotPatternHzPattern,
                                                moverobotPatternHzPoint    = self.param_moverobotPatternHzPoint,
                                                moverobotPatternCount      = self.param_moverobotPatternCount,
                                                moverobotPatternSizeX      = self.param_moverobotPatternSizeX,
                                                moverobotPatternSizeY      = self.param_moverobotPatternSizeY,
                                                moverobotPatternParam      = self.param_moverobotPatternParam,
                                                moverobotRelTracking       = self.param_moverobotRelTracking,
                                                moverobotRelOriginPosition = self.param_moverobotRelOriginPosition,
                                                moverobotRelOriginAngle    = self.param_moverobotRelOriginAngle,
                                                moverobotRelDistance       = self.param_moverobotRelDistance,
                                                moverobotRelAngle          = self.param_moverobotRelAngle,
                                                moverobotRelAngleType      = self.param_moverobotRelAngleType,
                                                moverobotRelSpeed          = self.param_moverobotRelSpeed,
                                                moverobotRelSpeedType      = self.param_moverobotRelSpeedType,
                                                moverobotRelTolerance      = self.param_moverobotRelTolerance,
                                                )
        
        paramsLasertrack_V2 = self.templateLasertrack_V2.format(
                                                laserEnabled               = self.param_laserEnabled,
                                                laserPatternShape          = self.param_laserPatternShape,
                                                laserPatternHzPattern      = self.param_laserPatternHzPattern,
                                                laserPatternHzPoint        = self.param_laserPatternHzPoint,
                                                laserPatternCount          = self.param_laserPatternCount,
                                                laserPatternSizeX          = self.param_laserPatternSizeX,
                                                laserPatternSizeY          = self.param_laserPatternSizeY,
                                                laserPatternParam          = self.param_laserPatternParam,
                                                laserStatefilterHi         = self.param_laserStatefilterHi,
                                                laserStatefilterLo         = self.param_laserStatefilterLo,
                                                laserStatefilterCriteria   = self.param_laserStatefilterCriteria,
                                                )
            
        paramsTriggerExit_V2 = self.templateTriggerExit_V2.format(
                                                trigger2Enabled            = str(self.param_trigger2Enabled),
                                                trigger2FrameidParent      = str(self.param_trigger2FrameidParent),
                                                trigger2FrameidChild       = str(self.param_trigger2FrameidChild),
                                                trigger2SpeedAbsParentMin  = str(self.param_trigger2SpeedAbsParentMin),
                                                trigger2SpeedAbsParentMax  = str(self.param_trigger2SpeedAbsParentMax),
                                                trigger2SpeedAbsChildMin   = str(self.param_trigger2SpeedAbsChildMin),
                                                trigger2SpeedAbsChildMax   = str(self.param_trigger2SpeedAbsChildMax),
                                                trigger2SpeedRelMin        = str(self.param_trigger2SpeedRelMin),
                                                trigger2SpeedRelMax        = str(self.param_trigger2SpeedRelMax),
                                                trigger2DistanceMin        = str(self.param_trigger2DistanceMin),
                                                trigger2DistanceMax        = str(self.param_trigger2DistanceMax),
                                                trigger2AngleMin           = str(self.param_trigger2AngleMin),
                                                trigger2AngleMax           = str(self.param_trigger2AngleMax),
                                                trigger2AngleTest          = str(self.param_trigger2AngleTest),
                                                trigger2AngleTestBilateral = str(self.param_trigger2AngleTestBilateral),
                                                trigger2TimeHold           = str(self.param_trigger2TimeHold),
                                                trigger2Timeout            = str(self.param_trigger2Timeout),
                                                )
        paramsWaitExit_V2 = self.templateWaitExit_V2.format(
                                                waitExit                   = str(self.param_waitExit),
                                                )

        with open(filename, 'w') as fid:
            fid.write(self.headingsExperiment_V2)
            fid.write(paramsExperiment_V2)
            fid.write('\n')
    
            fid.write(self.headingsRobot_V2)
            fid.write(paramsRobot_V2)
            fid.write('\n')
    
            fid.write(self.headingsFlies_V2)
            fid.write(paramsFlies_V2)
            fid.write('\n')
    
            fid.write(self.headingsTracking_V2)
            fid.write(paramsTracking_V2)
            fid.write('\n')
    
            fid.write(self.headingsWaitEntry_V2)
            fid.write(paramsWaitEntry_V2)
            fid.write('\n')
    
            fid.write(self.headingsTriggerEntry_V2)
            fid.write(paramsTriggerEntry_V2)
            fid.write('\n')
    
            fid.write(self.headingsMoveRobot_V2)
            fid.write(paramsMoveRobot_V2)
            fid.write('\n')
    
            fid.write(self.headingsLasertrack_V2)
            fid.write(paramsLasertrack_V2)
            fid.write('\n')
    
            fid.write(self.headingsTriggerExit_V2)
            fid.write(paramsTriggerExit_V2)
            fid.write('\n')
    
            fid.write(self.headingsWaitExit_V2)
            fid.write(paramsWaitExit_V2)
            fid.write('\n')
            
            fid.write('\n')
            fid.write('\n')
            fid.write('\n')
            
            fid.write('\n')
            fid.write('\n')
            fid.write('\n')


    def CopyDataLines(self, filenameIn, filenameOut):
        with open(filenameIn, 'r') as fidIn:
            self.AdvanceUntilField(fidIn, 'date_time')
            with open(filenameOut, 'a') as fidOut:
            
                for i in range(self.nHeader):
                    tmp = fidIn.readline()
                    
                fidOut.write(self.headingsData_V2)
        
                for line in fidIn:
                    fidOut.write(line)
                    

    # Convert all the .csv files in the input directory to version 2 files in the output directory.
    #
    def ConvertDirToDir(self, dirInBase, dirOutBase):
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
                print '%s version %f' % (filenameIn, self.GetVersion(filenameIn))
                filenameLeaf = filenameIn.split('/')[-1]
                filenameOut = dirOut + '/' + filenameLeaf
                self.ReadHeader(filenameIn)
                self.WriteHeader_V2(filenameOut)
                self.CopyDataLines(filenameIn, filenameOut)
        
        
if __name__ == '__main__':
    convert = ConvertCsv()
    
    dirIn   = '/home/ssafarik/WeeklyMeeting'
    dirOut  = '/home/ssafarik/WeeklyMeeting_V2'
    convert.ConvertDirToDir(dirIn, dirOut)
    
    
            
        
