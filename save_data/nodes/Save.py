#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save_data')
import rospy

import tf
import sys
import time, os, subprocess
import threading
import numpy as N

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge, CvBridgeError

from flycore.msg import MsgFrameState
from experiments.srv import Trigger, ExperimentParams
from tracking.msg import ArenaState


def Chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


###############################################################################
# Save() is a ROS node.  It saves Arenastate messages into .csv files, 
# and saves Image messages to .png files.
#
#  At the end of each trial, a video is made.  
# There should be one video frame per line in the .csv
#
class Save:
    def __init__(self):
        self.initialized = False
        self.dirWorking_base = os.path.expanduser("~/FlylabData")
        Chdir(self.dirWorking_base)


        # Create new directory each day
        self.dirRelative = time.strftime("%Y_%m_%d")
        self.dirWorking = self.dirWorking_base + "/" + self.dirRelative
        Chdir(self.dirWorking)

        self.triggered = False
        self.saveOnlyWhileTriggered = False # False: Save everything from one new_trial to the next new_trial.  True:  Save everything from trigger=on to trigger=off.

        self.nFlies         = rospy.get_param("nFlies", 0)
        self.typeFlies      = rospy.get_param("fly/type", "unspecified")
        self.genderFlies    = rospy.get_param("fly/gender", "unspecified")
        self.nRobots        = rospy.get_param("nRobots", 0)
        self.widthRobot     = rospy.get_param("robot/width", "3.175") # mm
        self.heightRobot    = rospy.get_param("robot/height", "3.175") # mm
        self.visibleRobot   = bool(rospy.get_param("robot/visible", "True"))
        self.paintRobot     = str(rospy.get_param("robot/paint", "blackoxide"))
        self.scentRobot     = str(rospy.get_param("robot/scent", "unscented"))

        rospy.Service('save/new_trial', ExperimentParams, self.NewTrial_callback)
        rospy.Service('save/trigger', Trigger, self.Trigger_callback)

        
        ############# Video stuff
        self.fileNull = open('/dev/null', 'w')
        self.iFrame = 0

        self.topicVideo = rospy.get_param("save/videotopic", 'camera/image_rect')
        self.subImage = rospy.Subscriber(self.topicVideo, Image, self.Image_callback)
        
        self.bridge = CvBridge()

        self.image = None
        self.filenameVideo = None
        self.saveVideo = False
        self.bSavingVideo = False

        #self.nRepeatFrames = int(rospy.get_param('save/video_image_repeat_count'))
        self.sizeImage = None

        
        
        ############# Arenastate/CSV stuff
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.sub_arenastate = rospy.Subscriber("ArenaState", ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        self.lockArenastate = threading.Lock()
        self.lockVideo = threading.Lock()
        
        self.filename = None
        self.fid = None
        self.saveArenastate = False
        self.bSavingArenastate = False

        self.format_align = ">"
        self.format_sign = " "
        self.format_width = "7"
        self.format_precision = "2"
        self.format_type = "f"

        
        #################################################################################
        self.versionFile = '2.7'    # Increment this when the file format changes.
        #################################################################################

        
        self.headingsVersionFile =  'versionFile\n'
        self.templateVersionFile =  '{versionFile:s}\n'
        
        self.headingsExperiment =   'date_time, '\
                                    'description, '\
                                    'maxTrials, '\
                                    'trial\n'
        self.templateExperiment =   '{date_time:s}, '\
                                    '{description:s}, '\
                                    '{maxTrials:s}, '\
                                    '{trial:s}\n'
                                  
        self.headingsRobot =        'nRobots, '\
                                    'widthRobot, '\
                                    'heightRobot, '\
                                    'visibleRobot, '\
                                    'paintRobot, '\
                                    'scentRobot\n'
        self.templateRobot =        '{nRobots:s}, '\
                                    '{widthRobot:s}, '\
                                    '{heightRobot:s}, '\
                                    '{visibleRobot:s}, '\
                                    '{paintRobot:s}, '\
                                    '{scentRobot:s}\n'
                                  
        self.headingsFlies =        'nFlies, '\
                                    'typeFlies, '\
                                    'genderFlies\n'
        self.templateFlies =        '{nFlies:s}, '\
                                    '{typeFlies:s}, '\
                                    '{genderFlies:s}\n'

        self.headingsTrackingA =    'trackingExclusionzoneEnabled'
        self.headingsTrackingB =    ', trackingExclusionzoneX, '\
                                    'trackingExclusionzoneY, '\
                                    'trackingExclusionzoneRadius'
        self.templateTrackingA =    '{trackingExclusionzoneEnabled:s}'
        self.templateTrackingB =    ', {trackingExclusionzoneX:s}, '\
                                    '{trackingExclusionzoneY:s}, '\
                                    '{trackingExclusionzoneRadius:s}'

        self.headingsPreRobot =     'preRobotEnabled, '\
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
                                    'preRobotMoveRelTolerance, ' \
                                    'preRobotMoveTimeout\n'
        self.templatePreRobot     = '{preRobotEnabled:s}, '\
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
                                    '{preRobotMoveRelTolerance:s}, ' \
                                    '{preRobotMoveTimeout:s}\n'
                                    
        self.headingsPreLaser     = 'preLaserEnabled, '\
                                    'preLaserPatternShape, '\
                                    'preLaserPatternHzPattern, '\
                                    'preLaserPatternHzPoint, '\
                                    'preLaserPatternCount, '\
                                    'preLaserPatternSizeX, '\
                                    'preLaserPatternSizeY, '\
                                    'preLaserPatternParam, '\
                                    'preLaserStatefilterLo, '\
                                    'preLaserStatefilterHi, '\
                                    'preLaserStatefilterCriteria, ' \
                                    'preLaserTimeout\n'
        self.templatePreLaser =     '{preLaserEnabled:s}, '\
                                    '{preLaserPatternShape:s}, '\
                                    '{preLaserPatternHzPattern:s}, '\
                                    '{preLaserPatternHzPoint:s}, '\
                                    '{preLaserPatternCount:s}, '\
                                    '{preLaserPatternSizeX:s}, '\
                                    '{preLaserPatternSizeY:s}, '\
                                    '{preLaserPatternParam:s}, '\
                                    '\"{preLaserStatefilterLo:s}\", '\
                                    '\"{preLaserStatefilterHi:s}\", '\
                                    '{preLaserStatefilterCriteria:s}, ' \
                                    '{preLaserTimeout:s}\n'
        
        self.headingsPreLEDPanels = 'preLEDPanelsEnabled, '\
                                    'preLEDPanelsCommand, '\
                                    'preLEDPanelsIdPattern, '\
                                    'preLEDPanelsFrameid, '\
                                    'preLEDPanelsStatefilterLo, '\
                                    'preLEDPanelsStatefilterHi, '\
                                    'preLEDPanelsStatefilterCriteria, '\
                                    'preLEDPanelsTimeout\n'
        self.templatePreLEDPanels = '{preLEDPanelsEnabled:s}, '\
                                    '{preLEDPanelsCommand:s}, '\
                                    '{preLEDPanelsIdPattern:s}, '\
                                    '{preLEDPanelsFrameid:s}, '\
                                    '{preLEDPanelsStatefilterLo:s}, '\
                                    '{preLEDPanelsStatefilterHi:s}, '\
                                    '{preLEDPanelsStatefilterCriteria:s}, '\
                                    '{preLEDPanelsTimeout:s}\n'

        
        self.headingsPreWait1 =     'preWait1\n'
        self.templatePreWait1 =     '{preWait1:s}\n'
        
        self.headingsPreTrigger =   'preTriggerEnabled, '\
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
                                    
        self.headingsPreWait2 =     'preWait2\n'
        self.templatePreWait2 =     '{preWait2:s}\n'
        
        
        self.headingsTrialRobot =   'trialRobotEnabled, '\
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
                                    'trialRobotMoveRelTolerance, ' \
                                    'trialRobotMoveTimeout\n'
        self.templateTrialRobot =   '{trialRobotEnabled:s}, '\
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
                                    '{trialRobotMoveRelTolerance:s}, ' \
                                    '{trialRobotMoveTimeout:s}\n'
                                    
        self.headingsTrialLaser =   'trialLaserEnabled, '\
                                    'trialLaserPatternShape, '\
                                    'trialLaserPatternHzPattern, '\
                                    'trialLaserPatternHzPoint, '\
                                    'trialLaserPatternCount, '\
                                    'trialLaserPatternSizeX, '\
                                    'trialLaserPatternSizeY, '\
                                    'trialLaserPatternParam, '\
                                    'trialLaserStatefilterLo, '\
                                    'trialLaserStatefilterHi, '\
                                    'trialLaserStatefilterCriteria, ' \
                                    'trialLaserTimeout\n'
        self.templateTrialLaser =   '{trialLaserEnabled:s}, '\
                                    '{trialLaserPatternShape:s}, '\
                                    '{trialLaserPatternHzPattern:s}, '\
                                    '{trialLaserPatternHzPoint:s}, '\
                                    '{trialLaserPatternCount:s}, '\
                                    '{trialLaserPatternSizeX:s}, '\
                                    '{trialLaserPatternSizeY:s}, '\
                                    '{trialLaserPatternParam:s}, '\
                                    '\"{trialLaserStatefilterLo:s}\", '\
                                    '\"{trialLaserStatefilterHi:s}\", '\
                                    '{trialLaserStatefilterCriteria:s}, ' \
                                    '{trialLaserTimeout:s}\n'
        
        self.headingsTrialLEDPanels = 'trialLEDPanelsEnabled, '\
                                    'trialLEDPanelsCommand, '\
                                    'trialLEDPanelsIdPattern, '\
                                    'trialLEDPanelsFrameid, '\
                                    'trialLEDPanelsStatefilterLo, '\
                                    'trialLEDPanelsStatefilterHi, '\
                                    'trialLEDPanelsStatefilterCriteria, '\
                                    'trialLEDPanelsTimeout\n'
        self.templateTrialLEDPanels = '{trialLEDPanelsEnabled:s}, '\
                                    '{trialLEDPanelsCommand:s}, '\
                                    '{trialLEDPanelsIdPattern:s}, '\
                                    '{trialLEDPanelsFrameid:s}, '\
                                    '{trialLEDPanelsStatefilterLo:s}, '\
                                    '{trialLEDPanelsStatefilterHi:s}, '\
                                    '{trialLEDPanelsStatefilterCriteria:s}, '\
                                    '{trialLEDPanelsTimeout:s}\n'
        
        self.headingsPostTrigger =  'postTriggerEnabled, '\
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
                                  
        self.headingsPostWait =     'postWait\n'
        self.templatePostWait =     '{postWait:s}\n'

        # Construct a variable-length heading for the data lines, depending on the number of flies.
        self.headingsDataLeft   = 'time, triggered'
        self.headingsDataRobot  = ', xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, aRobotWingLeft, aRobotWingRight'
        self.headingsDataFly    = ', xFly, yFly, aFly, vxFly, vyFly, vaFly, aFlyWingLeft, aFlyWingRight'
        self.templateDataLeft   = '{time:0.4f}, {triggered:s}'
        self.templateDataRobot  = ', {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {aRobotWingLeft:{align}{sign}{width}.{precision}{type}}, {aRobotWingRight:{align}{sign}{width}.{precision}{type}}'
        self.templateDataFly    = ', {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}, {aFlyWingLeft:{align}{sign}{width}.{precision}{type}}, {aFlyWingRight:{align}{sign}{width}.{precision}{type}}'

        self.headingsData = self.headingsDataLeft + self.headingsDataRobot
        for i in range(self.nFlies):
            self.headingsData += self.headingsDataFly
        self.headingsData += '\n'

        self.templateData = self.templateDataLeft + self.templateDataRobot
        for i in range(self.nFlies):
            self.templateData += self.templateDataFly
        self.templateData += '\n'


        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        with self.lockArenastate:
            if (self.fid is not None) and (not self.fid.closed):
                self.fid.close()
                rospy.logwarn('SA logfile close()')

        with self.lockVideo:
            if (self.fileNull is not None) and (not self.fileNull.closed):
                self.fileNull.close()
                self.fileNull = None
            
        

    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
    #    Closes the .csv file if no longer saving.
    #
    def Trigger_callback(self, reqTrigger):
        if (self.initialized):

            bRisingEdge = False
            bFallingEdge = False
            if self.triggered != reqTrigger.triggered:
                self.triggered = reqTrigger.triggered
                if self.triggered: # Rising edge.
                    bRisingEdge = True
                else:
                    bFallingEdge = True


            if (self.saveOnlyWhileTriggered) and (self.saveArenastate):
                if (reqTrigger.triggered):
                    self.bSavingArenastate = True
                else:
                    self.bSavingArenastate = False
            
            if (self.saveOnlyWhileTriggered) and (self.saveVideo):
                if (reqTrigger.triggered):
                    self.bSavingVideo = True
                else:
                    self.bSavingVideo = False
            
        
            # At the end of a run, close the file if we're no longer saving.
            if (self.saveOnlyWhileTriggered):
                if (self.saveArenastate):
                    with self.lockArenastate:
                        if (bFallingEdge) and (self.fid is not None) and (not self.fid.closed):
                            self.fid.close()
                            rospy.logwarn('SA logfile close()')
    
                if (self.saveVideo):                    
                    if (bRisingEdge):
                        self.ResetFrameCounter()
                            
                    if (bFallingEdge) and (self.filenameVideo is not None):
                        self.WriteVideoFromFrames()
                    
            
        return self.triggered
        

    # NewTrial_callback()
    # Close old .csv file if there was one.
    # Open a new .csv file when we start a new trial.
    # Possibly set flag self.bSavingArenastate to save arenastate.
    # Returns with a file open.
    # 
    def NewTrial_callback(self, experimentparamsReq):
        self.saveArenastate = experimentparamsReq.save.arenastate
        self.saveVideo = experimentparamsReq.save.video
        
        if (self.initialized):
            self.saveOnlyWhileTriggered = experimentparamsReq.save.onlyWhileTriggered

            if (self.saveArenastate):
                # Close old .csv file if there was one.
                with self.lockArenastate:
                    if (self.fid is not None) and (not self.fid.closed):
                        self.fid.close()
                        rospy.logwarn('SA logfile close()')
                    
                # Determine if we should be saving.
                if (self.saveOnlyWhileTriggered):
                    self.bSavingArenastate = False
                else:
                    self.bSavingArenastate = True
                
                self.OpenCsvAndWriteHeader(experimentparamsReq)


            if (self.saveVideo):
                # If there are prior frames to convert, then convert them.
                if (self.filenameVideo is not None):
                    self.WriteVideoFromFrames()
                
                self.ResetFrameCounter()
    
                # Determine if we should be saving.
                if (self.saveOnlyWhileTriggered):
                    self.bSavingVideo = False
                else:
                    self.bSavingVideo = True
                
                
                now = rospy.Time.now().to_sec()
                self.dirFrames = "%s%04d%02d%02d%02d%02d%02d" % (experimentparamsReq.save.filenamebase, 
                                                                time.localtime(now).tm_year,
                                                                time.localtime(now).tm_mon,
                                                                time.localtime(now).tm_mday,
                                                                time.localtime(now).tm_hour,
                                                                time.localtime(now).tm_min,
                                                                time.localtime(now).tm_sec)
    
                self.dirBase = os.path.expanduser("~/FlylabData")
                Chdir(self.dirBase)
                self.dirVideo = self.dirBase + "/" + time.strftime("%Y_%m_%d")
                Chdir(self.dirVideo)
                #self.dirFrames = self.dirVideo + "/frames"
                #Chdir(self.dirFrames)
                # At this point we should be in ~/FlylabData/YYYYmmdd/images
                
                try:
                    os.mkdir(self.dirFrames)
                except OSError, e:
                    rospy.logwarn ('Cannot create directory %s: %s' % (self.dirFrames,e))
    
            
                self.filenameVideo = "%s/%s%04d%02d%02d%02d%02d%02d.mov" % (self.dirVideo,
                                                                experimentparamsReq.save.filenamebase, 
                                                                time.localtime(now).tm_year,
                                                                time.localtime(now).tm_mon,
                                                                time.localtime(now).tm_mday,
                                                                time.localtime(now).tm_hour,
                                                                time.localtime(now).tm_min,
                                                                time.localtime(now).tm_sec)

                
        return True
                
                
    def OpenCsvAndWriteHeader(self, experimentparamsReq):                
        #self.filename = "%s%04d.csv" % (experimentparamsReq.save.filenamebase, experimentparamsReq.experiment.trial)
        now = rospy.Time.now().to_sec()
        self.filename = "%s%04d%02d%02d%02d%02d%02d.csv" % (experimentparamsReq.save.filenamebase, 
                                                            time.localtime(now).tm_year,
                                                            time.localtime(now).tm_mon,
                                                            time.localtime(now).tm_mday,
                                                            time.localtime(now).tm_hour,
                                                            time.localtime(now).tm_min,
                                                            time.localtime(now).tm_sec)
        
        paramsVersionFile = self.templateVersionFile.format(
                                                versionFile                = self.versionFile
                                                )
        paramsExperiment = self.templateExperiment.format(
                                                date_time                  = str(rospy.Time.now().to_sec()),
                                                description                = str(experimentparamsReq.experiment.description),
                                                maxTrials                  = str(experimentparamsReq.experiment.maxTrials),
                                                trial                      = str(experimentparamsReq.experiment.trial),
                                                )
        paramsRobot = self.templateRobot.format(
                                                nRobots                    = str(self.nRobots),
                                                widthRobot                 = str(self.widthRobot),
                                                heightRobot                = str(self.heightRobot),
                                                visibleRobot               = str(self.visibleRobot),
                                                paintRobot                 = str(self.paintRobot),
                                                scentRobot                 = str(self.scentRobot),
                                                )
        paramsFlies = self.templateFlies.format(
                                                nFlies                     = str(self.nFlies),
                                                typeFlies                  = str(self.typeFlies),
                                                genderFlies                = str(self.genderFlies),
                                                )

        self.headingsTracking = self.headingsTrackingA
        paramsTracking = self.templateTrackingA.format(trackingExclusionzoneEnabled = str(experimentparamsReq.tracking.exclusionzones.enabled))
        for i in range(len(experimentparamsReq.tracking.exclusionzones.point_list)):
            self.headingsTracking += self.headingsTrackingB
            paramsTracking += self.templateTrackingB.format(
                                                trackingExclusionzoneX       = str(experimentparamsReq.tracking.exclusionzones.point_list[i].x),
                                                trackingExclusionzoneY       = str(experimentparamsReq.tracking.exclusionzones.point_list[i].y),
                                                trackingExclusionzoneRadius  = str(experimentparamsReq.tracking.exclusionzones.radius_list[i]),
                                                )
        self.headingsTracking += '\n'
        paramsTracking += '\n'

        #######################################################################
        paramsPreRobot = self.templatePreRobot.format(
                                                preRobotEnabled               = str(experimentparamsReq.pre.robot.enabled),
                                                preRobotMovePatternShape      = str(experimentparamsReq.pre.robot.move.pattern.shape),
                                                preRobotMovePatternHzPattern  = str(experimentparamsReq.pre.robot.move.pattern.hzPattern),
                                                preRobotMovePatternHzPoint    = str(experimentparamsReq.pre.robot.move.pattern.hzPoint),
                                                preRobotMovePatternCount      = str(experimentparamsReq.pre.robot.move.pattern.count),
                                                preRobotMovePatternSizeX      = str(experimentparamsReq.pre.robot.move.pattern.size.x),
                                                preRobotMovePatternSizeY      = str(experimentparamsReq.pre.robot.move.pattern.size.y),
                                                preRobotMovePatternParam      = str(experimentparamsReq.pre.robot.move.pattern.param),
                                                preRobotMoveRelTracking       = str(experimentparamsReq.pre.robot.move.relative.tracking),
                                                preRobotMoveRelOriginPosition = str(experimentparamsReq.pre.robot.move.relative.frameidOriginPosition),
                                                preRobotMoveRelOriginAngle    = str(experimentparamsReq.pre.robot.move.relative.frameidOriginAngle),
                                                preRobotMoveRelDistance       = str(experimentparamsReq.pre.robot.move.relative.distance),
                                                preRobotMoveRelAngle          = str(experimentparamsReq.pre.robot.move.relative.angle),
                                                preRobotMoveRelAngleType      = str(experimentparamsReq.pre.robot.move.relative.angleType),
                                                preRobotMoveRelSpeed          = str(experimentparamsReq.pre.robot.move.relative.speed),
                                                preRobotMoveRelSpeedType      = str(experimentparamsReq.pre.robot.move.relative.speedType),
                                                preRobotMoveRelTolerance      = str(experimentparamsReq.pre.robot.move.relative.tolerance),
                                                preRobotMoveTimeout           = str(experimentparamsReq.pre.robot.move.timeout),
                                                )
        
        #######################################################################
        if len(experimentparamsReq.pre.lasertrack.pattern_list) > 0:
            patternShape           = str(experimentparamsReq.pre.lasertrack.pattern_list[0].shape)
            patternHzPattern       = str(experimentparamsReq.pre.lasertrack.pattern_list[0].hzPattern)
            patternHzPoint         = str(experimentparamsReq.pre.lasertrack.pattern_list[0].hzPoint)
            patternCount           = str(experimentparamsReq.pre.lasertrack.pattern_list[0].count)
            patternSizeX           = str(experimentparamsReq.pre.lasertrack.pattern_list[0].size.x)
            patternSizeY           = str(experimentparamsReq.pre.lasertrack.pattern_list[0].size.y)
            patternParam           = str(experimentparamsReq.pre.lasertrack.pattern_list[0].param)
        else:
            patternShape           = ""
            patternHzPattern       = str(0.0)
            patternHzPoint         = str(0.0)
            patternCount           = str(0)
            patternSizeX           = str(0.0)
            patternSizeY           = str(0.0)
            patternParam           = str(0.0)

        if len(experimentparamsReq.pre.lasertrack.statefilterHi_list) > 0:
            statefilterHi          = str(experimentparamsReq.pre.lasertrack.statefilterHi_list[0])
            statefilterLo          = str(experimentparamsReq.pre.lasertrack.statefilterLo_list[0])
            statefilterCriteria    = str(experimentparamsReq.pre.lasertrack.statefilterCriteria_list[0])
        else:
            statefilterHi          = ""
            statefilterLo          = ""
            statefilterCriteria    = ""

        paramsPreLaser = self.templatePreLaser.format(
                                                preLaserEnabled               = str(experimentparamsReq.pre.lasertrack.enabled),
                                                preLaserPatternShape          = patternShape,
                                                preLaserPatternHzPattern      = patternHzPattern,
                                                preLaserPatternHzPoint        = patternHzPoint,
                                                preLaserPatternCount          = patternCount,
                                                preLaserPatternSizeX          = patternSizeX,
                                                preLaserPatternSizeY          = patternSizeY,
                                                preLaserPatternParam          = patternParam,
                                                preLaserStatefilterHi         = statefilterHi,
                                                preLaserStatefilterLo         = statefilterLo,
                                                preLaserStatefilterCriteria   = statefilterCriteria,
                                                preLaserTimeout               = str(experimentparamsReq.pre.lasertrack.timeout),
                                                )
            
        #######################################################################
        paramsPreLEDPanels = self.templatePreLEDPanels.format(
                                                preLEDPanelsEnabled             = str(experimentparamsReq.pre.ledpanels.enabled),
                                                preLEDPanelsCommand             = str(experimentparamsReq.pre.ledpanels.command),
                                                preLEDPanelsIdPattern           = str(experimentparamsReq.pre.ledpanels.idPattern),
                                                preLEDPanelsFrameid             = str(experimentparamsReq.pre.ledpanels.frame_id),
                                                preLEDPanelsStatefilterLo       = str(experimentparamsReq.pre.ledpanels.statefilterLo),
                                                preLEDPanelsStatefilterHi       = str(experimentparamsReq.pre.ledpanels.statefilterHi),
                                                preLEDPanelsStatefilterCriteria = str(experimentparamsReq.pre.ledpanels.statefilterCriteria),
                                                preLEDPanelsTimeout             = str(experimentparamsReq.pre.ledpanels.timeout),
                                                )
        #######################################################################
        paramsPreWait1 = self.templatePreWait1.format(
                                                preWait1                  = str(experimentparamsReq.pre.wait1),
                                                )
        #######################################################################
        paramsPreTrigger = self.templatePreTrigger.format(
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
        paramsPreWait2 = self.templatePreWait2.format(
                                                preWait2                       = str(experimentparamsReq.pre.wait2),
                                                )

        
        #######################################################################
        paramsTrialRobot = self.templateTrialRobot.format(
                                                trialRobotEnabled               = str(experimentparamsReq.trial.robot.enabled),
                                                trialRobotMovePatternShape      = str(experimentparamsReq.trial.robot.move.pattern.shape),
                                                trialRobotMovePatternHzPattern  = str(experimentparamsReq.trial.robot.move.pattern.hzPattern),
                                                trialRobotMovePatternHzPoint    = str(experimentparamsReq.trial.robot.move.pattern.hzPoint),
                                                trialRobotMovePatternCount      = str(experimentparamsReq.trial.robot.move.pattern.count),
                                                trialRobotMovePatternSizeX      = str(experimentparamsReq.trial.robot.move.pattern.size.x),
                                                trialRobotMovePatternSizeY      = str(experimentparamsReq.trial.robot.move.pattern.size.y),
                                                trialRobotMovePatternParam      = str(experimentparamsReq.trial.robot.move.pattern.param),
                                                trialRobotMoveRelTracking       = str(experimentparamsReq.trial.robot.move.relative.tracking),
                                                trialRobotMoveRelOriginPosition = str(experimentparamsReq.trial.robot.move.relative.frameidOriginPosition),
                                                trialRobotMoveRelOriginAngle    = str(experimentparamsReq.trial.robot.move.relative.frameidOriginAngle),
                                                trialRobotMoveRelDistance       = str(experimentparamsReq.trial.robot.move.relative.distance),
                                                trialRobotMoveRelAngle          = str(experimentparamsReq.trial.robot.move.relative.angle),
                                                trialRobotMoveRelAngleType      = str(experimentparamsReq.trial.robot.move.relative.angleType),
                                                trialRobotMoveRelSpeed          = str(experimentparamsReq.trial.robot.move.relative.speed),
                                                trialRobotMoveRelSpeedType      = str(experimentparamsReq.trial.robot.move.relative.speedType),
                                                trialRobotMoveRelTolerance      = str(experimentparamsReq.trial.robot.move.relative.tolerance),
                                                trialRobotMoveTimeout           = str(experimentparamsReq.trial.robot.move.timeout),
                                                )
        
        #######################################################################
        if len(experimentparamsReq.trial.lasertrack.pattern_list) > 0:
            patternShape           = str(experimentparamsReq.trial.lasertrack.pattern_list[0].shape)
            patternHzPattern       = str(experimentparamsReq.trial.lasertrack.pattern_list[0].hzPattern)
            patternHzPoint         = str(experimentparamsReq.trial.lasertrack.pattern_list[0].hzPoint)
            patternCount           = str(experimentparamsReq.trial.lasertrack.pattern_list[0].count)
            patternSizeX           = str(experimentparamsReq.trial.lasertrack.pattern_list[0].size.x)
            patternSizeY           = str(experimentparamsReq.trial.lasertrack.pattern_list[0].size.y)
            patternParam           = str(experimentparamsReq.trial.lasertrack.pattern_list[0].param)
        else:
            patternShape           = ""
            patternHzPattern       = str(0.0)
            patternHzPoint         = str(0.0)
            patternCount           = str(0)
            patternSizeX           = str(0.0)
            patternSizeY           = str(0.0)
            patternParam           = str(0.0)

        if len(experimentparamsReq.trial.lasertrack.statefilterHi_list) > 0:
            statefilterHi          = str(experimentparamsReq.trial.lasertrack.statefilterHi_list[0])
            statefilterLo          = str(experimentparamsReq.trial.lasertrack.statefilterLo_list[0])
            statefilterCriteria    = str(experimentparamsReq.trial.lasertrack.statefilterCriteria_list[0])
        else:
            statefilterHi          = ""
            statefilterLo          = ""
            statefilterCriteria    = ""

        paramsTrialLaser = self.templateTrialLaser.format(
                                                trialLaserEnabled               = str(experimentparamsReq.trial.lasertrack.enabled),
                                                trialLaserPatternShape          = patternShape,
                                                trialLaserPatternHzPattern      = patternHzPattern,
                                                trialLaserPatternHzPoint        = patternHzPoint,
                                                trialLaserPatternCount          = patternCount,
                                                trialLaserPatternSizeX          = patternSizeX,
                                                trialLaserPatternSizeY          = patternSizeY,
                                                trialLaserPatternParam          = patternParam,
                                                trialLaserStatefilterHi         = statefilterHi,
                                                trialLaserStatefilterLo         = statefilterLo,
                                                trialLaserStatefilterCriteria   = statefilterCriteria,
                                                trialLaserTimeout               = str(experimentparamsReq.trial.lasertrack.timeout),
                                                )
            
        #######################################################################
        paramsTrialLEDPanels = self.templateTrialLEDPanels.format(
                                                trialLEDPanelsEnabled             = str(experimentparamsReq.trial.ledpanels.enabled),
                                                trialLEDPanelsCommand             = str(experimentparamsReq.trial.ledpanels.command),
                                                trialLEDPanelsIdPattern           = str(experimentparamsReq.trial.ledpanels.idPattern),
                                                trialLEDPanelsFrameid             = str(experimentparamsReq.trial.ledpanels.frame_id),
                                                trialLEDPanelsStatefilterLo       = str(experimentparamsReq.trial.ledpanels.statefilterLo),
                                                trialLEDPanelsStatefilterHi       = str(experimentparamsReq.trial.ledpanels.statefilterHi),
                                                trialLEDPanelsStatefilterCriteria = str(experimentparamsReq.trial.ledpanels.statefilterCriteria),
                                                trialLEDPanelsTimeout             = str(experimentparamsReq.trial.ledpanels.timeout),
                                                )
            
        #######################################################################
        paramsPostTrigger = self.templatePostTrigger.format(
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
        paramsPostWait = self.templatePostWait.format(
                                                postWait                      = str(experimentparamsReq.post.wait),
                                                )
        #######################################################################


        with self.lockArenastate:
            self.fid = open(self.filename, 'w')
            rospy.logwarn('SA logfile open(%s)' % self.filename)

            self.fid.write(self.headingsVersionFile)
            self.fid.write(paramsVersionFile)
            self.fid.write('\n')

            self.fid.write(self.headingsExperiment)
            self.fid.write(paramsExperiment)
            self.fid.write('\n')

            self.fid.write(self.headingsRobot)
            self.fid.write(paramsRobot)
            self.fid.write('\n')

            self.fid.write(self.headingsFlies)
            self.fid.write(paramsFlies)
            self.fid.write('\n')

            self.fid.write(self.headingsTracking)
            self.fid.write(paramsTracking)
            self.fid.write('\n')

            self.fid.write(self.headingsPreRobot)
            self.fid.write(paramsPreRobot)
            self.fid.write('\n')

            self.fid.write(self.headingsPreLaser)
            self.fid.write(paramsPreLaser)
            self.fid.write('\n')

            self.fid.write(self.headingsPreLEDPanels)
            self.fid.write(paramsPreLEDPanels)
            self.fid.write('\n')
            
            self.fid.write(self.headingsPreWait1)
            self.fid.write(paramsPreWait1)
            self.fid.write('\n')

            self.fid.write(self.headingsPreTrigger)
            self.fid.write(paramsPreTrigger)
            self.fid.write('\n')

            self.fid.write(self.headingsPreWait2)
            self.fid.write(paramsPreWait2)
            self.fid.write('\n')

            self.fid.write(self.headingsTrialRobot)
            self.fid.write(paramsTrialRobot)
            self.fid.write('\n')

            self.fid.write(self.headingsTrialLaser)
            self.fid.write(paramsTrialLaser)
            self.fid.write('\n')

            self.fid.write(self.headingsTrialLEDPanels)
            self.fid.write(paramsTrialLEDPanels)
            self.fid.write('\n')
            
            self.fid.write(self.headingsPostTrigger)
            self.fid.write(paramsPostTrigger)
            self.fid.write('\n')

            self.fid.write(self.headingsPostWait)
            self.fid.write(paramsPostWait)
            self.fid.write('\n')
            
            
            self.fid.write(self.headingsData)

    
    

    def get_imagenames(self, dir):
        proc_ls_png = subprocess.Popen('ls ' + dir + '/*.png',
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileNull)
        out = proc_ls_png.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    def WriteVideoFromFrames(self):
        with self.lockVideo:
            
            # Rewrite all the image files, with duplicate frames to simulate slow-motion.
            #Chdir(self.dirImage)
            #imagenames = self.get_imagenames(self.dirImage)
            #iFrame = 0
            #for imagename in imagenames:
            #    Chdir(self.dirFrames)
            #    image = cv.LoadImage(imagename)
            #    Chdir(self.dirFrames2)
            #    for iRepeat in range(self.nRepeatFrames):
            #        filenameImage = "{num:06d}.png".format(num=iFrame)
            #        cv.SaveImage(filenameImage, image)
            #        iFrame += 1
    
#            cmdCreateVideoFile = 'ffmpeg -f image2 -i ' + self.dirFrames + '/%06d.png -r ' + str(self.framerate) + ' ' + \
#                                   '-sameq -s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -g 100 -bf 2 -pass 1/2 ' + \
#                                   self.filenameVideo
            cmdCreateVideoFile = 'avconv -i ' + self.dirFrames + '/%06d.png ' + self.filenameVideo
            rospy.logwarn('Converting .png images to video using command:')
            rospy.logwarn (cmdCreateVideoFile)
            try:
                #subprocess.check_call(cmdCreateVideoFile, shell=True)
                subprocess.Popen(cmdCreateVideoFile, shell=True)
            except:
                rospy.logerr('Exception running avconv')
                
            rospy.logwarn('Saved %s' % (self.filenameVideo))
            self.filenameVideo = None

            
    def ResetFrameCounter(self):
        #try:
        #    rospy.logwarn('Deleting frame images.')
        #    subprocess.call('rm '+self.dirFrames+'/*.png', shell=True)
        #except OSError:
        #    pass
        
        self.iFrame = 0


    def WriteFilePng(self, cvimage):
        if self.sizeImage is None:
            self.sizeImage = cv.GetSize(cvimage)
        filenameImage = self.dirFrames+"/{num:06d}.png".format(num=self.iFrame)
        cv.SaveImage(filenameImage, cvimage)
        self.iFrame += 1


    def Image_callback(self, image):
            self.image = image


    def ArenaState_callback(self, arenastate):
        if (self.initialized) and (self.bSavingArenastate) and (self.fid is not None):

            # Get latest time.
            stamp = max(0.0, arenastate.robot.header.stamp.to_sec())
            for iFly in range(len(arenastate.flies)): #self.nFlies):
                stamp = max(stamp, arenastate.flies[iFly].header.stamp.to_sec())
            
            # Get the state of the robot.
            stateRobot = arenastate.robot
            q = stateRobot.pose.orientation
            rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            angleRobot = rpy[2] % (2.0 * N.pi)


            # Start putting the data row together.
            dataLeft = self.templateDataLeft.format(
                                                    align       = self.format_align,
                                                    sign        = self.format_sign,
                                                    width       = self.format_width,
                                                    precision   = self.format_precision,
                                                    type        = self.format_type,
                                                    time        = stamp, #rospy.Time.now().to_sec(), #stateRobot.header.stamp,
                                                    triggered   = str(int(self.triggered)),
                                                    )
            dataRobot = self.templateDataRobot.format(
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
            dataRow = dataLeft + dataRobot


            # Go through the flies.                
            for iFly in range(self.nFlies):
                # Get the state of each fly.
                if len(arenastate.flies)>iFly:
                    stateFly = arenastate.flies[iFly]
                else:
                    stateFly = MsgFrameState()
    
                q = stateFly.pose.orientation
                rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                angleFly = rpy[2] % (2.0 * N.pi)

                # Append the fly to the data row.            
                dataFly = self.templateDataFly.format(
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
                dataRow += dataFly
                
            dataRow += '\n'
            

            # Write the robot & fly data to the file.
            with self.lockArenastate:
                if (not self.fid.closed):
                    self.fid.write(dataRow)

    
        if (self.initialized) and (self.bSavingVideo) and (self.image is not None):
            with self.lockVideo:
                # Convert ROS image to OpenCV image
                try:
                  cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(self.image, "passthrough"))
                except CvBridgeError, e:
                  print e
                # cv.CvtColor(cv_image, self.im_display, cv.CV_GRAY2RGB)
    
                self.WriteFilePng(cv_image)

    
    def Main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('Save', log_level=rospy.INFO)
    save = Save()
    save.Main()
    
