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

        self.nFlies = rospy.get_param("nFlies", 0)
        self.typeFlies = rospy.get_param("fly/type", "unspecified")
        self.genderFlies = rospy.get_param("fly/gender", "unspecified")
        self.nRobots = rospy.get_param("nRobots", 0)
        self.widthRobot = rospy.get_param("robot/width", "3.175") # mm
        self.heightRobot = rospy.get_param("robot/height", "3.175") # mm
        self.visibleRobot = bool(rospy.get_param("robot/visible", "True"))
        self.paintRobot = str(rospy.get_param("robot/paint", "blackoxide"))
        self.scentRobot = str(rospy.get_param("robot/scent", "unscented"))

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

        self.headingsWaitEntry1 =   'waitEntry1\n'
        self.templateWaitEntry1 =   '{waitEntry1:s}\n'
        
        self.headingsTriggerEntry = 'trigger1Enabled, '\
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
        self.templateTriggerEntry = '{trigger1Enabled:s}, '\
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
                                    
        self.headingsWaitEntry2 =   'waitEntry2\n'
        self.templateWaitEntry2 =   '{waitEntry2:s}\n'
        
        self.headingsMoveRobot =    'moverobotEnabled, '\
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
                                    'moverobotRelTolerance, ' \
                                    'moverobotTimeout\n'
        self.templateMoveRobot =    '{moverobotEnabled:s}, '\
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
                                    '{moverobotRelTolerance:s}, ' \
                                    '{moverobotTimeout:s}\n'
                                    
        self.headingsLasertrack =   'laserEnabled, '\
                                    'laserPatternShape, '\
                                    'laserPatternHzPattern, '\
                                    'laserPatternHzPoint, '\
                                    'laserPatternCount, '\
                                    'laserPatternSizeX, '\
                                    'laserPatternSizeY, '\
                                    'laserPatternParam, '\
                                    'laserStatefilterLo, '\
                                    'laserStatefilterHi, '\
                                    'laserStatefilterCriteria, ' \
                                    'laserTimeout\n'
        self.templateLasertrack =   '{laserEnabled:s}, '\
                                    '{laserPatternShape:s}, '\
                                    '{laserPatternHzPattern:s}, '\
                                    '{laserPatternHzPoint:s}, '\
                                    '{laserPatternCount:s}, '\
                                    '{laserPatternSizeX:s}, '\
                                    '{laserPatternSizeY:s}, '\
                                    '{laserPatternParam:s}, '\
                                    '{laserStatefilterLo:s}, '\
                                    '{laserStatefilterHi:s}, '\
                                    '{laserStatefilterCriteria:s}, ' \
                                    '{laserTimeout:s}\n'
        
        self.headingsLEDPanels =    'ledpanelsEnabled, '\
                                    'ledpanelsCommand, '\
                                    'ledpanelsIdPattern, '\
                                    'ledpanelsFrameid, '\
                                    'ledpanelsStatefilterLo, '\
                                    'ledpanelsStatefilterHi, '\
                                    'ledpanelsStatefilterCriteria, '\
                                    'ledpanelsTimeout\n'
        self.templateLEDPanels =    '{ledpanelsEnabled:s}, '\
                                    '{ledpanelsCommand:s}, '\
                                    '{ledpanelsIdPattern:s}, '\
                                    '{ledpanelsFrameid:s}, '\
                                    '{ledpanelsStatefilterLo:s}, '\
                                    '{ledpanelsStatefilterHi:s}, '\
                                    '{ledpanelsStatefilterCriteria:s}, '\
                                    '{ledpanelsTimeout:s}\n'
        
        self.headingsTriggerExit =  'trigger2Enabled, '\
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
        self.templateTriggerExit =  '{trigger2Enabled:s}, '\
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
                                  
        self.headingsWaitExit =     'waitExit\n'
        self.templateWaitExit =     '{waitExit:s}\n'

        self.headingsData = 'time, xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, xFly, yFly, aFly, vxFly, vyFly, vaFly\n'
        self.templateData = '{time:0.4f}, {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}\n'

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
                    if (bFallingEdge) and (self.fid is not None) and (not self.fid.closed):
                        with self.lockArenastate:
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
        paramsTracking = self.templateTrackingA.format(trackingExclusionzoneEnabled = str(experimentparamsReq.tracking.exclusionzone.enabled))
        for i in range(len(experimentparamsReq.tracking.exclusionzone.point_list)):
            self.headingsTracking += self.headingsTrackingB
            paramsTracking += self.templateTrackingB.format(
                                                trackingExclusionzoneX       = str(experimentparamsReq.tracking.exclusionzone.point_list[i].x),
                                                trackingExclusionzoneY       = str(experimentparamsReq.tracking.exclusionzone.point_list[i].y),
                                                trackingExclusionzoneRadius  = str(experimentparamsReq.tracking.exclusionzone.radius_list[i]),
                                                )
        self.headingsTracking += '\n'
        paramsTracking += '\n'

        paramsWaitEntry1 = self.templateWaitEntry1.format(
                                                waitEntry1                  = str(experimentparamsReq.waitEntry1),
                                                )
        paramsTriggerEntry = self.templateTriggerEntry.format(
                                                trigger1Enabled            = str(experimentparamsReq.triggerEntry.enabled),
                                                trigger1FrameidParent      = str(experimentparamsReq.triggerEntry.frameidParent),
                                                trigger1FrameidChild       = str(experimentparamsReq.triggerEntry.frameidChild),
                                                trigger1SpeedAbsParentMin  = str(experimentparamsReq.triggerEntry.speedAbsParentMin),
                                                trigger1SpeedAbsParentMax  = str(experimentparamsReq.triggerEntry.speedAbsParentMax),
                                                trigger1SpeedAbsChildMin   = str(experimentparamsReq.triggerEntry.speedAbsChildMin),
                                                trigger1SpeedAbsChildMax   = str(experimentparamsReq.triggerEntry.speedAbsChildMax),
                                                trigger1SpeedRelMin        = str(experimentparamsReq.triggerEntry.speedRelMin),
                                                trigger1SpeedRelMax        = str(experimentparamsReq.triggerEntry.speedRelMax),
                                                trigger1DistanceMin        = str(experimentparamsReq.triggerEntry.distanceMin),
                                                trigger1DistanceMax        = str(experimentparamsReq.triggerEntry.distanceMax),
                                                trigger1AngleMin           = str(experimentparamsReq.triggerEntry.angleMin),
                                                trigger1AngleMax           = str(experimentparamsReq.triggerEntry.angleMax),
                                                trigger1AngleTest          = str(experimentparamsReq.triggerEntry.angleTest),
                                                trigger1AngleTestBilateral = str(experimentparamsReq.triggerEntry.angleTestBilateral),
                                                trigger1TimeHold           = str(experimentparamsReq.triggerEntry.timeHold),
                                                trigger1Timeout            = str(experimentparamsReq.triggerEntry.timeout),
                                                )
        paramsWaitEntry2 = self.templateWaitEntry2.format(
                                                waitEntry2                  = str(experimentparamsReq.waitEntry2),
                                                )
        paramsMoveRobot = self.templateMoveRobot.format(
                                                moverobotEnabled           = str(experimentparamsReq.move.enabled),
                                                moverobotPatternShape      = str(experimentparamsReq.move.pattern.shape),
                                                moverobotPatternHzPattern  = str(experimentparamsReq.move.pattern.hzPattern),
                                                moverobotPatternHzPoint    = str(experimentparamsReq.move.pattern.hzPoint),
                                                moverobotPatternCount      = str(experimentparamsReq.move.pattern.count),
                                                moverobotPatternSizeX      = str(experimentparamsReq.move.pattern.size.x),
                                                moverobotPatternSizeY      = str(experimentparamsReq.move.pattern.size.y),
                                                moverobotPatternParam      = str(experimentparamsReq.move.pattern.param),
                                                moverobotRelTracking       = str(experimentparamsReq.move.relative.tracking),
                                                moverobotRelOriginPosition = str(experimentparamsReq.move.relative.frameidOriginPosition),
                                                moverobotRelOriginAngle    = str(experimentparamsReq.move.relative.frameidOriginAngle),
                                                moverobotRelDistance       = str(experimentparamsReq.move.relative.distance),
                                                moverobotRelAngle          = str(experimentparamsReq.move.relative.angle),
                                                moverobotRelAngleType      = str(experimentparamsReq.move.relative.angleType),
                                                moverobotRelSpeed          = str(experimentparamsReq.move.relative.speed),
                                                moverobotRelSpeedType      = str(experimentparamsReq.move.relative.speedType),
                                                moverobotRelTolerance      = str(experimentparamsReq.move.relative.tolerance),
                                                moverobotTimeout           = str(experimentparamsReq.move.timeout),
                                                )
        
        if len(experimentparamsReq.lasertrack.pattern_list) > 0:
            patternShape           = str(experimentparamsReq.lasertrack.pattern_list[0].shape)
            patternHzPattern       = str(experimentparamsReq.lasertrack.pattern_list[0].hzPattern)
            patternHzPoint         = str(experimentparamsReq.lasertrack.pattern_list[0].hzPoint)
            patternCount           = str(experimentparamsReq.lasertrack.pattern_list[0].count)
            patternSizeX           = str(experimentparamsReq.lasertrack.pattern_list[0].size.x)
            patternSizeY           = str(experimentparamsReq.lasertrack.pattern_list[0].size.y)
            patternParam           = str(experimentparamsReq.lasertrack.pattern_list[0].param)
        else:
            patternShape           = "",
            patternHzPattern       = str(0.0),
            patternHzPoint         = str(0.0),
            patternCount           = str(0),
            patternSizeX           = str(0.0),
            patternSizeY           = str(0.0),
            patternParam           = str(0.0),

        if len(experimentparamsReq.lasertrack.statefilterHi_list) > 0:
            statefilterHi          = '\"' + str(experimentparamsReq.lasertrack.statefilterHi_list[0]) + '\"'
            statefilterLo          = '\"' + str(experimentparamsReq.lasertrack.statefilterLo_list[0]) + '\"'
            statefilterCriteria    = str(experimentparamsReq.lasertrack.statefilterCriteria_list[0])
        else:
            statefilterHi          = ""
            statefilterLo          = ""
            statefilterCriteria    = ""

        paramsLasertrack = self.templateLasertrack.format(
                                                laserEnabled               = str(experimentparamsReq.lasertrack.enabled),
                                                laserPatternShape          = patternShape,
                                                laserPatternHzPattern      = patternHzPattern,
                                                laserPatternHzPoint        = patternHzPoint,
                                                laserPatternCount          = patternCount,
                                                laserPatternSizeX          = patternSizeX,
                                                laserPatternSizeY          = patternSizeY,
                                                laserPatternParam          = patternParam,
                                                laserStatefilterHi         = statefilterHi,
                                                laserStatefilterLo         = statefilterLo,
                                                laserStatefilterCriteria   = statefilterCriteria,
                                                laserTimeout               = str(experimentparamsReq.lasertrack.timeout),
                                                )
            
        paramsLEDPanels = self.templateLEDPanels.format(
                                                ledpanelsEnabled             = str(experimentparamsReq.ledpanels.enabled),
                                                ledpanelsCommand             = str(experimentparamsReq.ledpanels.command),
                                                ledpanelsIdPattern           = str(experimentparamsReq.ledpanels.idPattern),
                                                ledpanelsFrameid             = str(experimentparamsReq.ledpanels.frame_id),
                                                ledpanelsStatefilterLo       = str(experimentparamsReq.ledpanels.statefilterLo),
                                                ledpanelsStatefilterHi       = str(experimentparamsReq.ledpanels.statefilterHi),
                                                ledpanelsStatefilterCriteria = str(experimentparamsReq.ledpanels.statefilterCriteria),
                                                ledpanelsTimeout             = str(experimentparamsReq.ledpanels.timeout),
                                                )
            
        paramsTriggerExit = self.templateTriggerExit.format(
                                                trigger2Enabled            = str(experimentparamsReq.triggerExit.enabled),
                                                trigger2FrameidParent      = str(experimentparamsReq.triggerExit.frameidParent),
                                                trigger2FrameidChild       = str(experimentparamsReq.triggerExit.frameidChild),
                                                trigger2SpeedAbsParentMin  = str(experimentparamsReq.triggerExit.speedAbsParentMin),
                                                trigger2SpeedAbsParentMax  = str(experimentparamsReq.triggerExit.speedAbsParentMax),
                                                trigger2SpeedAbsChildMin   = str(experimentparamsReq.triggerExit.speedAbsChildMin),
                                                trigger2SpeedAbsChildMax   = str(experimentparamsReq.triggerExit.speedAbsChildMax),
                                                trigger2SpeedRelMin        = str(experimentparamsReq.triggerExit.speedRelMin),
                                                trigger2SpeedRelMax        = str(experimentparamsReq.triggerExit.speedRelMax),
                                                trigger2DistanceMin        = str(experimentparamsReq.triggerExit.distanceMin),
                                                trigger2DistanceMax        = str(experimentparamsReq.triggerExit.distanceMax),
                                                trigger2AngleMin           = str(experimentparamsReq.triggerExit.angleMin),
                                                trigger2AngleMax           = str(experimentparamsReq.triggerExit.angleMax),
                                                trigger2AngleTest          = str(experimentparamsReq.triggerExit.angleTest),
                                                trigger2AngleTestBilateral = str(experimentparamsReq.triggerExit.angleTestBilateral),
                                                trigger2TimeHold           = str(experimentparamsReq.triggerExit.timeHold),
                                                trigger2Timeout            = str(experimentparamsReq.triggerExit.timeout),
                                                )
        paramsWaitExit = self.templateWaitExit.format(
                                                waitExit                   = str(experimentparamsReq.waitExit),
                                                )

        with self.lockArenastate:
            self.fid = open(self.filename, 'w')
            rospy.logwarn('SA logfile open(%s)' % self.filename)

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

            self.fid.write(self.headingsWaitEntry1)
            self.fid.write(paramsWaitEntry1)
            self.fid.write('\n')

            self.fid.write(self.headingsTriggerEntry)
            self.fid.write(paramsTriggerEntry)
            self.fid.write('\n')

            self.fid.write(self.headingsWaitEntry2)
            self.fid.write(paramsWaitEntry2)
            self.fid.write('\n')

            self.fid.write(self.headingsMoveRobot)
            self.fid.write(paramsMoveRobot)
            self.fid.write('\n')

            self.fid.write(self.headingsLasertrack)
            self.fid.write(paramsLasertrack)
            self.fid.write('\n')

            self.fid.write(self.headingsLEDPanels)
            self.fid.write(paramsLEDPanels)
            self.fid.write('\n')
            
            self.fid.write(self.headingsTriggerExit)
            self.fid.write(paramsTriggerExit)
            self.fid.write('\n')

            self.fid.write(self.headingsWaitExit)
            self.fid.write(paramsWaitExit)
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

            #rospy.logwarn ('SAVE %s' % [self.saveOnlyWhileTriggered,self.triggered,self.saveArenastate,self.bSavingArenastate])
            # Get the state of the robot.
            stateRobot = arenastate.robot
            q = stateRobot.pose.orientation
            rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            angleRobot = rpy[2] % (2.0 * N.pi)


            # Get the state of the fly.                
            iFly = 0 # Just use the first fly.
            if len(arenastate.flies)>iFly:
                stateFly = arenastate.flies[iFly]
            else:
                stateFly = MsgFrameState()

            q = stateFly.pose.orientation
            rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            angleFly = rpy[2] % (2.0 * N.pi)
            
            # Write the robot & fly data to the file.
            data_row = self.templateData.format(align   = self.format_align,
                                                sign    = self.format_sign,
                                                width   = self.format_width,
                                                precision = self.format_precision,
                                                type    = self.format_type,
                                                time    = rospy.Time.now().to_sec(), #stateRobot.header.stamp,
                                                xRobot  = stateRobot.pose.position.x,
                                                yRobot  = stateRobot.pose.position.y,
                                                aRobot  = angleRobot,
                                                vxRobot = stateRobot.velocity.linear.x,
                                                vyRobot = stateRobot.velocity.linear.y,
                                                vaRobot = stateRobot.velocity.angular.z,
                                                xFly    = stateFly.pose.position.x,
                                                yFly    = stateFly.pose.position.y,
                                                aFly    = angleFly,
                                                vxFly   = stateFly.velocity.linear.x,
                                                vyFly   = stateFly.velocity.linear.y,
                                                vaFly   = stateFly.velocity.angular.z,
                                                )

            with self.lockArenastate:
                self.fid.write(data_row)

    
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
    
