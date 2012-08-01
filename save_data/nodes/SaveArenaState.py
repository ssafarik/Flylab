#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save_data')
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import sys
import time, os, subprocess
import threading
import numpy as N

from flycore.msg import MsgFrameState
from experiments.srv import Trigger, ExperimentParams
from tracking.msg import ArenaState


def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


class SaveArenaState:
    def __init__(self):
        self.initialized = False
        self.dirWorking_base = os.path.expanduser("~/FlylabData")
        chdir(self.dirWorking_base)

        # Create new directory each day
        self.dirRelative = time.strftime("%Y_%m_%d")
        self.dirWorking = self.dirWorking_base + "/" + self.dirRelative
        chdir(self.dirWorking)

        self.sub_arenastate = rospy.Subscriber("ArenaState", ArenaState, self.ArenaState_callback, queue_size=2)
        #self.sub_commandsavedata = rospy.Subscriber("CommandSavedata", CommandSavedata, self.commandsavedata_callback)
        #self.sub_experimentparams = rospy.Subscriber("ExperimentParams", ExperimentParams, self.NewTrial_callback)
        rospy.Service('save/arenastate/new_trial', ExperimentParams, self.NewTrial_callback)
        rospy.Service('save/arenastate/trigger', Trigger, self.Trigger_callback)
        self.tfrx = tf.TransformListener()
        self.lock = threading.Lock()
        
        self.save_count = 0
        self.filename = None
        self.fid = None
        self.triggered = False
        self.saveArenastate = False
        self.saveOnlyWhileTriggered = None

        self.robot_move_commanded = False

        self.robot_width = rospy.get_param("robot/width","3.175") # mm
        self.robot_height = rospy.get_param("robot/height","3.175") # mm
        self.robot_visible = bool(rospy.get_param("robot/visible","true"))
        self.robot_paint = str(rospy.get_param("robot/paint","blackoxide"))
        self.robot_scent = str(rospy.get_param("robot/scent","unscented"))

        self.format_align = ">"
        self.format_sign = " "
        self.format_width = "7"
        self.format_precision = "2"
        self.format_type = "f"
        
        self.headingsExperiment = 'date_time, '\
                                  'description, '\
                                  'maxTrials, '\
                                  'trial, '\
                                  'robot_width, '\
                                  'robot_height, '\
                                  'robot_visible, '\
                                  'robot_paint, '\
                                  'robot_scent, '\
                                  'waitEntry, '\
                                  'trigger1FrameidParent, '\
                                  'trigger1FrameidChild, '\
                                  'trigger1SpeedParentMin, '\
                                  'trigger1SpeedParentMax, '\
                                  'trigger1SpeedChildMin, '\
                                  'trigger1SpeedChildMax, '\
                                  'trigger1DistanceMin, '\
                                  'trigger1DistanceMax, '\
                                  'trigger1AngleMin, '\
                                  'trigger1AngleMax, '\
                                  'trigger1AngleTest, '\
                                  'trigger1AngleTestBilateral, '\
                                  'trigger1TimeHold, '\
                                  'movePatternShape, '\
                                  'movePatternHz, '\
                                  'movePatternHzPoint, '\
                                  'movePatternCount, '\
                                  'movePatternSizeX, '\
                                  'movePatternSizeY, '\
                                  'moveRelTracking, '\
                                  'moveRelOriginPosition, '\
                                  'moveRelOriginAngle, '\
                                  'moveRelDistance, '\
                                  'moveRelAngle, '\
                                  'moveRelAngleType, '\
                                  'moveRelSpeed, '\
                                  'moveRelSpeedType, '\
                                  'moveRelTolerance, '\
                                  'laserEnabled, '\
                                  'trigger2FrameidParent, '\
                                  'trigger2FrameidChild, '\
                                  'trigger2SpeedParentMin, '\
                                  'trigger2SpeedParentMax, '\
                                  'trigger2SpeedChildMin, '\
                                  'trigger2SpeedChildMax, '\
                                  'trigger2DistanceMin, '\
                                  'trigger2DistanceMax, '\
                                  'trigger2AngleMin, '\
                                  'trigger2AngleMax, '\
                                  'trigger2AngleTest, '\
                                  'trigger2AngleTestBilateral, '\
                                  'trigger2TimeHold\n'
                                  
        self.templateExperiment = '{date_time:s}, '\
                                  '{description:s}, '\
                                  '{maxTrials:s}, '\
                                  '{trial:s}, '\
                                  '{robot_width:s}, '\
                                  '{robot_height:s}, '\
                                  '{robot_visible:s}, '\
                                  '{robot_paint:s}, '\
                                  '{robot_scent:s}, '\
                                  '{waitEntry:s}, '\
                                  '{trigger1FrameidParent:s}, '\
                                  '{trigger1FrameidChild:s}, '\
                                  '{trigger1SpeedParentMin:s}, '\
                                  '{trigger1SpeedParentMax:s}, '\
                                  '{trigger1SpeedChildMin:s}, '\
                                  '{trigger1SpeedChildMax:s}, '\
                                  '{trigger1DistanceMin:s}, '\
                                  '{trigger1DistanceMax:s}, '\
                                  '{trigger1AngleMin:s}, '\
                                  '{trigger1AngleMax:s}, '\
                                  '{trigger1AngleTest:s}, '\
                                  '{trigger1AngleTestBilateral:s}, '\
                                  '{trigger1TimeHold:s}, '\
                                  '{trigger1Timeout:s}, '\
                                  '{movePatternShape:s}, '\
                                  '{movePatternHz:s}, '\
                                  '{movePatternHzPoint:s}, '\
                                  '{movePatternCount:s}, '\
                                  '{movePatternSizeX:s}, '\
                                  '{movePatternSizeY:s}, '\
                                  '{moveRelTracking:s}, '\
                                  '{moveRelOriginPosition:s}, '\
                                  '{moveRelOriginAngle:s}, '\
                                  '{moveRelDistance:s}, '\
                                  '{moveRelAngle:s}, '\
                                  '{moveRelAngleType:s}, '\
                                  '{moveRelSpeed:s}, '\
                                  '{moveRelSpeedType:s}, '\
                                  '{moveRelTolerance:s}, '\
                                  '{laserEnabled:s}, '\
                                  '{trigger2FrameidParent:s}, '\
                                  '{trigger2FrameidChild:s}, '\
                                  '{trigger2SpeedParentMin:s}, '\
                                  '{trigger2SpeedParentMax:s}, '\
                                  '{trigger2SpeedChildMin:s}, '\
                                  '{trigger2SpeedChildMax:s}, '\
                                  '{trigger2DistanceMin:s}, '\
                                  '{trigger2DistanceMax:s}, '\
                                  '{trigger2AngleMin:s}, '\
                                  '{trigger2AngleMax:s}, '\
                                  '{trigger2AngleTest:s}, '\
                                  '{trigger2AngleTestBilateral:s}, '\
                                  '{trigger2TimeHold:s}, '\
                                  '{trigger2Timeout:s}\n\n'

        self.headingsAbsolute = 'time, xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, xFly, yFly, aFly, vxFly, vyFly, vaFly, xRobotRel, yRobotRel, aRobotRel, dRobotRel\n'
        self.templateAbsolute   = '{time:0.4f}, {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}, {xRobotRel:{align}{sign}{width}.{precision}{type}}, {yRobotRel:{align}{sign}{width}.{precision}{type}}, {aRobotRel:{align}{sign}{width}.{precision}{type}}, {dRobotRel:{align}{sign}{width}.{precision}{type}}\n'
        self.templateAbsoluteNR = '{time:0.4f}, {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}\n'

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        if (self.fid is not None) and (not self.fid.closed):
            with self.lock:
                self.fid.close()
            rospy.logwarn('SA logfile close()')
            
        

    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
    #    Closes the log file if triggering state goes from True->False.
    #
    def Trigger_callback(self, reqTrigger):
        if self.triggered != reqTrigger.triggered:
            self.triggered = reqTrigger.triggered
            
            # Close the file is we're no longer saving.
            if (self.saveOnlyWhileTriggered) and (not self.triggered):
                if self.fid is not None and not self.fid.closed:
                    with self.lock:
                        self.fid.close()
                    rospy.logwarn('SA logfile close()')
            
        return self.triggered
        

    def NewTrial_callback(self, experimentparamsReq):
        if self.initialized:
            self.saveOnlyWhileTriggered = experimentparamsReq.save.onlyWhileTriggered
            self.saveArenastate = experimentparamsReq.save.arenastate
            
            if experimentparamsReq.save.arenastate:
                if self.fid is not None:
                    if not self.fid.closed:
                        with self.lock:
                            self.fid.close()
                        rospy.logwarn('SA logfile close()')

                #self.filename = "%s%04d.csv" % (experimentparamsReq.save.filenamebase, experimentparamsReq.experiment.trial)
                now = rospy.Time.now().to_sec()
                self.filename = "%s%04d%02d%02d%02d%02d%02d.csv" % (experimentparamsReq.save.filenamebase, 
                                                                    time.localtime(now).tm_year,
                                                                    time.localtime(now).tm_mon,
                                                                    time.localtime(now).tm_mday,
                                                                    time.localtime(now).tm_hour,
                                                                    time.localtime(now).tm_min,
                                                                    time.localtime(now).tm_sec)
                with self.lock:
                    self.fid = open(self.filename, 'w')
                rospy.logwarn('SA logfile open(%s)' % self.filename)
                with self.lock:
                    self.fid.write(self.headingsExperiment)
                header_row = self.templateExperiment.format(date_time                  = str(rospy.Time.now().to_sec()),
                                                            description                = str(experimentparamsReq.experiment.description),
                                                            maxTrials                  = str(experimentparamsReq.experiment.maxTrials),
                                                            trial                      = str(experimentparamsReq.experiment.trial),
                                                            robot_width                = str(self.robot_width),
                                                            robot_height               = str(self.robot_height),
                                                            robot_visible              = str(self.robot_visible),
                                                            robot_paint                = str(self.robot_paint),
                                                            robot_scent                = str(self.robot_scent),
                                                            waitEntry                  = str(experimentparamsReq.waitEntry),
                                                            trigger1FrameidParent      = str(experimentparamsReq.triggerEntry.frameidParent),
                                                            trigger1FrameidChild       = str(experimentparamsReq.triggerEntry.frameidChild),
                                                            trigger1SpeedParentMin     = str(experimentparamsReq.triggerEntry.speedParentMin),
                                                            trigger1SpeedParentMax     = str(experimentparamsReq.triggerEntry.speedParentMax),
                                                            trigger1SpeedChildMin      = str(experimentparamsReq.triggerEntry.speedChildMin),
                                                            trigger1SpeedChildMax      = str(experimentparamsReq.triggerEntry.speedChildMax),
                                                            trigger1DistanceMin        = str(experimentparamsReq.triggerEntry.distanceMin),
                                                            trigger1DistanceMax        = str(experimentparamsReq.triggerEntry.distanceMax),
                                                            trigger1AngleMin           = str(experimentparamsReq.triggerEntry.angleMin),
                                                            trigger1AngleMax           = str(experimentparamsReq.triggerEntry.angleMax),
                                                            trigger1AngleTest          = str(experimentparamsReq.triggerEntry.angleTest),
                                                            trigger1AngleTestBilateral = str(experimentparamsReq.triggerEntry.angleTestBilateral),
                                                            trigger1TimeHold           = str(experimentparamsReq.triggerEntry.timeHold),
                                                            trigger1Timeout            = str(experimentparamsReq.triggerEntry.timeout),
                                                            movePatternShape           = str(experimentparamsReq.move.pattern.shape),
                                                            movePatternHz              = str(experimentparamsReq.move.pattern.hzPattern),
                                                            movePatternHzPoint         = str(experimentparamsReq.move.pattern.hzPoint),
                                                            movePatternCount           = str(experimentparamsReq.move.pattern.count),
                                                            movePatternSizeX           = str(experimentparamsReq.move.pattern.size.x),
                                                            movePatternSizeY           = str(experimentparamsReq.move.pattern.size.y),
                                                            moveRelTracking            = str(experimentparamsReq.move.relative.tracking),
                                                            moveRelOriginPosition      = str(experimentparamsReq.move.relative.frameidOriginPosition),
                                                            moveRelOriginAngle         = str(experimentparamsReq.move.relative.frameidOriginAngle),
                                                            moveRelDistance            = str(experimentparamsReq.move.relative.distance),
                                                            moveRelAngle               = str(experimentparamsReq.move.relative.angle),
                                                            moveRelAngleType           = str(experimentparamsReq.move.relative.angleType),
                                                            moveRelSpeed               = str(experimentparamsReq.move.relative.speed),
                                                            moveRelSpeedType           = str(experimentparamsReq.move.relative.speedType),
                                                            moveRelTolerance           = str(experimentparamsReq.move.relative.tolerance),
                                                            laserEnabled               = str(experimentparamsReq.lasertrack.enabled),
                                                            trigger2FrameidParent      = str(experimentparamsReq.triggerExit.frameidParent),
                                                            trigger2FrameidChild       = str(experimentparamsReq.triggerExit.frameidChild),
                                                            trigger2SpeedParentMin     = str(experimentparamsReq.triggerExit.speedParentMin),
                                                            trigger2SpeedParentMax     = str(experimentparamsReq.triggerExit.speedParentMax),
                                                            trigger2SpeedChildMin      = str(experimentparamsReq.triggerExit.speedChildMin),
                                                            trigger2SpeedChildMax      = str(experimentparamsReq.triggerExit.speedChildMax),
                                                            trigger2DistanceMin        = str(experimentparamsReq.triggerExit.distanceMin),
                                                            trigger2DistanceMax        = str(experimentparamsReq.triggerExit.distanceMax),
                                                            trigger2AngleMin           = str(experimentparamsReq.triggerExit.angleMin),
                                                            trigger2AngleMax           = str(experimentparamsReq.triggerExit.angleMax),
                                                            trigger2AngleTest          = str(experimentparamsReq.triggerExit.angleTest),
                                                            trigger2AngleTestBilateral = str(experimentparamsReq.triggerExit.angleTestBilateral),
                                                            trigger2TimeHold           = str(experimentparamsReq.triggerExit.timeHold),
                                                            trigger2Timeout            = str(experimentparamsReq.triggerExit.timeout),
                                                            )

                with self.lock:
                    self.fid.write(header_row)
                    self.fid.write(self.headingsAbsolute)

        return True
    
    

    def ArenaState_callback(self, arenastate):
        if self.initialized and (self.fid is not None):
            bSave = True
            if (self.saveOnlyWhileTriggered and not self.triggered) or not self.saveArenastate:
                bSave = False

            #rospy.logwarn ('SAVE %s' % [self.saveOnlyWhileTriggered,self.triggered,self.saveArenastate,bSave])
            if bSave:                    
                if self.fid.closed:
                    with self.lock:
                        self.fid = open(self.filename, 'wa')
                    rospy.logwarn('SA logfile open2(%s)' % self.filename)
                    
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
                
                # Get the robot data relative to the fly.
                isGoodRelative = False
                try:
                    poses = PoseStamped(header=stateRobot.header, pose=stateRobot.pose)
                    #self.tfrx.waitForTransform("Fly1", poses.header.frame_id, rospy.Time(), rospy.Duration(0.1))
                    robotRelativeToFly = self.tfrx.transformPose("Fly1", poses)
                    xRobotRel = robotRelativeToFly.pose.position.x
                    yRobotRel = robotRelativeToFly.pose.position.y
                    aRobotRel = N.arctan2(yRobotRel,xRobotRel) % (2.0*N.pi)
                    dRobotRel = N.sqrt(xRobotRel**2 + yRobotRel**2)
                    isGoodRelative = True
                except (tf.Exception):
                    xRobotRel = 0.0
                    yRobotRel = 0.0
                    aRobotRel = 0.0
                    dRobotRel = 0.0
                
                # Write the robot & fly data to the file.
                if isGoodRelative:
                    data_row = self.templateAbsolute.format(align   = self.format_align,
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
                                                            xRobotRel = xRobotRel,
                                                            yRobotRel = yRobotRel,
                                                            aRobotRel = aRobotRel,
                                                            dRobotRel = dRobotRel
                                                            )
                else:
                    data_row = self.templateAbsoluteNR.format(align   = self.format_align,
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
                                                            vaFly   = stateFly.velocity.angular.z
                                                            )
                    
    
                with self.lock:
                    self.fid.write(data_row)


if __name__ == '__main__':
    rospy.init_node('SaveArenaState', log_level=rospy.INFO)
    sk = SaveArenaState()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

