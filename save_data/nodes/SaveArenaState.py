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

        self.sub_arenastate = rospy.Subscriber("ArenaState", ArenaState, self.ArenaState_callback)
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
        
        self.headingsExperiment = 'date_time, description, maxTrials, trial, waitEntry, trigger1DistanceMin, trigger1DistanceMax, trigger1SpeedMin, trigger1SpeedMax, trigger1AngleMin, trigger1AngleMax, trigger1AngleTest, trigger1AngleTestBilateral, trigger1TimeHold, trigger2DistanceMin, trigger2DistanceMax, trigger2SpeedMin, trigger2SpeedMax, trigger2AngleMin, trigger2AngleMax, trigger2AngleTest, trigger2AngleTestBilateral, trigger2TimeHold, robot_width, robot_height, robot_visible, robot_paint, robot_scent\n'
        self.templateExperiment = '{date_time:s}, {description:s}, {maxTrials:>4d}, {trial:>4d}, {waitEntry:> 7.4f}, {trigger1DistanceMin:> 7.4f}, {trigger1DistanceMax:> 7.4f}, {trigger1SpeedMin:> 7.4f}, {trigger1SpeedMax:> 7.4f}, {trigger1AngleMin:> 7.4f}, {trigger1AngleMax:> 7.4f}, {trigger1AngleTest:s}, {trigger1AngleTestBilateral:>d}, {trigger1TimeHold:> 7.4f}, {trigger2DistanceMin:> 7.4f}, {trigger2DistanceMax:> 7.4f}, {trigger2SpeedMin:> 7.4f}, {trigger2SpeedMax:> 7.4f}, {trigger2AngleMin:> 7.4f}, {trigger2AngleMax:> 7.4f}, {trigger2AngleTest:s}, {trigger2AngleTestBilateral:>d}, {trigger2TimeHold:> 7.4f}, {robot_width:>6.3f}, {robot_height:>6.3f}, {robot_visible:>d}, {robot_paint:s}, {robot_scent:s}\n\n'

        self.headingsAbsolute = 'time, xRobot, yRobot, aRobot, vxRobot, vyRobot, vaRobot, xFly, yFly, aFly, vxFly, vyFly, vaFly, xRobotRel, yRobotRel, aRobotRel, dRobotRel\n'
        self.templateAbsolute = '{time:0.4f}, {xRobot:{align}{sign}{width}.{precision}{type}}, {yRobot:{align}{sign}{width}.{precision}{type}}, {aRobot:{align}{sign}{width}.{precision}{type}}, {vxRobot:{align}{sign}{width}.{precision}{type}}, {vyRobot:{align}{sign}{width}.{precision}{type}}, {vaRobot:{align}{sign}{width}.{precision}{type}}, {xFly:{align}{sign}{width}.{precision}{type}}, {yFly:{align}{sign}{width}.{precision}{type}}, {aFly:{align}{sign}{width}.{precision}{type}}, {vxFly:{align}{sign}{width}.{precision}{type}}, {vyFly:{align}{sign}{width}.{precision}{type}}, {vaFly:{align}{sign}{width}.{precision}{type}}, {xRobotRel:{align}{sign}{width}.{precision}{type}}, {yRobotRel:{align}{sign}{width}.{precision}{type}}, {aRobotRel:{align}{sign}{width}.{precision}{type}}, {dRobotRel:{align}{sign}{width}.{precision}{type}}\n'
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
                                                            description                = experimentparamsReq.experiment.description,
                                                            maxTrials                  = experimentparamsReq.experiment.maxTrials,
                                                            trial                      = experimentparamsReq.experiment.trial,
                                                            waitEntry                  = experimentparamsReq.waitEntry,
                                                            trigger1DistanceMin        = experimentparamsReq.triggerEntry.distanceMin,
                                                            trigger1DistanceMax        = experimentparamsReq.triggerEntry.distanceMax,
                                                            trigger1SpeedMin           = experimentparamsReq.triggerEntry.speedMin,
                                                            trigger1SpeedMax           = experimentparamsReq.triggerEntry.speedMax,
                                                            trigger1AngleMin           = experimentparamsReq.triggerEntry.angleMin,
                                                            trigger1AngleMax           = experimentparamsReq.triggerEntry.angleMax,
                                                            trigger1AngleTest          = experimentparamsReq.triggerEntry.angleTest,
                                                            trigger1AngleTestBilateral = experimentparamsReq.triggerEntry.angleTestBilateral,
                                                            trigger1TimeHold           = experimentparamsReq.triggerEntry.timeHold,
                                                            trigger2DistanceMin        = experimentparamsReq.triggerExit.distanceMin,
                                                            trigger2DistanceMax        = experimentparamsReq.triggerExit.distanceMax,
                                                            trigger2SpeedMin           = experimentparamsReq.triggerExit.speedMin,
                                                            trigger2SpeedMax           = experimentparamsReq.triggerExit.speedMax,
                                                            trigger2AngleMin           = experimentparamsReq.triggerExit.angleMin,
                                                            trigger2AngleMax           = experimentparamsReq.triggerExit.angleMax,
                                                            trigger2AngleTest          = experimentparamsReq.triggerExit.angleTest,
                                                            trigger2AngleTestBilateral = experimentparamsReq.triggerExit.angleTestBilateral,
                                                            trigger2TimeHold           = experimentparamsReq.triggerExit.timeHold,
                                                            robot_width                = self.robot_width,
                                                            robot_height               = self.robot_height,
                                                            robot_visible              = self.robot_visible,
                                                            robot_paint                = str(self.robot_paint),
                                                            robot_scent                = str(self.robot_scent)
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

