#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import numpy as N
import tf
from arena_tf.msg import CalibrationStage
from geometry_msgs.msg import Point, Quaternion
from experiment_srvs.srv import Trigger, ExperimentParams


class TransformServerStageArena:
    def __init__(self):
        self.initialized = False
        self.initConstructor = False

        self.calibration_stage = None

        rospy.init_node('TransformServerStageArena')
        rospy.sleep(1)

         
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.subCalibrationOriginate = rospy.Subscriber('stage/calibration_originate', CalibrationStage, self.Calibration_callback) # This topic brings in data, but it doesn't get recorded.
        self.subCalibrationSet       = rospy.Subscriber('stage/calibration_set',       CalibrationStage, self.Calibration_callback) # This topic brings in data, and gets recorded into the .bag file if being saved.
        self.pubCalibrationSet       = rospy.Publisher('stage/calibration_set',        CalibrationStage, latch=True)                # We republish the data at each trial.

        self.services = {}
        self.services['transformserverarenastage/init']            = rospy.Service('transformserverarenastage/init',            ExperimentParams, self.Init_callback)
        self.services['transformserverarenastage/trial_start']     = rospy.Service('transformserverarenastage/trial_start',     ExperimentParams, self.TrialStart_callback)
        self.services['transformserverarenastage/trial_end']       = rospy.Service('transformserverarenastage/trial_end',       ExperimentParams, self.TrialEnd_callback)
        self.services['transformserverarenastage/trigger']         = rospy.Service('transformserverarenastage/trigger',         Trigger,          self.Trigger_callback)
        self.services['transformserverarenastage/wait_until_done'] = rospy.Service('transformserverarenastage/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)

        self.initConstructor = True


    # Receive calibration values (wherever they came from), and republish them (for the bag file).
    # Either:
    # -- file -> params -> OriginateCalibrationStage.py -> here.
    # or
    # -- bagfile -> here.
    def Calibration_callback (self, calibration):
        while (not self.initConstructor):
            rospy.loginfo('Waiting for initConstructor.')
            rospy.sleep(0.1)
        
            
        self.calibration_stage = calibration
        self.initialized = True
        
        
    def Init_callback(self, experimentparams):
        return True

    # TrialStart_callback()
    # Publishes the calibration.
    #
    def TrialStart_callback(self, experimentparams):
        while (self.calibration_stage is None):
            rospy.sleep(0.5)

        self.pubCalibrationSet.publish(self.calibration_stage)
            
        return True
                
                
    def Trigger_callback(self, reqTrigger):
        return True
    
    def TrialEnd_callback(self, experimentparams):
        return True

    def WaitUntilDone_callback(self, experimentparams):
        return True
        
        
        
        
    def SendTransforms(self):
        if (self.initialized):
            self.tfbx.sendTransform((self.calibration_stage.arena_x,
                                     self.calibration_stage.arena_y,
                                     self.calibration_stage.arena_z),
                                    (self.calibration_stage.arena_qx,
                                     self.calibration_stage.arena_qy,
                                     self.calibration_stage.arena_qz,
                                     self.calibration_stage.arena_qw),
                                    rospy.Time.now(),
                                    "Stage",     # child
                                    "Arena"      # parent
                                    )
        
        
    def Main(self):
        rate = rospy.Rate(100) # BUG: Reducing this causes erratic behavior w/ patterngen & fivebar. 
        while not rospy.is_shutdown():
            #try:
            self.SendTransforms()
            rate.sleep()
            #except (tf.Exception, rospy.exceptions.ROSInterruptException), e:
            #    pass # ROS shutting down.

        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            


if __name__ == "__main__":
    tsps = TransformServerStageArena()
    tsps.Main()
    
