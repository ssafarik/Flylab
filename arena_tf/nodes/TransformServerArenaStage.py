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

        self.calibration =None 

        rospy.init_node('TransformServerStageArena')

         
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()

        self.subCalibrationOriginate = rospy.Subscriber('stage/calibration_originate', CalibrationStage, self.Calibration_callback)
        self.subCalibrationSet       = rospy.Subscriber('stage/calibration_set',       CalibrationStage, self.Calibration_callback)
        self.pubCalibrationSet       = rospy.Publisher('stage/calibration_set',        CalibrationStage, latch=True)
        self.selfPublishedCalibration = False

        rospy.Service('transformserverarenastage/init',            ExperimentParams, self.Init_callback)
        rospy.Service('transformserverarenastage/trial_start',     ExperimentParams, self.TrialStart_callback)
        rospy.Service('transformserverarenastage/trial_end',       ExperimentParams, self.TrialEnd_callback)
        rospy.Service('transformserverarenastage/trigger',         Trigger,          self.Trigger_callback)
        rospy.Service('transformserverarenastage/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)

        self.initConstructor = True


    # Receive calibration values (wherever they came from), and republish them (for the bag file).
    # Either:
    # -- file -> params -> calibration_originator.py -> here and bagfile.
    # or
    # -- bagfile -> here.
    def Calibration_callback (self, calibration):
        while (not self.initConstructor):
            rospy.loginfo('Waiting for initConstructor.')
            rospy.sleep(0.1)
        
            
        if (not self.selfPublishedCalibration):
            self.calibration = calibration
            
            ptStageArena = Point()
            qStageArena = Quaternion()
            
            ptStageArena.x = calibration.arena_x #rospy.get_param('stage/arena_x', 0.0)
            ptStageArena.y = calibration.arena_y #rospy.get_param('stage/arena_y', 0.0)
            ptStageArena.z = calibration.arena_z #rospy.get_param('stage/arena_z', 0.0)
                 
            qStageArena.x = calibration.arena_qx #rospy.get_param('stage/arena_qx', 0.0)
            qStageArena.y = calibration.arena_qy #rospy.get_param('stage/arena_qy', 0.0)
            qStageArena.z = calibration.arena_qz #rospy.get_param('stage/arena_qz', 0.0)
            qStageArena.w = calibration.arena_qw #rospy.get_param('stage/arena_qw', 1.0)

            self.transStageArena = (ptStageArena.x, 
                                    ptStageArena.y,
                                    ptStageArena.z)
            self.rotStageArena   = (qStageArena.x,
                                    qStageArena.y,
                                    qStageArena.z,
                                    qStageArena.w)
            
            self.initialized = True
        else:
            self.selfPublishedCalibration = False
        
        
    def Init_callback(self, experimentparams):
        return True

    # TrialStart_callback()
    # Publishes the calibration.
    #
    def TrialStart_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)

        if (self.calibration is not None):
            self.selfPublishedCalibration = True
            self.pubCalibrationSet.publish(self.calibration)
            
        return True
                
                
    def Trigger_callback(self, reqTrigger):
        return True
    
    def TrialEnd_callback(self, experimentparams):
        return True

    def WaitUntilDone_callback(self, experimentparams):
        return True
        
        
        
        
    def SendTransforms(self):
        if self.initialized:
            self.tfbx.sendTransform(self.transStageArena, 
                                    self.rotStageArena,
                                    rospy.Time.now(),
                                    "Stage",     # child
                                    "Arena"      # parent
                                    )
        
        
    def Main(self):
        rate = rospy.Rate(100) # BUG: Reducing this causes erratic behavior w/ patterngen & fivebar. 
        try:
            while not rospy.is_shutdown():
                try:
                    self.SendTransforms()
                    rate.sleep()
                except tf.Exception:
                    rospy.logwarn ('Exception in TransformServerStageArena()')
        except:
            print "Shutting down"


if __name__ == "__main__":
    tsps = TransformServerStageArena()
    tsps.Main()
    
