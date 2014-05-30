#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save')
import rospy

import tf
import sys
import time, os, subprocess
import threading
import numpy as N

from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge, CvBridgeError

from flycore.msg import MsgFrameState
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsChoices



###############################################################################
#
class SaveBag:
    def __init__(self):
        self.initialized = False

        # Create new directory each day
        self.dirBase = os.path.expanduser('~/FlylabData')
        self.dirBag = self.dirBase + '/' + time.strftime('%Y_%m_%d')

        # Make sure path exists.
        try:
            os.makedirs(self.dirBag)
        except OSError:
            pass


        self.filenameBag = None
        self.fid = None
        self.bSaveBag = False
        self.bSaveOnlyWhileTriggered = False # False: Save everything from one trial_start to the trial_end.  True:  Save everything from trigger=on to trigger=off.
        self.bTriggered = False
        #self.imagetopic = rospy.get_param('save/imagetopic', 'camera/image_rect')
        self.paramsSave = None

        # All the per-topic stuff.
        self.processRosbag = None

        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Offer some services.
        self.services = {}
        self.services['savebag/init']            = rospy.Service('savebag/init',            ExperimentParamsChoices, self.Init_callback)
        self.services['savebag/trial_start']     = rospy.Service('savebag/trial_start',     ExperimentParams,        self.TrialStart_callback)
        self.services['savebag/trial_end']       = rospy.Service('savebag/trial_end',       ExperimentParams,        self.TrialEnd_callback)
        self.services['savebag/trigger']         = rospy.Service('savebag/trigger',         Trigger,                 self.Trigger_callback)
        self.services['savebag/wait_until_done'] = rospy.Service('savebag/wait_until_done', ExperimentParams,        self.WaitUntilDone_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        self.StopRecordingBag()
        

    # Service callback to perform initialization that requires experimentparams, e.g. subscribing to image topics.
    def Init_callback(self, experimentparamsChoices):
        self.paramsSave = experimentparamsChoices.save
        
        return True



    def WaitUntilDone_callback(self, experimentparams):
        #self.processRosbag.wait()
        return True

            
                    
    # TrialStart_callback()
    # 
    def TrialStart_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)
            
        self.paramsSave = experimentparams.save
        self.bSaveOnlyWhileTriggered = self.paramsSave.onlyWhileTriggered
        
        if (self.paramsSave.bag):                
            # Get the directory:  dirBag = 'FlylabData/YYYY_MM_DD'
            self.dirBag = self.dirBase + '/' + time.strftime('%Y_%m_%d')

            # Make sure path exists.
            try:
                os.makedirs(self.dirBag)
            except OSError:
                pass
            
            self.filenameBag = '%s%s.bag' % (self.paramsSave.filenamebase, self.paramsSave.timestamp) 
    
        
        # Should we be saving?
        bSaveBagPrev = self.bSaveBag
        if (self.paramsSave.bag) and ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            self.bSaveBag = True
        else:
            self.bSaveBag = False

            
        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveBagPrev) and (self.bSaveBag)
        bFallingEdge = (bSaveBagPrev) and (not self.bSaveBag)


        if (bRisingEdge):
            self.StartRecordingBag()
            

            
        return True
                
                
    # Trigger_callback() 
    #    Set the trigger state, and detect edges.
    #
    def Trigger_callback(self, reqTrigger):
        while (not self.initialized):
            rospy.sleep(0.5)

        self.bTriggered = reqTrigger.triggered


        # Should we be saving?
        bSaveBagPrev = self.bSaveBag
        if (self.paramsSave.bag) and ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            self.bSaveBag = True
        else:
            self.bSaveBag = False

        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveBagPrev) and (self.bSaveBag)
        bFallingEdge = (bSaveBagPrev) and (not self.bSaveBag)


        if (bRisingEdge):
            self.StartRecordingBag()
        
        if (bFallingEdge):
            self.StopRecordingBag()
        
        
        return self.bTriggered
        

    # TrialEnd_callback()
    # 
    def TrialEnd_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)


        # Should we be saving?
        bSaveBagPrev = self.bSaveBag
        self.bSaveBag = False

        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveBagPrev) and (self.bSaveBag)
        bFallingEdge = (bSaveBagPrev) and (not self.bSaveBag)


        if (bRisingEdge):
            self.StartRecordingBag()
        
        if (bFallingEdge):
            self.StopRecordingBag()
                
                
        return True


    def StartRecordingBag(self):
        if (self.processRosbag is None):
            rospy.logwarn('Saving bag file: %s' % (self.dirBag+'/'+self.filenameBag))
            topic1 = 'camera/image_background_set'
            topic2 = 'camera/calibration_set'
            topic3 = 'stage/calibration_set'
            topic4 = 'camera/camera_info'
            topic5 = 'camera/image_rect/compressed'
            topic6 = 'tracking/command'
            topic7 = 'end_effector'
            cmdline = ['rosbag', 'record','-O', self.dirBag+'/'+self.filenameBag, topic1, topic2, topic3, topic4, topic5, topic6, topic7]
            self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)

    
    def StopRecordingBag(self):
        if (self.processRosbag is not None):
            #self.processRosbag.send_signal(subprocess.signal.SIGINT)
            subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
            self.processRosbag.wait()
            rospy.logwarn('Closed bag file.')
            self.processRosbag = None
                

    def Main(self):
        rospy.spin()

#         # Shutdown all the services we offered.
#         for key in self.services:
#             self.services[key].shutdown()
            
        
        

if __name__ == '__main__':
    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag()
    savebag.Main()
    
