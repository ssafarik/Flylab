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
from experiment_srvs.srv import Trigger, ExperimentParams


def Chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


###############################################################################
# Save() is a ROS node.  It saves Image messages from a list of topics to image files (.png, .bmp, etc).
#
#  At the end of each trial, a video is made.  
# There should be one video frame per line in the .csv
#
class SaveBag:
    def __init__(self):
        self.initialized = False
        self.dirWorking_base = os.path.expanduser("~/FlylabData")
        Chdir(self.dirWorking_base)

        # Create new directory each day
        self.dirRelative = time.strftime("%Y_%m_%d")
        self.dirWorking = self.dirWorking_base + "/" + self.dirRelative
        Chdir(self.dirWorking)

        self.filename = None
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
        rospy.Service('savebag/init',            ExperimentParams, self.Init_callback)
        rospy.Service('savebag/trial_start',     ExperimentParams, self.TrialStart_callback)
        rospy.Service('savebag/trial_end',       ExperimentParams, self.TrialEnd_callback)
        rospy.Service('savebag/trigger',         Trigger,          self.Trigger_callback)
        rospy.Service('savebag/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        self.StopBag()
        

    # Service callback to perform initialization that requires experimentparams, e.g. subscribing to image topics.
    def Init_callback(self, experimentparams):
        rospy.logwarn('Init_callback')
        self.paramsSave = experimentparams.save
        
        return True



    def WaitUntilDone_callback(self, experimentparams):
        #self.processRosbag.wait()
        return True

            
                    
    # TrialStart_callback()
    # 
    def TrialStart_callback(self, experimentparams):
        rospy.logwarn('TrialStart_callback')
        while (not self.initialized):
            rospy.sleep(0.5)
            
        self.paramsSave = experimentparams.save
        self.bSaveOnlyWhileTriggered = self.paramsSave.onlyWhileTriggered
        
        if (self.paramsSave.bag):                
            # Get the directory:  dirBag = 'FlylabData/YYYY_MM_DD'
            self.dirBase = os.path.expanduser("~/FlylabData")
            self.dirBag = self.dirBase + "/" + time.strftime("%Y_%m_%d")
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
            self.StartBag()
            

            
        return True
                
                
    # Trigger_callback() 
    #    Set the trigger state, and detect edges.
    #
    def Trigger_callback(self, reqTrigger):
        rospy.logwarn('Trigger_callback %s' % reqTrigger.triggered)
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
            self.StartBag()
        
        if (bFallingEdge):
            self.StopBag()
        
        
        return self.bTriggered
        

    # TrialEnd_callback()
    # 
    def TrialEnd_callback(self, experimentparams):
        rospy.logwarn('TrialEnd_callback')
        while (not self.initialized):
            rospy.sleep(0.5)


        # Should we be saving?
        bSaveBagPrev = self.bSaveBag
        self.bSaveBag = False

        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveBagPrev) and (self.bSaveBag)
        bFallingEdge = (bSaveBagPrev) and (not self.bSaveBag)


        if (bRisingEdge):
            self.StartBag()
        
        if (bFallingEdge):
            self.StopBag()
                
                
        return True


    def StartBag(self):
        if (self.processRosbag is None):
            topic1 = 'camera/image_rect/compressed'
            topic2 = 'camera/image_backgroundinit'
            topic3 = 'camera/camera_info'
            cmdline = ['rosbag','record','-O',self.dirBag+'/'+self.filenameBag, topic1, topic2, topic3]
            self.processRosbag = subprocess.Popen(cmdline)

    
    def StopBag(self):
        if (self.processRosbag is not None):
            self.processRosbag.send_signal(subprocess.signal.SIGINT)
            self.processRosbag.wait()
            self.processRosbag = None
                

    def Main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('SaveBag', log_level=rospy.INFO)
    savebag = SaveBag()
    savebag.Main()
    
