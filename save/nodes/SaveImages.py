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



###############################################################################
# Save() is a ROS node.  It saves Image messages from a list of topics to image files (.png, .bmp, etc).
#
#  At the end of each trial, a video is made.  
# There should be one video frame per line in the .csv
#
class SaveImages:
    def __init__(self):
        self.initialized = False

        # Create new directory each day
        self.dirBase = os.path.expanduser('~/FlylabData')
        self.dirWorking = self.dirBase + '/' + time.strftime('%Y_%m_%d')

        # Make sure dir exists.
        try:
            os.makedirs(self.dirWorking)
        except OSError:
            pass
        


        self.fileErrors = open('/dev/null', 'w')

        self.imageext = rospy.get_param('save/imageext', 'png')
        
        self.cvbridge = CvBridge()
        self.lockImage = threading.Lock()
        self.lockVideo = threading.Lock()
        self.filename = None
        self.fid = None
        self.bSaveImages = False
        self.bSaveOnlyWhileTriggered = False # False: Save everything from one trial_start to the trial_end.  True:  Save everything from trigger=on to trigger=off.
        self.bTriggered = False
        #self.imagetopic = rospy.get_param('save/imagetopic', 'camera/image_rect')
        self.paramsSave = None

        # All the per-topic stuff.
        self.subImage_dict = {}          # All the subscriptions.
        self.fullpathVideo_dict = {}     # The video filename of each imagetopic.
        self.dirFrames = {}         # The directory of each imagetopic.
        self.iFrame = {}            # The frame counter for each imagetopic.
        self.processVideoConversion = {}

        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Offer some services.
        rospy.Service('saveimages/init',            ExperimentParams, self.Init_callback)
        rospy.Service('saveimages/trial_start',     ExperimentParams, self.TrialStart_callback)
        rospy.Service('saveimages/trial_end',       ExperimentParams, self.TrialEnd_callback)
        rospy.Service('saveimages/trigger',         Trigger,          self.Trigger_callback)
        rospy.Service('saveimages/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        with self.lockImage:
            if (self.fileErrors is not None) and (not self.fileErrors.closed):
                self.fileErrors.close()
                self.fileErrors = None
            
        

    # Service callback to perform initialization that requires experimentparams, e.g. subscribing to image topics.
    def Init_callback(self, experimentparams):
        self.paramsSave = experimentparams.save
        
        for imagetopic in self.paramsSave.imagetopic_list:
            rospy.loginfo('Subscribing to %s' % imagetopic)
            self.subImage_dict[imagetopic] = rospy.Subscriber(imagetopic, Image, self.Image_callback, callback_args=imagetopic)

        return True



    def WaitUntilDone_callback(self, experimentparams):
        for imagetopic,value in self.processVideoConversion.iteritems():
            self.processVideoConversion[imagetopic].wait()

        return True

            
                    
    # TrialStart_callback()
    # 
    def TrialStart_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)
            
        self.paramsSave = experimentparams.save
        self.bSaveOnlyWhileTriggered = self.paramsSave.onlyWhileTriggered
        
        if (self.paramsSave.mov):                
            # Get the directory:  dirWorking = '~/FlylabData/YYYY_MM_DD'
            self.dirWorking = self.dirBase + '/' + time.strftime('%Y_%m_%d')
            
            # Make sure dir exists.
            try:
                os.makedirs(self.dirWorking)
            except OSError:
                pass

                
            # Initialize vars for each imagetopic.
            # Make a subdir and video filename for each timestamped imagetopic:  dirFrames['camera/image_raw'] = '/home/user/FlylabData/YYYY_MM_DD/test20130418131415_camera_image_raw'
            for (imagetopic,value) in self.subImage_dict.iteritems():
                self.dirFrames[imagetopic] = self.dirWorking + '/' + self.paramsSave.filenamebase + self.paramsSave.timestamp + '_' + imagetopic.replace('/','_')
                self.fullpathVideo_dict[imagetopic] = '%s.mov' % (self.dirFrames[imagetopic]) 
                self.iFrame[imagetopic] = 0
    
                try:
                    os.mkdir(self.dirFrames[imagetopic])
                except OSError, e:
                    rospy.logwarn ('Cannot create directory %s: %s' % (self.dirFrames[imagetopic],e))
        
                
            # Should we be saving?
            bSaveImagesPrev = self.bSaveImages
            if (self.paramsSave.mov) and ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
                self.bSaveImages = True
            else:
                self.bSaveImages = False
    
            # Edge detection for save start/stop.            
            bRisingEdge = (not bSaveImagesPrev) and (self.bSaveImages)
            bFallingEdge = (bSaveImagesPrev) and (not self.bSaveImages)
            

            
        return True
                
                
    # Trigger_callback() 
    #    Set the trigger state, and detect edges.
    #
    def Trigger_callback(self, reqTrigger):
        while (not self.initialized):
            rospy.sleep(0.5)

        self.bTriggered = reqTrigger.triggered

            
        # Should we be saving?
        bSaveImagesPrev = self.bSaveImages
        if (self.paramsSave.mov) and ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            self.bSaveImages = True
        else:
            self.bSaveImages = False

        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveImagesPrev) and (self.bSaveImages)
        bFallingEdge = (bSaveImagesPrev) and (not self.bSaveImages)

        
        return self.bTriggered
        

    # TrialEnd_callback()
    # 
    def TrialEnd_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)

        # Should we be saving?
        bSaveImagesPrev = self.bSaveImages
        if (self.paramsSave.mov) and ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            self.bSaveImages = True
        else:
            self.bSaveImages = False

        # Edge detection for save start/stop.            
        bRisingEdge = (not bSaveImagesPrev) and (self.bSaveImages)
        bFallingEdge = (bSaveImagesPrev) and (not self.bSaveImages)

            
        # If there are videos to make, then make them.
        for (imagetopic,fullpathVideo) in self.fullpathVideo_dict.iteritems():
            self.WriteVideoFromFrames(fullpathVideo, imagetopic)
            self.iFrame[imagetopic] = 0
            
        
        self.fullpathVideo_dict = {}
                
                
        return True
                
                
    def Image_callback(self, image, imagetopic):
        if (self.initialized) and (self.bSaveImages) and (image is not None):
            with self.lockImage:
                # Convert ROS image to OpenCV image
                try:
                  matImage = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, 'passthrough'))
                except CvBridgeError, e:
                  print e
                # cv.CvtColor(matImage, self.im_display, cv.CV_GRAY2RGB)
    
                self.WriteImageFile(matImage, imagetopic)

        
    def get_imagenames(self, dir):
        proc_ls = subprocess.Popen('ls %s/*.%s' % (dir, self.imageext),
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileErrors)
        out = proc_ls.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    def WriteImageFile(self, matImage, imagetopic):
        if (imagetopic in self.dirFrames):
            filenameImage = self.dirFrames[imagetopic]+'/{num:06d}.{ext:s}'.format(num=self.iFrame[imagetopic], ext=self.imageext)
    
            cv.SaveImage(filenameImage, matImage)
            self.iFrame[imagetopic] += 1


    def WriteVideoFromFrames(self, fullpathVideo, imagetopic):
        with self.lockVideo:
            if (imagetopic in self.dirFrames):
                # Run avconv, and then remove all the png files.
                cmdCreateVideoFile = 'avconv -r 60 -i %s/%%06d.%s -same_quant -r 60 %s && rm -rf %s' % (self.dirFrames[imagetopic], self.imageext, fullpathVideo, self.dirFrames[imagetopic])
                rospy.logwarn('Converting images to video using command:')
                rospy.logwarn (cmdCreateVideoFile)
                try:
                    self.processVideoConversion[imagetopic] = subprocess.Popen(cmdCreateVideoFile, shell=True)
                except:
                    rospy.logerr('Exception running avconv')
                



    def Main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('SaveImages', log_level=rospy.INFO)
    saveimages = SaveImages()
    saveimages.Main()
    
