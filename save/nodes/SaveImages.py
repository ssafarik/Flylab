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
from experiments.srv import Trigger, ExperimentParams


def Chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


###############################################################################
# Save() is a ROS node.  It saves Image messages to image files (.png, .bmp, etc).
#
#  At the end of each trial, a video is made.  
# There should be one video frame per line in the .csv
#
class SaveImages:
    def __init__(self):
        self.initialized = False
        self.dirWorking_base = os.path.expanduser("~/FlylabData")
        Chdir(self.dirWorking_base)


        # Create new directory each day
        self.dirRelative = time.strftime("%Y_%m_%d")
        self.dirWorking = self.dirWorking_base + "/" + self.dirRelative
        Chdir(self.dirWorking)

        self.triggered = False
        self.saveOnlyWhileTriggered = False # False: Save everything from one trial_start to the trial_end.  True:  Save everything from trigger=on to trigger=off.

        rospy.Service('saveimages/trial_start', ExperimentParams, self.TrialStart_callback)
        rospy.Service('saveimages/trial_end', ExperimentParams, self.TrialEnd_callback)
        rospy.Service('saveimages/trigger', Trigger, self.Trigger_callback)
        rospy.Service('saveimages/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)

        
        self.fileNull = open('/dev/null', 'w')
        self.iFrame = 0

        self.imageext = rospy.get_param('save/imageext', 'png')
        self.imagetopic = rospy.get_param('save/imagetopic', 'camera/image_rect')
        self.subImage = rospy.Subscriber(self.imagetopic, Image, self.Image_callback)
        
        self.cvbridge = CvBridge()

        self.image = None
        self.filenameVideo = None
        self.saveVideo = False
        self.bSavingVideo = False
        self.processVideoConversion = None

        self.sizeImage = None

        
        
        self.lockVideo = threading.Lock()
        
        self.filename = None
        self.fid = None

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.initialized = True


    def OnShutdown_callback(self):
        with self.lockVideo:
            if (self.fileNull is not None) and (not self.fileNull.closed):
                self.fileNull.close()
                self.fileNull = None
            
        

    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
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


            if (self.saveOnlyWhileTriggered) and (self.saveVideo):
                if (reqTrigger.triggered):
                    self.bSavingVideo = True
                else:
                    self.bSavingVideo = False
            
        
            # At the end of a run, close the file if we're no longer saving.
            if (self.saveOnlyWhileTriggered):
                if (self.saveVideo):                    
                    if (bRisingEdge):
                        self.ResetFrameCounter()
                            
                    if (bFallingEdge) and (self.filenameVideo is not None):
                        self.WriteVideoFromFrames()
                    
            
        return self.triggered
        

    # TrialStart_callback()
    # 
    def TrialStart_callback(self, experimentparamsReq):
        self.saveVideo = experimentparamsReq.save.video
        
        if (self.initialized):
            self.saveOnlyWhileTriggered = experimentparamsReq.save.onlyWhileTriggered

            if (self.saveVideo):
                self.ResetFrameCounter()
    
                # Determine if we should be saving.
                if (self.saveOnlyWhileTriggered):
                    self.bSavingVideo = False
                else:
                    self.bSavingVideo = True
                
                
                self.dirFrames = experimentparamsReq.save.filenamepart
                
    
                self.dirBase = os.path.expanduser("~/FlylabData")
                Chdir(self.dirBase)
                self.dirVideo = self.dirBase + "/" + time.strftime("%Y_%m_%d")
                Chdir(self.dirVideo)
                
                try:
                    os.mkdir(self.dirFrames)
                except OSError, e:
                    rospy.logwarn ('Cannot create directory %s: %s' % (self.dirFrames,e))
    
            
                self.filenameVideo = "%s/%s.mov" % (self.dirVideo, experimentparamsReq.save.filenamepart) 

                
        return True
                
                
    # TrialEnd_callback()
    # 
    def TrialEnd_callback(self, experimentparamsReq):
        if (self.initialized):
            if (self.saveVideo):
                # If there are prior frames to convert, then convert them.
                if (self.filenameVideo is not None):
                    self.WriteVideoFromFrames()
                
        return True
                
                
    def get_imagenames(self, dir):
        proc_ls = subprocess.Popen('ls %s/*.%s' % (dir, self.imageext),
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileNull)
        out = proc_ls.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    def WriteVideoFromFrames(self):
        with self.lockVideo:
            
            cmdCreateVideoFile = 'avconv -r 60 -i %s/%%06d.%s -r 60 %s' % (self.dirFrames, self.imageext, self.filenameVideo)
            rospy.logwarn('Converting images to video using command:')
            rospy.logwarn (cmdCreateVideoFile)
            try:
                self.processVideoConversion = subprocess.Popen(cmdCreateVideoFile, shell=True)
            except:
                rospy.logerr('Exception running avconv')
                
            rospy.logwarn('Saved %s' % (self.filenameVideo))
            self.filenameVideo = None



    def WaitUntilDone_callback(self, experimentparams):
        if self.processVideoConversion is not None:
            self.processVideoConversion.wait()

        return True

            
                    
    def ResetFrameCounter(self):
        #try:
        #    rospy.logwarn('Deleting frame images.')
        #    subprocess.call('rm '+self.dirFrames+'/*.png', shell=True)
        #except OSError:
        #    pass
        
        self.iFrame = 0


    def WriteImageFile(self, cvimage):
        if self.sizeImage is None:
            self.sizeImage = cv.GetSize(cvimage)
        filenameImage = self.dirFrames+"/{num:06d}.{ext:s}".format(num=self.iFrame, ext=self.imageext)

        cv.SaveImage(filenameImage, cvimage)
        self.iFrame += 1


    def Image_callback(self, image):
        self.image = image

        if (self.initialized) and (self.bSavingVideo) and (self.image is not None):
            with self.lockVideo:
                # Convert ROS image to OpenCV image
                try:
                  cv_image = cv.GetImage(self.cvbridge.imgmsg_to_cv(self.image, "passthrough"))
                except CvBridgeError, e:
                  print e
                # cv.CvtColor(cv_image, self.im_display, cv.CV_GRAY2RGB)
    
                self.WriteImageFile(cv_image)

        


    def Main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('Save', log_level=rospy.INFO)
    saveimages = SaveImages()
    saveimages.Main()
    
