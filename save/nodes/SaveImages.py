#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save')
import rospy

import tf
import sys
import time, os, subprocess
import threading
import numpy as np

from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge, CvBridgeError
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat

from flycore.msg import MsgFrameState
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsChoices



###############################################################################
# Save() is a ROS node.  It saves Image messages from a list of topics to image files (.png, .bmp, etc).
#
#  At the end of each trial, a video is made.  
# There should be one video frame per line in the .csv
#
class SaveImages:
    def __init__(self):
        self.bInitConstructor = False

        # Create new directory each day
        self.dirBase = os.path.expanduser('~/FlylabData')
        self.dirImages = self.dirBase + '/' + time.strftime('%Y_%m_%d')

        # Make sure dir exists.
        try:
            os.makedirs(self.dirImages)
        except OSError:
            pass
        


        self.fileErrors = open('/dev/null', 'w')

        self.imageext = rospy.get_param('save/imageext', 'png')
        
        self.cvbridge = CvBridge()
        self.lockImage = threading.Lock()
        self.lockMov = threading.Lock()
        self.filename = None
        self.fid = None
        self.bSaveImages = False    # When true, each image gets saved to a .png (or other) file.
        self.bSaveFmf = False       # When true, each image gets added to an .fmf video file.
        self.bSaveOnlyWhileTriggered = False # False: Save everything from one trial_start to the trial_end.  True:  Save everything from trigger=on to trigger=off.
        self.bTriggered = False
        #self.imagetopic = rospy.get_param('save/imagetopic', 'camera/image_rect')
        self.paramsSave = None

        
        
        
        # All the per-topic stuff.
        self.subImage_dict = {}          # All the subscriptions.
        self.fullpathMov_dict = {}     # The video filename of each imagetopic.
        self.fullpathFmf_dict = {}     # The video filename of each imagetopic.
        self.dirFrames = {}         # The directory of each imagetopic.
        self.iFrame = {}            # The frame counter for each imagetopic.
        self.processMovConversion = {}
        self.fmf_dict = {}

        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Offer some services.
        self.services = {}
        self.services['saveimages/init']            = rospy.Service('saveimages/init',            ExperimentParamsChoices, self.Init_callback)
        self.services['saveimages/trial_start']     = rospy.Service('saveimages/trial_start',     ExperimentParams,        self.TrialStart_callback)
        self.services['saveimages/trial_end']       = rospy.Service('saveimages/trial_end',       ExperimentParams,        self.TrialEnd_callback)
        self.services['saveimages/trigger']         = rospy.Service('saveimages/trigger',         Trigger,                 self.Trigger_callback)
        self.services['saveimages/wait_until_done'] = rospy.Service('saveimages/wait_until_done', ExperimentParamsChoices, self.WaitUntilDone_callback)
        
        self.bInitConstructor = True


    def OnShutdown_callback(self):
        with self.lockImage:
            if (self.fileErrors is not None) and (not self.fileErrors.closed):
                self.fileErrors.close()
                self.fileErrors = None
            
        

    # Service callback to perform initialization, e.g. subscribing to image topics.
    def Init_callback(self, experimentparamsChoices):
        self.paramsSave = experimentparamsChoices.save
        
        for imagetopic in self.paramsSave.imagetopic_list:
            rospy.loginfo('Subscribing to %s' % imagetopic)
            self.subImage_dict[imagetopic] = rospy.Subscriber(imagetopic, Image, self.Image_callback, callback_args=imagetopic)

        return True



    def WaitUntilDone_callback(self, experimentparamsChoices):
        for imagetopic,value in self.processMovConversion.iteritems():
            self.processMovConversion[imagetopic].wait()

        return True

            
                    
    # TrialStart_callback()
    # 
    def TrialStart_callback(self, experimentparams):
        while (not self.bInitConstructor):
            rospy.sleep(0.5)
            
        self.paramsSave = experimentparams.save
        self.bSaveOnlyWhileTriggered = self.paramsSave.onlyWhileTriggered

        
        
        # Set flags for saving images.
        bSaveImagesPrev = self.bSaveImages
        self.bSaveImages = False
        self.bSaveFmf = False
        if ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            # Should we save image files for a subsequent movie?
            if (self.paramsSave.mov):
                self.bSaveImages = True

            # Should we save .fmf files?
            if (self.paramsSave.fmf):
                self.bSaveFmf = True

        #rospy.logwarn('paramsSave.fmf=%s, bSaveFmf=%s' % (self.paramsSave.fmf, self.bSaveFmf))
        
        # Make a directory for the images:  dirImages = '~/FlylabData/YYYY_MM_DD'
        self.dirImages = self.dirBase + '/' + time.strftime('%Y_%m_%d')
        try:
            os.makedirs(self.dirImages)
        except OSError:
            pass

                
        # Initialize vars for each imagetopic.
        # Make a subdir and video filename for each timestamped imagetopic:  dirFrames['camera/image_raw'] = '/home/user/FlylabData/YYYY_MM_DD/test20130418131415_camera_image_raw'
        for (imagetopic,value) in self.subImage_dict.iteritems():
            self.dirFrames[imagetopic] = self.dirImages + '/' + self.paramsSave.filenamebase + self.paramsSave.timestamp + '_' + imagetopic.replace('/','_')
            self.iFrame[imagetopic] = 0
            
            if (self.paramsSave.mov):                
                self.fullpathMov_dict[imagetopic] = '%s.mov' % (self.dirFrames[imagetopic])

                try:
                    os.mkdir(self.dirFrames[imagetopic])
                except OSError, e:
                    rospy.logwarn ('Cannot create directory %s: %s' % (self.dirFrames[imagetopic],e))
            
            
            if (self.paramsSave.fmf):                
                self.fullpathFmf_dict[imagetopic] = '%s.fmf' % (self.dirFrames[imagetopic])
                self.fmf_dict = {}
                


        
                
        return True
                
                
    # Trigger_callback() 
    #    Set the trigger state, and detect edges.
    #
    def Trigger_callback(self, reqTrigger):
        while (not self.bInitConstructor):
            rospy.sleep(0.5)

        self.bTriggered = reqTrigger.triggered

            
        # Should we be saving?
        bSaveImagesPrev = self.bSaveImages
        self.bSaveImages = False
        self.bSaveFmf = False
        if ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            # Should we save image files for a subsequent movie?
            if (self.paramsSave.mov):
                self.bSaveImages = True

            # Should we save .fmf files?
            if (self.paramsSave.fmf):
                self.bSaveFmf = True

        
        return self.bTriggered
        

    # TrialEnd_callback()
    # 
    def TrialEnd_callback(self, experimentparams):
        while (not self.bInitConstructor):
            rospy.sleep(0.5)

        # Should we be saving?
        bSaveImagesPrev = self.bSaveImages
        self.bSaveImages = False
        self.bSaveFmf = False
        if ((self.bSaveOnlyWhileTriggered and self.bTriggered) or (not self.bSaveOnlyWhileTriggered)):
            # Should we save image files for a subsequent movie?
            if (self.paramsSave.mov):
                self.bSaveImages = True

            # Should we save .fmf files?
            if (self.paramsSave.fmf):
                self.bSaveFmf = True

            
        # If there are videos to make, then make them.
        for (imagetopic,fullpathMov) in self.fullpathMov_dict.iteritems():
            self.WriteMovFromFrames(fullpathMov, imagetopic)
            self.iFrame[imagetopic] = 0
            
        
        self.fullpathMov_dict = {}
        self.fullpathFmf_dict = {}
        
        for imagetopic,fmf in self.fmf_dict.iteritems():
            fmf.close()                
            
        self.fmf_dict = {}
        
                
        return True
                
                
    def Image_callback(self, image, imagetopic):
        if (self.bInitConstructor) and (image is not None):
            if (self.bSaveImages):
                self.WriteImageToFile(image, imagetopic)

            if (self.bSaveFmf):
                self.AddImageToFmf(image, imagetopic)


                
    def get_imagenames(self, dir):
        proc_ls = subprocess.Popen('ls %s/*.%s' % (dir, self.imageext),
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileErrors)
        out = proc_ls.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    # WriteImageToFile()
    # Write the given image to the file specified by the imagetopic, using the extension from self.imageext.
    #
    def WriteImageToFile(self, image, imagetopic):
        if (imagetopic in self.dirFrames):
            filenameImage = self.dirFrames[imagetopic]+'/{num:06d}.{ext:s}'.format(num=self.iFrame[imagetopic], ext=self.imageext)
    
            matImage = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, 'passthrough'))
            cv.SaveImage(filenameImage, matImage)
            self.iFrame[imagetopic] += 1


    # AddImageToFmf()
    # Add the given image to the .fmf file specified by the imagetopic.
    #
    def AddImageToFmf(self, image, imagetopic):
        if (imagetopic not in self.fmf_dict):
            self.fmf_dict[imagetopic] = FlyMovieFormat.FlyMovieSaver(self.fullpathFmf_dict[imagetopic], 
                                                                     version=3, 
                                                                     format=image.encoding.upper(), 
                                                                     bits_per_pixel = int(8 * image.step / image.width))
            
        # Cast the pixels to the proper type, then add the frame to the .fmf
        pixels = np.array(image.data, 'c').view(np.uint8).reshape((image.height, image.width))
        self.fmf_dict[imagetopic].add_frame(pixels, image.header.stamp.to_sec())

        
    # WriteMovFromFrames()
    # Convert a directory full of image files into a single .mov file.
    #
    def WriteMovFromFrames(self, fullpathMov, imagetopic):
        with self.lockMov:
            if (imagetopic in self.dirFrames):
                # Run avconv, and then remove all the png files.
                cmdCreateMovFile = 'avconv -r 60 -i %s/%%06d.%s -same_quant -r 60 %s && rm -rf %s' % (self.dirFrames[imagetopic], self.imageext, fullpathMov, self.dirFrames[imagetopic])
                rospy.logwarn('Converting images to video using command:')
                rospy.logwarn (cmdCreateMovFile)
                try:
                    self.processMovConversion[imagetopic] = subprocess.Popen(cmdCreateMovFile, shell=True)
                except:
                    rospy.logerr('Exception running avconv')
                



    def Main(self):
        rospy.spin()

#        # Shutdown all the services we offered.
#        for key in self.services:
#            self.services[key].shutdown()

        

if __name__ == '__main__':
    rospy.init_node('SaveImages', log_level=rospy.INFO)
    rospy.sleep(1)
    saveimages = SaveImages()
    saveimages.Main()
    
