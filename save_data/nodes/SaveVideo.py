#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save_data')
import rospy

import sys
import time,os,subprocess
import cv
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from experiments.srv import Trigger, ExperimentParams

LOGLEVEL = rospy.INFO


def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)


class SaveVideo:
    def __init__(self):
        self.initialized = False
        self.triggered = False

        self.lock = threading.Lock()
        self.fileNull = open('/dev/null', 'w')
        self.iFrame = 0

        self.subImage = rospy.Subscriber('camera/image_processed', Image, self.Image_callback)
        rospy.Service('save/video/new_trial', ExperimentParams, self.NewTrial_callback)
        rospy.Service('save/video/trigger', Trigger, self.Trigger_callback)
        
        self.bridge = CvBridge()

        self.filenameVideo = None
        self.saveVideo = False
        self.saveOnlyWhileTriggered = False # False: Save everything from one new_trial to the next new_trial.  True:  Save everything from trigger=on to trigger=off.
        self.triggered = False
        self.bSaving = False

        self.framerate = rospy.get_param("save/framerate", 30)
        #self.nRepeatFrames = int(rospy.get_param('save/video_image_repeat_count'))
        self.sizeImage = None
        
        #self.ResetFrameCounter()
        rospy.on_shutdown(self.OnShutdown_callback)

        self.initialized = True


    def OnShutdown_callback(self):
        with self.lock:
            if (self.fileNull is not None) and (not self.fileNull.closed):
                self.fileNull.close()
                self.fileNull = None


    def get_imagenames(self, dir):
        proc_ls_png = subprocess.Popen('ls ' + dir + '/*.png',
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileNull)
        out = proc_ls_png.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    def WriteVideoFromFrames(self):
        with self.lock:
            
            # Rewrite all the image files, with duplicate frames to simulate slow-motion.
            #chdir(self.dirImage)
            #imagenames = self.get_imagenames(self.dirImage)
            #iFrame = 0
            #for imagename in imagenames:
            #    chdir(self.dirFrames)
            #    image = cv.LoadImage(imagename)
            #    chdir(self.dirFrames2)
            #    for iRepeat in range(self.nRepeatFrames):
            #        filenameImage = "{num:06d}.png".format(num=iFrame)
            #        cv.SaveImage(filenameImage, image)
            #        iFrame += 1
    
#            cmdCreateVideoFile = 'ffmpeg -f image2 -i ' + self.dirFrames + '/%06d.png -r ' + str(self.framerate) + ' ' + \
#                                   '-sameq -s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -g 100 -bf 2 -pass 1/2 ' + \
#                                   self.filenameVideo
            cmdCreateVideoFile = 'avconv -i ' + self.dirFrames + '/%06d.png -r' + str(self.framerate) + self.filenameVideo
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


    def WriteFilePng(self, cv_image):
        if self.sizeImage is None:
            self.sizeImage = cv.GetSize(cv_image)
        filenameImage = self.dirFrames+"/{num:06d}.png".format(num=self.iFrame)
        cv.SaveImage(filenameImage, cv_image)
        self.iFrame += 1


    def Image_callback(self, image):
        if (self.initialized) and (self.bSaving):
            with self.lock:
                # Convert ROS image to OpenCV image
                try:
                  cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(image, "passthrough"))
                except CvBridgeError, e:
                  print e
                # cv.CvtColor(cv_image, self.im_display, cv.CV_GRAY2RGB)
    
                self.WriteFilePng(cv_image)


    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
    #
    def Trigger_callback(self, reqTrigger):
        if (self.initialized) and (self.saveVideo):
            
            if (self.saveOnlyWhileTriggered):
                if (reqTrigger.triggered):
                    self.bSaving = True
                else:
                    self.bSaving = False
            
        
            bRisingEdge = False
            bFallingEdge = False
            if self.triggered != reqTrigger.triggered:
                self.triggered = reqTrigger.triggered
                if self.triggered: # Rising edge.
                    bRisingEdge = True
                else:
                    bFallingEdge = True
                    
            if (self.saveOnlyWhileTriggered) and (bRisingEdge):
                self.ResetFrameCounter()
                    
            # At the end of a run, convert the frames to video.
            if (self.saveOnlyWhileTriggered) and (bFallingEdge):
                self.WriteVideoFromFrames()
                            
        return self.triggered
        

    def NewTrial_callback(self, experimentparamsReq):
        self.saveVideo = experimentparamsReq.save.video
        if (self.initialized) and (self.saveVideo):
            self.saveOnlyWhileTriggered = experimentparamsReq.save.onlyWhileTriggered
           #self.saveOnlyWhileTriggered = True # Otherwise, there would be no interval to perform the conversion to video.  #experimentparamsReq.save.onlyWhileTriggered
            
            # If there are prior frames to convert, then convert them.
            if (self.filenameVideo is not None):
                self.WriteVideoFromFrames()
            
            self.ResetFrameCounter()

            # Determine if we should be saving.
            if (not self.saveOnlyWhileTriggered):
                self.bSaving = True
            else:
                self.bSaving = False
            
            
            now = rospy.Time.now().to_sec()
            self.dirFrames = "%s%04d%02d%02d%02d%02d%02d" % (experimentparamsReq.save.filenamebase, 
                                                            time.localtime(now).tm_year,
                                                            time.localtime(now).tm_mon,
                                                            time.localtime(now).tm_mday,
                                                            time.localtime(now).tm_hour,
                                                            time.localtime(now).tm_min,
                                                            time.localtime(now).tm_sec)

            self.dirBase = os.path.expanduser("~/FlylabData")
            chdir(self.dirBase)
            self.dirVideo = self.dirBase + "/" + time.strftime("%Y_%m_%d")
            chdir(self.dirVideo)
            #self.dirFrames = self.dirVideo + "/frames"
            #chdir(self.dirFrames)
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
    
    
    def main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('SaveVideo', log_level=LOGLEVEL)
    sv = SaveVideo()
    sv.main()

