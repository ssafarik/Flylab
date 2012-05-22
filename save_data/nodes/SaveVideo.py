#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
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
        self.dirBase = os.path.expanduser("~/FlylabData")
        chdir(self.dirBase)
        self.dirVideo = self.dirBase + "/" + time.strftime("%Y_%m_%d")
        chdir(self.dirVideo)
        #self.dirFrames = self.dirVideo + "/frames"
        #chdir(self.dirFrames)
        # At this point we should be in ~/FlylabData/YYYYmmdd/images

        self.fileNull = open('/dev/null', 'w')
        self.lock = threading.Lock()


        self.subImage = rospy.Subscriber('camera/image_rect', Image, self.image_callback)
        rospy.Service('save/video/new_trial', ExperimentParams, self.NewTrial_callback)
        rospy.Service('save/video/trigger', Trigger, self.Trigger_callback)
        
        self.bridge = CvBridge()

        self.framerate = rospy.get_param("save/framerate", 30)
        #self.nRepeatFrames = int(rospy.get_param('save/video_image_repeat_count'))
        self.sizeImage = None
        
        self.reset_frames()
        rospy.on_shutdown(self.OnShutdown_callback)

        self.initialized = True


    def OnShutdown_callback(self):
        with self.lock:
            #self.reset_frames()
            if (self.fileNull is not None) and (not self.fileNull.closed):
                self.fileNull.close()
                self.fileNull = None



    def save_png(self, cv_image):
        if self.sizeImage is None:
            self.sizeImage = cv.GetSize(cv_image)
        filenameImage = self.dirFrames+"/{num:06d}.png".format(num=self.iFrame)
        cv.SaveImage(filenameImage, cv_image)
        self.iFrame += 1


    def image_callback(self, image):
        if (self.saveVideo) and (self.initialized) and (self.triggered):
            with self.lock:
                # Convert ROS image to OpenCV image
                try:
                  cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(image, "passthrough"))
                except CvBridgeError, e:
                  print e
                # cv.CvtColor(cv_image, self.im_display, cv.CV_GRAY2RGB)
    
                self.save_png(cv_image)


    def get_imagenames(self, dir):
        proc_ls_png = subprocess.Popen('ls ' + dir + '/*.png',
                                    shell=True,
                                    stdout=subprocess.PIPE,
                                    stderr=self.fileNull)
        out = proc_ls_png.stdout.readlines()
        imagenames = [s.rstrip() for s in out]
        return imagenames
    

    def save_video_from_frames(self):
        if (self.saveVideo) and (self.initialized):
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
                cmdCreateVideoFile = 'avconv -i ' + self.dirFrames + '/%06d.png ' + self.filenameVideo
                rospy.logwarn('Converting .png images to video using command:')
                rospy.logwarn (cmdCreateVideoFile)
                try:
                    subprocess.check_call(cmdCreateVideoFile, shell=True)
                except:
                    rospy.logerr('Exception running avconv')
                    
                rospy.logwarn('Saved %s' % (self.filenameVideo))
                #self.reset_frames()

            
    def reset_frames(self):
        try:
            subprocess.call('rm '+self.dirFrames+'/*.png')
        except OSError:
            pass
        
        self.iFrame = 0

            
        

    # Trigger_callback() 
    #    This gets called when the triggering state changes, either a trigger state has succeeded,
    #     or a trial run has concluded.
    #    Closes the log file if triggering state goes from True->False.
    #
    def Trigger_callback(self, reqTrigger):
        if self.triggered != reqTrigger.triggered: # Change in trigger state.
            self.triggered = reqTrigger.triggered
            
            if self.triggered: # Rising edge.
                self.reset_frames()
                
            # At the end of a trial (i.e. trigger falling edge), convert the frames to video.
            if (self.saveOnlyWhileTriggered) and (not self.triggered):
                self.save_video_from_frames()
            
        return self.triggered
        

    def NewTrial_callback(self, experimentparamsReq):
        #rospy.logwarn('savevideo new_trial callback')
        if self.initialized:
            self.experimentparams = experimentparamsReq
            
            self.saveOnlyWhileTriggered = True # Otherwise, there would be no interval to perform the conversion to video.  #self.experimentparams.save.onlyWhileTriggered
            self.saveVideo = self.experimentparams.save.video
            
            #self.filename = "%s%04d.csv" % (experimentparamsReq.save.filenamebase, experimentparamsReq.experiment.trial)
            now = rospy.Time.now().to_sec()
            self.dirFrames = "%s%04d%02d%02d%02d%02d%02d" % (self.experimentparams.save.filenamebase, 
                                                            time.localtime(now).tm_year,
                                                            time.localtime(now).tm_mon,
                                                            time.localtime(now).tm_mday,
                                                            time.localtime(now).tm_hour,
                                                            time.localtime(now).tm_min,
                                                            time.localtime(now).tm_sec)
            try:
                os.mkdir(self.dirFrames)
            except OSError:
                pass
            
            self.filenameVideo = "%s/%s%04d%02d%02d%02d%02d%02d.mov" % (self.dirVideo,
                                                            self.experimentparams.save.filenamebase, 
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

