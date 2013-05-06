#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import cv2
import copy
import numpy as N
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from flycore.msg import TrackingCommand




###############################################################################
###############################################################################
###############################################################################
# The class BackgroundImage simply reads and writes a background image file.
# The purpose is that since the background is input to the system, we want 
# to be able to change whether we use the image coming from disk, or the image 
# coming from a .bag file.
#
class BackgroundImage:

    def __init__(self):

        self.initialized = False        
        self.bUseBackgroundSubtraction  = rospy.get_param('tracking/usebackgroundsubtraction', True)    # Set False to turn off bg subtraction.        
        
        # Messages
        self.subTrackingCommand     = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        self.pubImageBackgroundInit = rospy.Publisher("camera/image_backgroundinit", Image, latch=True)     # Published each time the background image file gets read.
        
        self.filenameBackground = os.path.expanduser(rospy.get_param('tracking/filenameBackground', '~/background.png'))

        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.cvbridge = CvBridge()
        

        # Load the background image file, or if no file then use the first one from the camera.
        self.matBackground  = cv2.imread(self.filenameBackground, cv.CV_LOAD_IMAGE_GRAYSCALE)
        if self.matBackground is None:
            self.matBackground = self.SubscribeOneImage('camera/image_rect')
            
            bSuccess = cv2.imwrite(self.filenameBackground, self.matBackground)
            rospy.logwarn ('Saving new background image %s:  %s.' % (self.filenameBackground, 'Succeeded' if bSuccess else 'FAILED'))
            
        self.PublishBackgroundInit(self.matBackground)
            
        self.initialized = True


    def SubscribeOneImage(self, topic):
        queue_size_images = rospy.get_param('tracking/queue_size_images', 1)
        subImage = rospy.Subscriber(topic, Image, self.Image_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True)
        self.matImage = None
        while (self.matImage is None):
            rospy.loginfo('Waiting for image on %s' % topic)
        del subImage

        return self.matImage
        

    def Image_callback(self, image):
        try:
            self.matImage = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, "passthrough")))
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting ROS image to opencv:  %s' % e)
        
        
    def PublishBackgroundInit(self, imgBackground):
        try:
            image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(imgBackground), "passthrough")
            image2.header.stamp = rospy.Time.now() #image.header.stamp
            self.pubImageBackgroundInit.publish(image2)
            del image2
        except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
            rospy.logwarn ('Exception in PublishBackgroundInit(): %s' % e)
            

        
    # TrackingCommand_callback()
    # Receives commands to change the tracking behavior.
    # See TrackingCommand.msg for details.
    #
    def TrackingCommand_callback(self, trackingcommand):
        #with self.lock:
            if trackingcommand.command == 'save_background':
                rospy.logwarn ('Saving new background image %s' % self.filenameBackground)
                self.matBackground = self.SubscribeOneImage('camera/image_background')
                cv2.imwrite(self.filenameBackground, self.matBackground)

    
    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('BackgroundImage')
    try:
        bg = BackgroundImage()
        bg.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
  
