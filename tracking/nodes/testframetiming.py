#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo




###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourInfo message, and several intermediary images.
#
class TestFrameTiming:

    def __init__(self):
        
        # Image Initialization
        self.initialized = False
        
        # Messages,
        self.camerainfo = None
        self.subCameraInfo       = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        #self.subImageRect        = rospy.Subscriber("camera/image_rect", Image, self.ImageRect_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        self.subImageRaw         = rospy.Subscriber("camera/image_raw", Image, self.ImageRaw_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        
        self.timeRawPrev = rospy.Time.now().to_sec()
        self.timeRectPrev = rospy.Time.now().to_sec()
        self.initialized = True
        

    def CameraInfo_callback (self, camerainfo):
        if not self.initialized:
            return
        
        self.camerainfo = camerainfo
        

    def ImageRaw_callback(self, image):
        rospy.logwarn('ImageRaw_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timeRawPrev))
        self.timeRawPrev = rospy.Time.now().to_sec()


    def ImageRect_callback(self, image):
        rospy.logwarn('ImageRect_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timeRectPrev))
        self.timeRectPrev = rospy.Time.now().to_sec()


    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('TestFrameTiming')
    try:
        ip = TestFrameTiming()
        ip.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
