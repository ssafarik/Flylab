#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import sys
import rospy
import cv
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

class ImageDisplay:

    def __init__(self):
        self.image_sub = rospy.Subscriber("camera/image_rect", sensor_msgs.msg.Image, self.image_callback)

        cv.NamedWindow("Display",1)
        self.bridge = CvBridge()

        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.initialized_images = False

    def initialize_images(self,cv_image):
        self.im_display = cv.CreateImage(cv.GetSize(cv_image),cv.IPL_DEPTH_8U,3)
        self.initialized_images = True

    def image_callback(self,data):
        # Convert ROS image to OpenCV image
        try:
          cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
        except CvBridgeError, e:
          print e

        if not self.initialized_images:
            self.initialize_images(cv_image)

        cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

        cv.ShowImage("Display", self.im_display)
        cv.WaitKey(3)


if __name__ == '__main__':
    rospy.init_node('ImageDisplay')
    id = ImageDisplay()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()
