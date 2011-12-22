#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time,os,subprocess

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveImages:
    def __init__(self):
        self.working_dir_base = os.path.expanduser("~/Videos")
        chdir(self.working_dir_base)

        self.image_frame = rospy.get_param("save_image_frame")
        self.image_sub = rospy.Subscriber(self.image_frame, Image, self.image_callback)

        self.bridge = CvBridge()
        self.image_number = 0
        self.frame_rate = rospy.get_param("framerate","30")
        self.video_format = rospy.get_param("save_video_format","flv")

        self.saving_images_started = False
        self.last_image_time = None
        self.rate = rospy.Rate(10)     # Hz
        self.time_limit = 3

        # self.color_max = 255
        # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        # self.video_initialized = False

    def save_png(self,cv_image):
        image_name = "{num:06d}.png".format(num=self.image_number)
        cv.SaveImage(image_name,cv_image)

    # def write_frame(self,cv_image):
    #     cv.WriteFrame(self.video_writer,cv_image)

    # def initialize_video(self,cv_image):
    #     self.video_writer = cv.CreateVideoWriter(self.working_dir+".mjpg",
    #                                              cv.CV_FOURCC('M','J','P','G'),
    #                                              self.frame_rate,
    #                                              cv.GetSize(cv_image))

        # self.im_display = cv.CreateImage(cv.GetSize(cv_image),cv.IPL_DEPTH_8U,3)
        # self.video_initialized = True

    def image_callback(self,data):
        if not self.saving_images_started:
            self.images_dir = time.strftime("%Y-%m-%d-%H-%M-%S")
            self.working_dir = self.working_dir_base + "/" + self.images_dir
            chdir(self.working_dir)
            self.saving_images_started = True

        self.last_image_time = rospy.get_time()
        # Convert ROS image to OpenCV image
        try:
          cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
        except CvBridgeError, e:
          print e

        self.save_png(cv_image)

        # if self.video_format in "png":
        #     self.save_png(cv_image)
        # else:
        #     if not self.video_initialized:
        #         self.initialize_video(cv_image)
        #     self.write_frame(cv_image)

        rospy.loginfo("Saved image {num:06d}\n".format(num=self.image_number))
        self.image_number += 1
        # if not self.images_initialized:
        #     self.initialize_images(cv_image)

        # cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

    def save_video(self):
        self.saving_images_started = False
        chdir(self.working_dir_base)
        saved_video_str = 'Saved video ' + self.images_dir
        if self.video_format in "flv":
            subprocess.check_call(['ffmpeg','-f','image2',
                                   '-i',self.images_dir+'/%06d.png',
                                   '-sameq',
                                   '-ar','44100',
                                   '-ab','64k',
                                   '-ac','2',
                                   '-r',str(self.frame_rate),
                                   '-s','640x480',
                                   self.images_dir+'.flv'])
            saved_video_str = saved_video_str + '.flv'
        elif self.video_format in "gif":
            subprocess.check_call(['ffmpeg','-f','image2',
                                   '-i',self.images_dir+'/%06d.png',
                                   '-sameq',
                                   '-r',str(self.frame_rate),
                                   '-s','640x480',
                                   '-pix_fmt','rgb24',
                                   self.images_dir+'.gif'])
            saved_video_str = saved_video_str + '.gif'
        elif self.video_format in "avi":
            subprocess.check_call(['ffmpeg','-f','image2',
                                   '-i',self.images_dir+'/%06d.png',
                                   '-sameq',
                                   '-r',str(self.frame_rate),
                                   '-s','640x480',
                                   self.images_dir+'.avi'])
            saved_video_str = saved_video_str + '.avi'
        elif self.video_format in "mpeg1":
            subprocess.check_call(['ffmpeg','-f','image2',
                                   '-i',self.images_dir+'/%06d.png',
                                   '-sameq',
                                   '-r',str(self.frame_rate),
                                   '-s','640x480',
                                   '-mbd','rd',
                                   '-trellis','2',
                                   '-cmp','2',
                                   '-subcmp','2',
                                   '-pass','1/2',
                                   self.images_dir+'.mpg'])
            saved_video_str = saved_video_str + '.mpg'

        rospy.logwarn(saved_video_str)

    def main(self):
        while not rospy.is_shutdown():
            if self.saving_images_started and (self.last_image_time is not None):
                t = rospy.get_time()
                dt = t - self.last_image_time
                # rospy.logwarn("dt = %s" % (str(dt)))
                if self.time_limit < dt:
                    self.save_video()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SaveImages',log_level=rospy.INFO)
    si = SaveImages()
    si.main()
