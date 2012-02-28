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
from save_data.msg import BagInfo, VideoInfo

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveVideo:
    def __init__(self):
        self.initialized = False
        self.working_dir_base = os.path.expanduser("~/Videos")
        chdir(self.working_dir_base)
        # self.working_dir_base = self.working_dir_base + "/" + time.strftime("%Y-%m-%d")
        # chdir(self.working_dir_base)

        self.working_dir = None

        self.bag_info_sub = rospy.Subscriber("bag_info",BagInfo,self.bag_info_callback)
        self.bag_info = BagInfo()

        self.saving_images = False
        self.saving_video = False
        self.ready_to_save_video = False
        self.saved_video = False
        self.saved_cat_video = False

        self.video_info_pub = rospy.Publisher("video_info",VideoInfo)
        self.video_info = VideoInfo()
        self.video_info.ready_for_bag_info = True
        self.video_info.ready_to_record = False
        self.video_info.saved_video = False

        self.bag_name = ""

        self.image_frame = rospy.get_param("save_image_frame")
        self.image_sub = rospy.Subscriber(self.image_frame, Image, self.image_callback)

        self.cat = rospy.get_param("video_cat",False)

        self.bridge = CvBridge()
        self.image_number = 0
        self.frame_rate = rospy.get_param("framerate",30)
        self.video_format = rospy.get_param("save_video_format","flv")

        # self.saving_images_started = False
        # self.last_image_time = None
        self.rate = rospy.Rate(10)     # Hz
        # self.time_limit = 3

        self.black_image_count = int(rospy.get_param("black_image_count",0))
        self.black_images_saved = False
        self.im_black = None
        self.im_size = None

        self.NULL = open('/dev/null', 'w')

        self.repeat_count = int(rospy.get_param('video_image_repeat_count'))
        # rospy.logwarn("repeat count = %s" % (str(self.repeat_count)))

        self.initialized = True

        # self.color_max = 255
        # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        # self.video_initialized = False

    def bag_info_callback(self,data):
        if self.initialized:
            self.bag_info = data
            bag_name = data.bag_name
            ready_to_play = data.ready_to_play
            finished_playing = data.finished_playing
            end_of_bag_files = data.end_of_bag_files

            if end_of_bag_files:
                if self.cat and (not self.saved_cat_video) and (self.bag_name != ""):
                    self.ready_to_save_video = True
                    self.saved_cat_video = True
                    self.saved_video = False
                    rospy.logwarn("Saving video.")
                elif self.cat and self.saved_cat_video and self.saved_video:
                    rospy.logwarn("Saved video.")
                if (not self.cat) and self.saved_video:
                    rospy.logwarn("End of bag files.")
                return
            elif self.saving_video:
                self.video_info.ready_for_bag_info = False
                self.video_info.ready_to_record = False
            elif (bag_name == "") and ready_to_play and (not finished_playing) and (not self.saving_images):
                self.video_info.ready_for_bag_info = True
                self.video_info.ready_to_record = False
            elif (bag_name != "") and ready_to_play and (not finished_playing) and (not self.saving_images):
                self.video_info.ready_for_bag_info = False
                self.video_info.ready_to_record = True
                self.saving_images = True
                self.saved_video = False
                self.video_info.saved_video = False
                rospy.logwarn("Saving images.")
                self.black_images_saved = False
                if not self.cat or (self.bag_name == ""):
                    self.bag_name = bag_name
                    self.image_number = 0
                    self.working_dir = self.working_dir_base + "/" + bag_name
                    chdir(self.working_dir)
            elif finished_playing and (not self.saved_video):
                self.video_info.ready_for_bag_info = False
                self.video_info.ready_to_record = False
                self.saving_images = False
                self.save_black_png()
                if not self.cat:
                    self.ready_to_save_video = True
                    rospy.logwarn("Saving video.")
                else:
                    self.ready_to_save_video = False
                    self.video_info.saved_video = True
                    self.saved_cat_video = False
            elif finished_playing and self.saved_video:
                self.video_info.saved_video = True

    def save_png(self,cv_image):
        if self.im_size is None:
            self.im_size = cv.GetSize(cv_image)
        image_name = "{num:06d}.png".format(num=self.image_number)
        cv.SaveImage(image_name,cv_image)
        self.image_number += 1

    def save_black_png(self):
        if 0 < self.black_image_count:
            if (self.im_black is None) and (self.im_size is not None):
                self.im_black = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)
                cv.SetZero(self.im_black)

            if (self.im_black is not None) and (not self.black_images_saved):
                for bi in range(self.black_image_count):
                    image_name = "{num:06d}.png".format(num=self.image_number)
                    cv.SaveImage(image_name,self.im_black)
                    self.image_number += 1
                self.black_images_saved = True

    def image_callback(self,data):
        if (self.working_dir is not None) and (self.saving_images):
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

            # rospy.loginfo("Saved image {num:06d}\n".format(num=self.image_number))
            # if not self.images_initialized:
            #     self.initialize_images(cv_image)

            # cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

    def find_png_list(self,bag_name):
        p_ls_png = subprocess.Popen('ls ' + bag_name + '/*.png',shell=True,stdout=subprocess.PIPE,stderr=self.NULL)
        out = p_ls_png.stdout.readlines()
        png_list = [s.rstrip() for s in out]
        return png_list

    def save_video(self):
        self.ready_to_save_video = False
        self.saving_video = True
        dir_base = self.working_dir_base
        chdir(dir_base)
        bag_name = self.bag_name
        png_list = self.find_png_list(bag_name)
        # rospy.logwarn("png_list = %s" % (str(png_list)))
        if 1 < self.repeat_count:
            dir_bag = bag_name + "_"
            dir_new = self.working_dir_base + "/" + dir_bag
            image_number = 0
            for png_file in png_list:
                chdir(dir_base)
                im = cv.LoadImage(png_file)
                chdir(dir_new)
                for repeat in range(self.repeat_count):
                    image_name = "{num:06d}.png".format(num=image_number)
                    cv.SaveImage(image_name,im)
                    image_number += 1
        else:
            dir_bag = bag_name

        chdir(self.working_dir_base)
        time.sleep(2)
        subprocess.check_call('ffmpeg -f image2 -i ' + \
                               dir_bag + '/%06d.png ' + \
                               '-r ' + str(self.frame_rate) + ' ' + \
                               '-sameq -s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -g 100 -bf 2 -pass 1/2 ' + \
                               # '-sameq -s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -bf 2 -pass 1/2 ' + \
                               bag_name + '.mpg',shell=True)

        # call_string = 'ffmpeg -f image2 -i ' + bag_name + '/%06d.png ' + '-r ' + str(self.frame_rate) + ' ' + '-sameq -s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -bf 2 -pass 1/2 ' + bag_name + '.mpg'
        # time.sleep(3)
        rospy.logwarn('Saved video %s' % (bag_name + '.mpg'))
        self.saving_video = False
        self.saved_video = True

    def main(self):
        while not rospy.is_shutdown():
            if self.ready_to_save_video and (not self.saving_video):
                self.save_video()
            # if self.saving_images_started and (self.last_image_time is not None):
            #     t = rospy.get_time()
            #     dt = t - self.last_image_time
            #     # rospy.logwarn("dt = %s" % (str(dt)))
            #     if self.time_limit < dt:
            #         self.save_video()
            self.video_info_pub.publish(self.video_info)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SaveVideo',log_level=rospy.INFO)
    sv = SaveVideo()
    sv.main()
