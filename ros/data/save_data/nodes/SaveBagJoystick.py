#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import cv
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
import time,os,subprocess,signal
from joystick_commands.msg import JoystickCommands


class RecordingStatus:
    def __init__(self):
        self.color_max = 255
        self.set_status(0)

    def get_status(self):
        return self.status_number, self.status_string, self.status_color

    def set_status(self,status_number):
        if status_number == 0:
            self.status_number = status_number
            self.status_string = "Ready"
            self.status_color = cv.CV_RGB(0,self.color_max,0)
        elif status_number == 1:
            self.status_number = status_number
            self.status_string = "Recording"
            self.status_color = cv.CV_RGB(self.color_max,0,0)
        elif status_number == 2:
            self.status_number = status_number
            self.status_string = "Waiting"
            self.status_color = cv.CV_RGB(self.color_max,self.color_max,0)
        else:
            self.status_number = status_number
            self.status_string = "Unknown"
            self.status_color = cv.CV_RGB(0,0,0)

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveBags:
    def __init__(self):
        self.initialized = False
        self.working_dir = os.path.expanduser("~/Bags")
        chdir(self.working_dir)
        # time.strftime("%Y-%m-%d-%H-%M-%S")

        # Create new directory each day
        self.working_dir = self.working_dir + "/" + time.strftime("%Y-%m-%d")
        chdir(self.working_dir)

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joystick_commands_callback)
        self.topic_record_list = ["/camera/image_display"]

        self.NULL = open('/dev/null', 'w')

        cv.NamedWindow("Recording Status",1)
        self.im_size = (100,100)
        self.im_status = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)

        self.circle_size = min(self.im_size[0],self.im_size[1])/4
        self.rs = RecordingStatus()

        self.status_number_previous = 0
        self.save = None

        # Find all processes named 'record'
        self.find_record_pids()
        # Kill all processes name 'record'
        self.kill_record_pids()

        # Find set of bag files
        self.bag_set = self.find_bag_set()

        self.initialized = True

    def find_bag_set(self):
        p_ls_bag = subprocess.Popen('ls *.bag',shell=True,stdout=subprocess.PIPE,stderr=self.NULL)
        out = p_ls_bag.stdout.readlines()
        bag_set = set([s.rstrip() for s in out])
        return bag_set

    def rm_bag_set_diff(self):
        bag_set_new = self.find_bag_set()
        diff = bag_set_new - self.bag_set
        rm_str = "rm "
        for bag_file in diff:
            p_rm = subprocess.Popen(rm_str + bag_file,shell=True)
            rospy.logwarn("removing = %s" % (str(bag_file)))

    def add_bag_set_diff(self):
        bag_set_new = self.find_bag_set()
        diff = bag_set_new - self.bag_set
        for bag_file in diff:
            rospy.logwarn("adding = %s" % (str(bag_file)))
        self.bag_set = self.find_bag_set()

    def find_record_pids(self):
        p_pid = subprocess.Popen('pidof record',shell=True,stdout=subprocess.PIPE)
        out = p_pid.stdout.readlines()
        # rospy.logwarn("out = %s" % (str(out)))
        if 0 < len(out):
            p_list_str = out[0].rsplit()
            self.pid = [int(s) for s in p_list_str]
            # rospy.logwarn("pid_list = %s" % (str(self.pid)))
        else:
            self.pid = None

    def kill_record_pids(self):
        if self.pid is not None:
            for p in range(len(self.pid)):
                os.kill(self.pid[p],signal.SIGINT)

    def status_update(self):
        self.status_number, self.status_string, self.status_color = self.rs.get_status()
        # rospy.logwarn("status_number = %s" % (str(self.status_number)))
        # rospy.logwarn("status_string = %s" % (str(self.status_string)))
        # rospy.logwarn("status_color = %s" % (str(self.status_color)))
        cv.Circle(self.im_status,
                  (int(self.im_size[0]/2), int(self.im_size[1]/2)),
                  int(self.circle_size), self.status_color, cv.CV_FILLED)

        if self.status_number == self.status_number_previous:
            if self.status_number == 2:
                if self.save is not None:
                    if self.save:
                        self.add_bag_set_diff()
                    else:
                        self.rm_bag_set_diff()
                    self.rs.set_status(0)
        elif (self.status_number_previous == 0) and \
             (self.status_number == 1):
            call_string = 'rosbag record -b 0 ' + '-o ' + self.working_dir + '/image_display'
            for s in self.topic_record_list:
                call_string = call_string + " " + s
            # rospy.logwarn("call_string = \n%s" % (str(call_string)))
            self.process = subprocess.Popen(call_string,shell=True)
            # time.sleep(0.5)
            # self.find_record_pids()
        elif (self.status_number_previous == 1) and \
             (self.status_number == 2):
            # rospy.logwarn("sending ctrl-c...")
            # self.process.signal(CTRL_C_EVENT)
            # rospy.logwarn("sending terminate...")
            # self.process.terminate()
            # rospy.logwarn("sending kill...")
            # self.process.kill()
            # rospy.logwarn("os.kill...")

            # Find all processes named 'record'
            self.find_record_pids()
            # Kill all processes name 'record'
            self.kill_record_pids()

        self.status_number_previous = self.status_number

    def joystick_commands_callback(self,data):
        if data.start and (not data.stop):
            self.rs.set_status(1)
        elif data.stop and (not data.start):
            self.rs.set_status(2)

        if data.yes and (not data.no):
            self.save = True
        elif data.no and (not data.yes):
            self.save = False
        else:
            self.save = None

        # self.image_frame = rospy.get_param("save_image_frame")
        # self.image_sub = rospy.Subscriber(self.image_frame, Image, self.image_callback)

        # self.bridge = CvBridge()
        # self.image_number = 0
        # self.frame_rate = rospy.get_param("framerate","30")
        # self.video_format = rospy.get_param("save_video_format","flv")

        # self.saving_images_started = False
        # self.last_image_time = None
        # self.rate = rospy.Rate(10)     # Hz
        # self.time_limit = 3

        # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        # self.video_initialized = False

    # def save_png(self,cv_image):
    #     image_name = "{num:06d}.png".format(num=self.image_number)
    #     cv.SaveImage(image_name,cv_image)

    # def write_frame(self,cv_image):
    #     cv.WriteFrame(self.video_writer,cv_image)

    # def initialize_video(self,cv_image):
    #     self.video_writer = cv.CreateVideoWriter(self.working_dir+".mjpg",
    #                                              cv.CV_FOURCC('M','J','P','G'),
    #                                              self.frame_rate,
    #                                              cv.GetSize(cv_image))

        # self.im_display = cv.CreateImage(cv.GetSize(cv_image),cv.IPL_DEPTH_8U,3)
        # self.video_initialized = True

    # def image_callback(self,data):
    #     if not self.saving_images_started:
    #         self.saving_images_started = True
    #     self.last_image_time = rospy.get_time()
    #     # Convert ROS image to OpenCV image
    #     try:
    #       cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
    #     except CvBridgeError, e:
    #       print e

    #     self.save_png(cv_image)

    #     # if self.video_format in "png":
    #     #     self.save_png(cv_image)
    #     # else:
    #     #     if not self.video_initialized:
    #     #         self.initialize_video(cv_image)
    #     #     self.write_frame(cv_image)

    #     rospy.loginfo("Saved image {num:06d}\n".format(num=self.image_number))
    #     self.image_number += 1
    #     # if not self.images_initialized:
    #     #     self.initialize_images(cv_image)

    #     # cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

    # def save_video(self):
    #     self.saving_images_started = False
    #     os.chdir(os.path.expanduser("~/Videos"))
    #     if self.video_format in "flv":
    #         subprocess.check_call(['ffmpeg','-f','image2',
    #                                '-i',self.working_dir+'/%06d.png',
    #                                '-sameq',
    #                                '-ar','44100',
    #                                '-ab','64k',
    #                                '-ac','2',
    #                                '-r',str(self.frame_rate),
    #                                '-s','640x480',
    #                                self.working_dir+'.flv'])
    #         print 'Saved video ' + self.working_dir + '.flv'
    #     elif self.video_format in "gif":
    #         subprocess.check_call(['ffmpeg','-f','image2',
    #                                '-i',self.working_dir+'/%06d.png',
    #                                '-sameq',
    #                                '-r',str(self.frame_rate),
    #                                '-s','640x480',
    #                                '-pix_fmt','rgb24',
    #                                self.working_dir+'.gif'])
    #         print 'Saved video ' + self.working_dir + '.gif'
    #     elif self.video_format in "avi":
    #         subprocess.check_call(['ffmpeg','-f','image2',
    #                                '-i',self.working_dir+'/%06d.png',
    #                                '-sameq',
    #                                '-r',str(self.frame_rate),
    #                                '-s','640x480',
    #                                self.working_dir+'.avi'])
    #         print 'Saved video ' + self.working_dir + '.avi'
    #     elif self.video_format in "mpeg1":
    #         subprocess.check_call(['ffmpeg','-f','image2',
    #                                '-i',self.working_dir+'/%06d.png',
    #                                '-sameq',
    #                                '-r',str(self.frame_rate),
    #                                '-s','640x480',
    #                                '-mbd','rd',
    #                                '-trellis','2',
    #                                '-cmp','2',
    #                                '-subcmp','2',
    #                                '-pass','1/2',
    #                                self.working_dir+'.mpg'])
    #         print 'Saved video ' + self.working_dir + '.mpg'

    def main(self):
        while not rospy.is_shutdown():
            self.status_update()
            cv.ShowImage("Recording Status",self.im_status)
            cv.WaitKey(3)

            # if self.saving_images_started and (self.last_image_time is not None):
            #     t = rospy.get_time()
            #     dt = t - self.last_image_time
            #     # rospy.logwarn("dt = %s" % (str(dt)))
            #     if self.time_limit < dt:
            #         self.save_video()
            # self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SaveBags',log_level=rospy.INFO)
    sb = SaveBags()
    sb.main()
