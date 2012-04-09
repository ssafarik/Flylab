#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import time,os,subprocess
from save_data.msg import BagInfo, VideoInfo
import re

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class PlayBags:
    def __init__(self):
        self.initialized = False
        self.working_dir_base = os.path.expanduser("~/FlylabData")
        chdir(self.working_dir_base)

        self.bag_info_pub = rospy.Publisher("bag_info",BagInfo)
        self.bag_info = BagInfo()
        self.bag_info.bag_name = ""
        self.bag_info.ready_to_play = True
        self.bag_info.finished_playing = False
        self.bag_info.end_of_bag_files = False

        self.video_info_sub = rospy.Subscriber("video_info",VideoInfo,self.video_info_callback)
        self.video_info = VideoInfo()
        self.video_info.ready_for_bag_info = False
        self.video_info.ready_to_record = False
        self.video_info.saved_video = False

        self.NULL = open('/dev/null', 'w')

        self.rate = rospy.Rate(10)

        self.bag_file_play_list_name = "bag_file_play_list"
        self.bag_list = self.find_bag_list()
        rospy.logwarn("bag_list = %s" % (str(self.bag_list)))
        self.bag_count = len(self.bag_list)
        self.bag_n = 0

        self.initialized = True

    def video_info_callback(self,data):
        if self.initialized:
            self.video_info = data

    def find_bag_list(self):
        # p_ls_bag = subprocess.Popen('ls *.bag',shell=True,stdout=subprocess.PIPE,stderr=self.NULL)
        # out = p_ls_bag.stdout.readlines()
        # bag_list = [s.rstrip() for s in out]
        fid = open(self.bag_file_play_list_name,'r')
        bag_list = [line.strip() for line in fid.readlines()]
        if 0 < len(bag_list):
            bag_file0 = bag_list[0]
            match = re.search('^\d+_\d+_\d+',bag_file0)
            if match is not None:
                bag_dir = match.group(0) + '_Bags'
                try:
                    os.chdir(bag_dir)
                except (OSError):
                    bag_list = []
            else:
                bag_list = []
        # rospy.logwarn("bag_list = %s" % (str(bag_list)))
        return bag_list

    def play_bag_file(self,bag_file):
        # rospy.loginfo("Playing %s" % (bag_file))
        subprocess.check_call('rosbag play ' + bag_file,shell=True)

    def main(self):
        while not rospy.is_shutdown():
            if self.bag_n < self.bag_count:
                # rospy.logwarn("bag_n = %s" % (str(self.bag_n)))
                self.bag_info.end_of_bag_files = False
                if self.video_info.ready_for_bag_info and (not self.video_info.ready_to_record):
                    rospy.logwarn("Ready to play and ready for bag info...")
                    self.bag_file = self.bag_list[self.bag_n]
                    bag_name,bag_ext = os.path.splitext(self.bag_file)
                    self.bag_info.bag_name = bag_name
                    rospy.logwarn("bag name = %s" % (bag_name))
                elif (not self.video_info.ready_for_bag_info) and self.video_info.ready_to_record and (not self.video_info.saved_video):
                    rospy.logwarn("Playing bag file...")
                    time.sleep(2)
                    self.play_bag_file(self.bag_file)
                    self.bag_info.finished_playing = True
                    self.bag_n += 1
                    self.bag_info.bag_name = ""
                    self.video_info.ready_to_record = False
                elif (not self.video_info.ready_for_bag_info) and (not self.video_info.ready_to_record) and self.video_info.saved_video:
                    self.bag_info.finished_playing = False
                    # if 0 < self.bag_count:
                    #     self.bag_info.end_of_bag_files = False
                    #     for bag_file in self.bag_set:
                    #         bag_name,bag_ext = os.path.splitext(bag_file)
                    #         self.bag_info.bag_name = bag_name
                    #         self.bag_info.ready_to_play = True
                    #         self.bag_info.finished_playing = False
                    #         rospy.logwarn("about to spin")
                    #         while (not rospy.is_shutdown()) and (not self.video_info.ready_to_record):
                    #             rospy.logwarn("spinning...")
                    #             self.bag_info_pub.publish(self.bag_info)
                    #             rospy.wait_for_message("video_info",VideoInfo)

                    #         self.video_info.ready_to_record = False
                    #         rospy.logwarn("Playing bag file...")
                    #         self.play_bag_file(bag_file)
                    #         self.bag_info.finished_playing = True
                    #         self.bag_info_pub.publish(self.bag_info)
                    # else:
                    #     rospy.logwarn("No bag files in %s" % (self.working_dir))

                    # self.bag_info.end_of_bag_files = True
            else:
                self.bag_info.end_of_bag_files = True

            self.bag_info_pub.publish(self.bag_info)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('PlayBags',log_level=rospy.INFO)
    pb = PlayBags()
    pb.main()
    # rospy.Service('bag_info', BagInfo, pb.bag_info)
    # rospy.Service('play_bag', BagInfo, pb.play_bag)
    # rospy.spin()
