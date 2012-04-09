#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('save_data')
import sys
import rospy
import cv
import time,os,subprocess,signal
from save_data.msg import CommandSavedata


def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveBag:
    def __init__(self):
        self.initialized = False
        self.working_dir_base = os.path.expanduser("~/FlylabData")
        chdir(self.working_dir_base)

        # Create new directory each day
        self.files_dir = time.strftime("%Y_%m_%d")
        self.working_dir = self.working_dir_base + "/" + self.files_dir + "_Bags"
        chdir(self.working_dir)

        self.sub_commandsavedata = rospy.Subscriber("CommandSavedata", CommandSavedata, self.commandsavedata_callback)

        self.topic_record_list = ["/camera/image_display"]

        self.NULL = open('/dev/null', 'w')

        self.save_bag = False

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

    def commandsavedata_callback(self,data):
        if not data.rm_file:
            if data.save_bag and (not self.save_bag):
                self.file_name = data.file_name_base + '.bag'
                call_string = 'rosbag record -b 0 ' + '-O ' + self.working_dir + '/' + self.file_name
                for s in self.topic_record_list:
                    call_string = call_string + " " + s
                # rospy.logwarn("call_string = \n%s" % (str(call_string)))
                self.process = subprocess.Popen(call_string,shell=True)
                self.save_bag = data.save_bag
            elif (not data.save_bag) and self.save_bag:
                self.save_bag = data.save_bag
                # Find all processes named 'record'
                self.find_record_pids()
                # Kill all processes name 'record'
                self.kill_record_pids()
                self.add_bag_set_diff()
        else:
            self.save_bag = False
            # Find all processes named 'record'
            self.find_record_pids()
            # Kill all processes name 'record'
            self.kill_record_pids()
            self.rm_bag_set_diff()


if __name__ == '__main__':
    rospy.init_node('SaveBag',log_level=rospy.INFO)
    sb = SaveBag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
