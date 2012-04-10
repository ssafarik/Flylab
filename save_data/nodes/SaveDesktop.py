#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import cv
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

class SaveDesktop:
    def __init__(self):
        self.initialized = False
        self.working_dir = os.path.expanduser("~/RecordedDesktop")
        chdir(self.working_dir)

        # Create new directory each day
        self.working_dir = self.working_dir + "/" + time.strftime("%Y-%m-%d")
        chdir(self.working_dir)

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joystick_commands_callback)

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


    def main(self):
        while not rospy.is_shutdown():
            self.status_update()
            cv.ShowImage("Recording Status",self.im_status)
            cv.WaitKey(3)

if __name__ == '__main__':
    rospy.init_node('SaveDesktop',log_level=rospy.INFO)
    sd = SaveDesktop()
    sd.main()
