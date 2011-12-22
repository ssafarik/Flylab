#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg

class Colors:
    def __init__(self):
        self.color_max = 255

        self.red = image_gui.msg.CvColor()
        self.red.red = self.color_max
        self.red.green = 0
        self.red.blue = 0
        self.green = image_gui.msg.CvColor()
        self.green.red = 0
        self.green.green = self.color_max
        self.green.blue = 0
        self.blue = image_gui.msg.CvColor()
        self.blue.red = 0
        self.blue.green = 0
        self.blue.blue = self.color_max
        self.magenta = image_gui.msg.CvColor()
        self.magenta.red = self.color_max
        self.magenta.green = 0
        self.magenta.blue = self.color_max
        self.yellow = image_gui.msg.CvColor()
        self.yellow.red = self.color_max
        self.yellow.green = self.color_max
        self.yellow.blue = 0
        self.cyan = image_gui.msg.CvColor()
        self.cyan.red = 0
        self.cyan.green = self.color_max
        self.cyan.blue = self.color_max
