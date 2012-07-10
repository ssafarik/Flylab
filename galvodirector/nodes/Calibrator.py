#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
#import tf
#import numpy as N
from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from geometry_msgs.msg import Point
#from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header, String
from patterngen.msg import MsgPattern
#from patterngen.srv import *



###############################################################################
###############################################################################
###############################################################################
# The class GalvoCalibrator commands the galvos to a set of input points,
# and reads the output points from the tracking system. 
# Computes a linear relationship between the two.
#
# 1. Delete any prior background image (e.g. /cameras/background.png)
# 2. Turn off any illumination so that only the laser is seen.
# 3. Turn down the laser brightness so each spot looks like a fly.
# 4. Transfer the calculated numbers.
#
###############################################################################
###############################################################################
###############################################################################
class GalvoCalibrator:

    def __init__(self):
        self.initialized = False

        # Messages
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)
        self.pubGalvoCommand = rospy.Publisher('GalvoDirector/command', MsgGalvoCommand)
        self.pointsInput = [Point(x=-4.0, y=1.0), 
                            Point(x=3.0, y=1.0), 
                            Point(x=-1.0, y=7.0)]

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.initialized = True



    def OnShutdown_callback(self):
        pass
    

    def ArenaState_callback(self, arenastate):
        # Compute the linear relationship between volts and arenastate units.
        # Should be correct, except for the possible ordering of the points.
        # Need to determine the output orientation of the triangle.
        if len(arenastate.flies)==len(self.pointsInput):
            dxIn = self.pointsInput[1].x - self.pointsInput[0].x # i.e. +1
            dyIn = self.pointsInput[1].y - self.pointsInput[2].y # i.e. -2
            
            xmax = 0
            xmin = 99999
            ymax = 0
            ymin = 99999
            for i in range(len(arenastate.flies)):
                xmin = min(xmin,arenastate.flies[i].pose.position.x)
                ymin = min(ymin,arenastate.flies[i].pose.position.y)
                xmax = max(xmax,arenastate.flies[i].pose.position.x)
                ymax = max(ymax,arenastate.flies[i].pose.position.y)
    
            dxOut = xmax-xmin
            dyOut = ymax-ymin
    
            mx = dxIn / dxOut
            my = dyIn / dyOut
            
            bx = -((mx * xmin) - self.pointsInput[0].x)
            by = -((my * ymin) - self.pointsInput[2].y)
            rospy.logwarn ('mx, bx, my, by:  %0.6f, %0.6f, %0.6f, %0.6f' % (mx,bx,my,by))
             
    
    def SendInputPoints(self):
        pattern = MsgPattern()
        pattern.mode       = 'bypoints'
        pattern.shape      = 'constant'
        pattern.frame_id   = 'Plate'
        pattern.hzPattern  = 1.0
        pattern.hzPoint    = 100.0
        pattern.count      = 1
        pattern.points     = self.pointsInput
        pattern.size.x     = 20.0
        pattern.size.y     = 20.0
        pattern.preempt    = False
        pattern.param      = 0.0
    
        command = MsgGalvoCommand()
        command.pattern_list = [pattern,]
        command.units = 'volts' #'millimeters' # 'volts' #
        self.pubGalvoCommand.publish(command)
        

    def Main(self):
        rospy.logwarn ('Find the median values, and enter them in GalvoDirector.py')
        rospy.logwarn ('mx, bx, my, by:')

        while not rospy.is_shutdown():
            self.SendInputPoints()
            rospy.sleep(1.0)
            
    
        

if __name__ == '__main__':
    rospy.init_node('GalvoCalibrator')
    gc = GalvoCalibrator()
    gc.Main()


