#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import numpy as N
import threading
import matplotlib.pyplot as plt

from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String
from patterngen.msg import MsgPattern



###############################################################################
###############################################################################
###############################################################################
# The class GalvoCalibrator commands the galvos to a set of input points,
# and reads the output points from the tracking system. 
# Computes a linear relationship between the two.
#
# 1. Delete any prior background image (e.g. /cameras/background.png)
# 2. Remove any filters so the camera can see the laser.
# 3. Turn off any illumination so that only the laser is seen.
# 4. Turn down the laser brightness so each spot looks like a fly.
# 5. Turn off the laser.
# 5. roslaunch galvodirector calibrator.launch
# 5. Turn on the laser.
# 6. Transfer the calculated numbers to GalvoDirector.py.
#
###############################################################################
###############################################################################
###############################################################################
class GalvoCalibrator:

    def __init__(self):
        self.initialized = False
        self.lock = threading.Lock()

        # Messages
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)
        self.pubGalvoCommand = rospy.Publisher('GalvoDirector/command', MsgGalvoCommand)
        self.pointsInput = [
                            Point(x=-2.0, y=0.0), 
                            Point(x=-1.0, y=0.0), 
                            Point(x=0.0, y=0.0), 
                            Point(x=1.0, y=0.0), 

                            Point(x=-4.0, y=1.0), 
                            Point(x=-3.0, y=1.0), 
                            Point(x=-2.0, y=1.0), 
                            Point(x=-1.0, y=1.0), 
                            Point(x=0.0, y=1.0), 
                            Point(x=1.0, y=1.0), 
                            Point(x=2.0, y=1.0), 
                            Point(x=3.0, y=1.0), 
                            
                            Point(x=-4.0, y=2.0),
                            Point(x=-3.5, y=2.0),
                            Point(x=0.0, y=2.0),
                            Point(x=2.5, y=2.0), 
                            Point(x=3.0, y=2.0), 
                            
                            Point(x=-5.0, y=3.0),
                            Point(x=-3.0, y=3.0),
                            Point(x=-2.0, y=3.0),
                            Point(x=-1.0, y=3.0),
                            Point(x=0.0, y=3.0),
                            Point(x=1.0, y=3.0),
                            Point(x=2.0, y=3.0), 
                            Point(x=2.5, y=3.0), 
                            Point(x=3.0, y=3.0), 
                            Point(x=3.5, y=3.0), 
                            
                            Point(x=-5.0, y=4.0),
                            Point(x=-4.5, y=4.0),
                            Point(x=-4.0, y=4.0),
                            Point(x=-3.5, y=4.0),
                            Point(x=-3.0, y=4.0),
                            Point(x=-2.5, y=4.0),
                            Point(x=-2.0, y=4.0),
                            Point(x=-1.5, y=4.0),
                            Point(x=-1.0, y=4.0),
                            Point(x=-0.5, y=4.0),
                            Point(x=0.0, y=4.0),
                            Point(x=0.5, y=4.0),
                            Point(x=1.0, y=4.0),
                            Point(x=1.5, y=4.0), 
                            Point(x=2.0, y=4.0), 
                            Point(x=2.5, y=4.0), 
                            Point(x=3.0, y=4.0), 
                            #Point(x=3.5, y=4.0), 
                            #Point(x=4.0, y=4.0), 
                            
                            #Point(x=-5.0, y=5.0),
                            Point(x=-4.0, y=5.0),
                            Point(x=-3.0, y=5.0),
                            Point(x=-2.0, y=5.0),
                            Point(x=-1.0, y=5.0),
                            Point(x=0.0, y=5.0),
                            Point(x=1.0, y=5.0), 
                            Point(x=2.0, y=5.0), 
                            Point(x=3.0, y=5.0), 
                            
                            Point(x=-3.0, y=6.0),
                            Point(x=-2.0, y=6.0),
                            Point(x=-1.5, y=6.0),
                            Point(x=0.0, y=6.0), 
                            Point(x=1.0, y=6.0), 
                            
                            Point(x=-1.0, y=7.0),
                            ]
        self.caldata = {}

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.initialized = True



    def OnShutdown_callback(self):
        pass
    

#    def ArenaState_callback(self, arenastate):
#        # Compute the linear relationship between volts and arenastate units.
#        # Should be correct, except for the possible ordering of the points.
#        # Need to determine the output orientation of the triangle.
            
#        if len(arenastate.flies)==len(self.pointsInput):
#            dxIn = self.pointsInput[1].x - self.pointsInput[0].x # i.e. +1
#            dyIn = self.pointsInput[1].y - self.pointsInput[2].y # i.e. -2
#            
#            xmax = 0
#            xmin = 99999
#            ymax = 0
#            ymin = 99999
#            for i in range(len(arenastate.flies)):
#                xmin = min(xmin,arenastate.flies[i].pose.position.x)
#                ymin = min(ymin,arenastate.flies[i].pose.position.y)
#                xmax = max(xmax,arenastate.flies[i].pose.position.x)
#                ymax = max(ymax,arenastate.flies[i].pose.position.y)
#    
#            dxOut = xmax-xmin
#            dyOut = ymax-ymin
#    
#            mx = dxIn / dxOut
#            my = dyIn / dyOut
#            
#            bx = -((mx * xmin) - self.pointsInput[0].x)
#            by = -((my * ymin) - self.pointsInput[2].y)
#            rospy.logwarn ('mx, bx, my, by:  %0.6f, %0.6f, %0.6f, %0.6f' % (mx,bx,my,by))

             
    # Append the input/output pairs to the calibration data.
    def ArenaState_callback(self, arenastate):
        with self.lock:
            if len(arenastate.flies)>0:
                if self.caldata.has_key(self.pointInput):
                    self.caldata[self.pointInput].append(arenastate.flies[0].pose.position)
                else:
                    self.caldata[self.pointInput] = [arenastate.flies[0].pose.position,]
        
        
    
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
        

    def SendPoint(self, point):
        pattern = MsgPattern()
        pattern.mode       = 'bypoints'
        pattern.shape      = 'constant'
        pattern.frame_id   = 'Plate'
        pattern.hzPattern  = 1.0
        pattern.hzPoint    = 100.0
        pattern.count      = 1
        pattern.points     = [point,]
        pattern.size.x     = 0.0
        pattern.size.y     = 0.0
        pattern.preempt    = False
        pattern.param      = 0.0
    
        command = MsgGalvoCommand()
        command.pattern_list = [pattern,]
        command.units = 'volts' #'millimeters' # 'volts' #
        self.pubGalvoCommand.publish(command)
        
        
    def GetInputOutputMedian(self):
        with self.lock:
            rv = {}
            
            for input,output_list in self.caldata.iteritems():
                x_list = []
                y_list = []
                for output in output_list:
                    x_list.append(output.x)
                    y_list.append(output.y)
                    
                x = N.median(x_list)
                y = N.median(y_list)
                rv[input] = Point(x=x, y=y, z=len(output_list))
            
        return rv
            

    def Main(self):
        rosRate = rospy.Rate(1.0)
    
        rospy.logwarn ('Find the median values, and enter them in GalvoDirector.py')
        rospy.logwarn ('mx, bx, my, by:')

        self.iPoint = 0
        while not rospy.is_shutdown():
            self.pointInput = self.pointsInput[self.iPoint]
            self.SendPoint(self.pointInput)
            self.iPoint = (self.iPoint+1) % len(self.pointsInput)
            
            output_byinput = self.GetInputOutputMedian()

            if len(output_byinput)>1:
                # Convert galvo axis(1,2) in/out pairs to x/y pairs.
                x1 = [] # galvo axis 1
                y1 = [] # galvo axis 1
                x2 = [] # galvo axis 2
                y2 = [] # galvo axis 2
                n = 9999999999
                for input,output in output_byinput.iteritems():
                    x1.append(input.x)
                    y1.append(output.x)
                    x2.append(input.y)
                    y2.append(output.y)
                    n = min(n,output.z)
                
                
                # Find least squares line for axis 1.
                x = N.array(y1)
                y = N.array(x1)
                A = N.vstack([x, N.ones(len(x))]).T
                m1, b1 = N.linalg.lstsq(A, y)[0]
    
                # Find least squares line for axis 2.
                x = N.array(y2)
                y = N.array(x2)
                A = N.vstack([x, N.ones(len(x))]).T
                m2, b2 = N.linalg.lstsq(A, y)[0]
                
                rospy.logwarn ('I/O pairs: %d' % len(output_byinput))
                rospy.logwarn ('Samples per input point: %d' % n)
                rospy.logwarn ('Millimeters -> Volts:')
                rospy.logwarn ('mx=%0.8f, bx=%0.8f, my=%0.8f, by=%0.8f' % (m1,b1,m2,b2))

                plt.scatter(x, y, 1)
                plt.plot(x, m*x + c, 'r', label='Fitted line')
                #plt.legend()
                plt.show()    
    
                rospy.logwarn('-------------------')
            rosRate.sleep()
            
    
        

if __name__ == '__main__':
    rospy.init_node('GalvoCalibrator')
    gc = GalvoCalibrator()
    gc.Main()


