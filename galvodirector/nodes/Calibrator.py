#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import copy
import numpy as N
import threading
import matplotlib.pyplot as plt
import pylab

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
                            Point(x=1.5, y=0.0), 

                            Point(x=-3.0, y=0.5), 
                            Point(x=-2.0, y=0.5), 
                            Point(x=-1.0, y=0.5), 
                            Point(x=0.0, y=0.5), 
                            Point(x=1.0, y=0.5), 
                            Point(x=2.0, y=0.5), 
                            Point(x=2.5, y=0.5), 

                            Point(x=-3.5, y=1.0), 
                            Point(x=-3.0, y=1.0), 
                            Point(x=-2.0, y=1.0), 
                            Point(x=-1.0, y=1.0), 
                            Point(x=0.0, y=1.0), 
                            Point(x=1.0, y=1.0), 
                            Point(x=2.0, y=1.0), 
                            Point(x=3.0, y=1.0), 
                            
                            Point(x=-4.0, y=1.5),
                            Point(x=-3.5, y=1.5),
                            #Point(x=0.0, y=1.5),
                            Point(x=2.5, y=1.5), 
                            Point(x=3.0, y=1.5), 
                            
                            Point(x=-4.0, y=2.0),
                            Point(x=-3.5, y=2.0),
                            #Point(x=0.0, y=2.0),
                            Point(x=2.5, y=2.0), 
                            Point(x=3.0, y=2.0), 
                            
                            Point(x=-4.0, y=3.0),
                            Point(x=-3.0, y=3.0),
                            Point(x=-2.0, y=3.0),
                            Point(x=-1.0, y=3.0),
                            Point(x=0.0, y=3.0),
                            Point(x=1.0, y=3.0),
                            Point(x=2.0, y=3.0), 
                            Point(x=2.5, y=3.0), 
                            Point(x=3.0, y=3.0), 
                            Point(x=3.5, y=3.0), 
                            Point(x=4.0, y=3.0), 
                            
                            Point(x=-4.0, y=3.5),
                            Point(x=-3.0, y=3.5),
                            Point(x=-2.0, y=3.5),
                            Point(x=-1.0, y=3.5),
                            Point(x=0.0, y=3.5),
                            Point(x=1.0, y=3.5),
                            Point(x=2.0, y=3.5), 
                            Point(x=2.5, y=3.5), 
                            Point(x=3.0, y=3.5), 
                            Point(x=3.5, y=3.5), 
                            Point(x=4.0, y=3.5), 
                            
                            #Point(x=-5.0, y=4.0),
                            #Point(x=-4.5, y=4.0),
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
                            Point(x=3.5, y=4.0), 
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
                            Point(x=3.5, y=5.0), 
                            
                            Point(x=-3.5, y=5.5),
                            Point(x=-3.0, y=5.5),
                            Point(x=-2.0, y=5.5),
                            #Point(x=-1.5, y=5.5),
                            #Point(x=0.0, y=5.5), 
                            #Point(x=1.0, y=5.5), 
                            #Point(x=1.5, y=5.5), 
                            Point(x=2.0, y=5.5), 
                            Point(x=2.5, y=5.5), 
                            Point(x=3.0, y=5.5), 
                            
                            Point(x=-3.0, y=6.0),
                            Point(x=-2.0, y=6.0),
                            Point(x=-1.5, y=6.0),
                            Point(x=0.0, y=6.0), 
                            Point(x=1.0, y=6.0), 
                            Point(x=1.5, y=6.0), 
                            Point(x=2.0, y=6.0), 
                            Point(x=2.5, y=6.0), 
                            
                            Point(x=-2.0, y=6.5),
                            Point(x=-1.5, y=6.5),
                            Point(x=0.0, y=6.5), 
                            Point(x=1.0, y=6.5), 
                            Point(x=1.5, y=6.5), 
                            Point(x=2.0, y=6.5), 
                            
                            Point(x=-1.0, y=7.0),
                            Point(x=-0.5, y=7.0),
                            Point(x=0.0, y=7.0),
                            Point(x=0.5, y=7.0),
                            Point(x=1.0, y=7.0),
                            Point(x=1.5, y=7.0),
                            ]
        self.caldata = {}
        pylab.ion()
        self.pointInput = None

        rospy.on_shutdown(self.OnShutdown_callback)
        


    def OnShutdown_callback(self):
        pass
    

    # Append the input/output pairs to the calibration data.
    def ArenaState_callback(self, arenastate):
        if self.initialized:
            with self.lock:
                if (self.pointInput is not None) and len(arenastate.flies)>0:
                    if self.pointInput not in self.caldata:
                        self.caldata[self.pointInput] = []

                    self.caldata[self.pointInput].append(Point(x=arenastate.flies[0].pose.position.x,
                                                               y=arenastate.flies[0].pose.position.y))
        
        
    
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
        command.enable_laser = True
        command.pattern_list = [pattern,]
        command.units = 'volts' #'millimeters' # 'volts' #
        self.pubGalvoCommand.publish(command)
        
    
    # For each input point, find the median value of the output points.
    # Return a dict[input]=output
    #
    def GetInputOutputMedian(self):
        with self.lock:
            rv = {}
            
            for input,output_list in self.caldata.iteritems():
                x_list = []
                y_list = []
                for output in output_list:
                    x_list.append(output.x)
                    y_list.append(output.y)
                    
                xMedian = N.median(x_list)
                yMedian = N.median(y_list)
                rv[input] = [Point(x=xMedian, y=yMedian, z=len(output_list)),]
            
        return rv
            

    def Main(self):
        rosRate = rospy.Rate(0.1)
    
        rospy.sleep(5)
        self.initialized = True

        rospy.logwarn ('Find the median values, and enter them in params_galvos.launch')
        rospy.logwarn ('mx, bx, my, by:')
        plt.figure(1)
        rospy.sleep(20)
        
        while not rospy.is_shutdown():
            # Send all the input points.  The arenastate callback collects the output points into self.caldata.
            for pointInput in self.pointsInput:
                with self.lock:
                    self.pointInput = pointInput
                    self.SendPoint(pointInput)
                rosRate.sleep()
                
            outputlist_byinput = self.GetInputOutputMedian() #self.caldata#
            #rospy.logwarn(outputlist_byinput)

            # Compute the linear relationship between volts and arenastate units.
            if len(outputlist_byinput)>1:
                # Convert galvo axis(1,2) in/out pairs to x/y pairs.
                x1 = [] # galvo axis 1
                y1 = [] # galvo axis 1
                x2 = [] # galvo axis 2
                y2 = [] # galvo axis 2
                n = 9999999999
                for input,output_list in outputlist_byinput.iteritems():
                    for output in output_list:
                        x1.append(input.x)
                        y1.append(output.x)
                        
                    for output in output_list:
                        x2.append(input.y)
                        y2.append(output.y)
                    
                    n = min(n,len(self.caldata[input]))#output_list))
                
                
                # Find least squares line for axis 1.
                x = N.array(y1) # Millimeters
                y = N.array(x1) # to Volts
                A = N.vstack([x, N.ones(len(x))]).T
                m1, b1 = N.linalg.lstsq(A, y)[0]
    
                plt.subplot(1,2,1)
                plt.cla()
                plt.scatter(x, y, 4)
                plt.plot(x, m1*x + b1)
                plt.title('axis 1')
                plt.draw()    
    
                # Find least squares line for axis 2.
                x = N.array(y2) # Millimeters
                y = N.array(x2) # to Volts
                A = N.vstack([x, N.ones(len(x))]).T
                m2, b2 = N.linalg.lstsq(A, y)[0]
                
                plt.subplot(1,2,2)
                plt.cla()
                plt.scatter(x, y, 4)
                plt.plot(x, m2*x + b2)
                plt.title('axis 2')
                plt.draw()    
    
                rospy.logwarn ('I/O point pairs: %d' % len(outputlist_byinput))
                rospy.logwarn ('Samples per input point: %d' % n)
                rospy.logwarn ('Least Squares.  Millimeters -> Volts:')
                rospy.logwarn ('    <param name="galvodirector/mx" type="double" value="%0.7f" />' % m1)
                rospy.logwarn ('    <param name="galvodirector/bx" type="double" value="%0.7f" />' % b1)
                rospy.logwarn ('    <param name="galvodirector/my" type="double" value="%0.7f" />' % m2)
                rospy.logwarn ('    <param name="galvodirector/by" type="double" value="%0.7f" />' % b2)
                #rospy.logwarn ('mx=%0.8f, bx=%0.8f, my=%0.8f, by=%0.8f' % (m1,b1,m2,b2))

                rospy.logwarn('-------------------')
        
    
        

if __name__ == '__main__':
    rospy.init_node('GalvoCalibrator')
    gc = GalvoCalibrator()
    gc.Main()


