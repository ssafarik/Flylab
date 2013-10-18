#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import copy
import numpy as N
from scipy.optimize import leastsq
import threading
import matplotlib.pyplot as plt
import pylab
import pickle

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
# See calibrate_galvos.txt for how to calibrate the galvos.
#
###############################################################################
###############################################################################
###############################################################################
class GalvoCalibrator:

    def __init__(self):
        self.initialized = False
        self.lock = threading.Lock()
        self.command = 'continue'
        self.bCollectOutput = False
        
        # Messages
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)
        self.subCommand    = rospy.Subscriber('broadcast/command', String, self.Command_callback)

        self.pubGalvodirectorCommand = rospy.Publisher('galvodirector/command', MsgGalvoCommand)


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
                            
                            Point(x=-3.9, y=1.5),
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
                            #Point(x=4.0, y=3.0), 
                            
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
                            #Point(x=4.0, y=3.5), 
                            
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
                            #Point(x=3.5, y=5.0), 
                            
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
    

    def Command_callback(self, msgString):
        self.command = msgString.data
          
        
    # Append the input/output pairs to the calibration data.
    def ArenaState_callback(self, arenastate):
        self.initialized = True
        
        if (self.bCollectOutput):
            with self.lock:
                if (self.pointInput is not None) and len(arenastate.flies)>0:
                    if self.pointInput not in self.caldata:
                        self.caldata[self.pointInput] = []
    
                    self.caldata[self.pointInput].append(Point(x=arenastate.flies[0].pose.position.x,
                                                               y=arenastate.flies[0].pose.position.y))
        
        
    
    def SendInputPoints(self):
        pattern = MsgPattern()
        pattern.frameidPosition = '/Arena'
        pattern.frameidAngle    = '/Arena'
        pattern.shape      = 'bypoints'
        pattern.hzPattern  = 1.0
        pattern.hzPoint    = 100.0
        pattern.count      = 1
        pattern.points     = self.pointsInput
        pattern.size.x     = 20.0
        pattern.size.y     = 20.0
        pattern.preempt    = False
        pattern.param      = 0.0
        pattern.direction  = 1
    
        command = MsgGalvoCommand()
        command.pattern_list = [pattern,]
        command.units = 'volts' #'millimeters' # 'volts' #
        self.pubGalvodirectorCommand.publish(command)
        

    def SendPoint(self, point):
        pattern = MsgPattern()
        pattern.frameidPosition = '/Arena'
        pattern.frameidAngle    = '/Arena'
        pattern.shape      = 'bypoints'
        pattern.hzPattern  = 1.0
        pattern.hzPoint    = 100.0
        pattern.count      = 1
        pattern.points     = [point,]
        pattern.size.x     = 0.0
        pattern.size.y     = 0.0
        pattern.preempt    = False
        pattern.param      = 0.0
        pattern.direction  = 1
    
        command = MsgGalvoCommand()
        command.enable_laser = True
        command.pattern_list = [pattern,]
        command.units = 'volts' #'millimeters' # 'volts' #

        self.pubGalvodirectorCommand.publish(command)
        
    
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
            

    def residuals(self, p, y, x):
        err = self.yfit(x,p) - y
        return err
    
    def yfit(self, x, p):
        rv = p[3]*x**3 + p[2]*x**2 + p[1]*x + p[0]
        return rv


    def Main(self):
        rosRate = rospy.Rate(0.5)#(0.1)

        while (not self.initialized):    
            rospy.sleep(0.5)

        rospy.logwarn ('Find the regression parameter values, and enter them in params_galvos.launch')
        plt.figure(1)
        
        rospy.logwarn ('Running.')
        
        # Initial conditions.
        p = (N.array([0,0,0,0]),0)
        q = (N.array([0,0,0,0]),0)
        pi = (N.array([0,0,0,0]),0)
        qi = (N.array([0,0,0,0]),0)
        while (not rospy.is_shutdown()) and ('exit' not in self.command):
            # Send all the input points.  The arenastate callback collects the output points into self.caldata.
            bUseRandomPoints = True
            if (bUseRandomPoints):
                # Specify the location of a circle (in volts) where we'll choose the input points.
                r = 3.5
                cx = -0.25
                cy = 3.5
                
                for i in range(10):
                    # Choose a random point in the circle.
                    x = 99
                    y = 99
                    while (N.linalg.norm([x-cx,y-cy])>r):
                        x = cx - r + 2*r*N.random.random()
                        y = cy - r + 2*r*N.random.random()
                        
                    self.pointInput = Point(x=x, y=y)
                    
                    # Send the point to the galvos, and wait for the output data (via the callback). 
                    self.bCollectOutput = False # Wait for the galvos.
                    self.SendPoint(self.pointInput)
                    rospy.sleep(0.3) # Give the galvo command some time to make it to the galvos.
                    self.bCollectOutput = True
                    rosRate.sleep()
                    
                    if (self.command=='exit_now'):
                        break
                    
                self.bCollectOutput = False
                
                
            else: # Use pre-specified points.
                for pointInput in self.pointsInput:
                    with self.lock:
                        self.pointInput = pointInput
                        self.SendPoint(pointInput)
                    rosRate.sleep()
                    if (self.command=='exit_now'):
                        break

                
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



                # Find forward least squares line for axis 1 (millimeters to volts).
                xa = N.array(y1) # Millimeters
                ya = N.array(x1) # to Volts
                p = leastsq(self.residuals, p[0], args=(ya, xa))
                a = p[0]
    
                # Find forward least squares line for axis 2 (millimeters to volts).
                xb = N.array(y2) # Millimeters
                yb = N.array(x2) # to Volts
                q = leastsq(self.residuals, q[0], args=(yb, xb))
                b = q[0]

                # Find inverse least squares line for axis 1 (volts to millimeters).
                xai = N.array(x1) # Millimeters
                yai = N.array(y1) # to Volts
                pi = leastsq(self.residuals, pi[0], args=(yai, xai))
                ai = pi[0]
    
                # Find inverse least squares line for axis 2 (volts to millimeters).
                xbi = N.array(x2) # Millimeters
                ybi = N.array(y2) # to Volts
                qi = leastsq(self.residuals, qi[0], args=(ybi, xbi))
                bi = qi[0]


                # Sort the points.
                xy1=N.array([xa,ya]).T.tolist()
                xy1.sort()
                xy1s = N.array(xy1).T
                xa = xy1s[0]
                ya = xy1s[1]

                xy2=N.array([xb,yb]).T.tolist()
                xy2.sort()
                xy2s = N.array(xy2).T
                xb = xy2s[0]
                yb = xy2s[1]
                
                xy1i=N.array([xai,yai]).T.tolist()
                xy1i.sort()
                xy1si = N.array(xy1i).T
                xai = xy1si[0]
                yai = xy1si[1]

                xy2i=N.array([xbi,ybi]).T.tolist()
                xy2i.sort()
                xy2si = N.array(xy2i).T
                xbi = xy2si[0]
                ybi = xy2si[1]
                
                
                # Plot the two axes in each of:  mm to volts, and volts to mm.
                plt.subplot(2,2,1)
                plt.cla()
                plt.plot(xa,ya,'b.', xa,self.yfit(xa,a),'r-')
                plt.title('axis 1, mm to volts')
                plt.draw()    

                plt.subplot(2,2,2)
                plt.cla()
                plt.plot(xb,yb,'b.', xb,self.yfit(xb,b),'r-')
                plt.title('axis 2, mm to volts')
                plt.draw()    
    
                plt.subplot(2,2,3)
                plt.cla()
                plt.plot(xai,yai,'b.', xai,self.yfit(xai,ai),'r-')
                plt.title('axis 1, volts to mm')
                plt.draw()    

                plt.subplot(2,2,4)
                plt.cla()
                plt.plot(xbi,ybi,'b.', xbi,self.yfit(xbi,bi),'r-')
                plt.title('axis 2, volts to mm')
                plt.draw()    
    
    
                rospy.logwarn ('I/O point pairs: %d' % len(outputlist_byinput))
                rospy.logwarn ('Samples per input point: %d' % n)
                rospy.logwarn ('<!-- Least Squares for y = a3*x**3 + a2*x**2 + a1*x + a0.  Millimeters to Volts: -->')
                rospy.logwarn ('    <param name="galvodirector/a3" type="double" value="%g" />' % a[3])
                rospy.logwarn ('    <param name="galvodirector/a2" type="double" value="%g" />' % a[2])
                rospy.logwarn ('    <param name="galvodirector/a1" type="double" value="%g" />' % a[1])
                rospy.logwarn ('    <param name="galvodirector/a0" type="double" value="%g" />' % a[0])
                rospy.logwarn ('    <param name="galvodirector/b3" type="double" value="%g" />' % b[3])
                rospy.logwarn ('    <param name="galvodirector/b2" type="double" value="%g" />' % b[2])
                rospy.logwarn ('    <param name="galvodirector/b1" type="double" value="%g" />' % b[1])
                rospy.logwarn ('    <param name="galvodirector/b0" type="double" value="%g" />' % b[0])
                rospy.logwarn ('<!-- Least Squares for y = a3i*x**3 + a2i*x**2 + a1i*x + a0i.  Volts to Millimeters: -->')
                rospy.logwarn ('    <param name="galvodirector/a3i" type="double" value="%g" />' % ai[3])
                rospy.logwarn ('    <param name="galvodirector/a2i" type="double" value="%g" />' % ai[2])
                rospy.logwarn ('    <param name="galvodirector/a1i" type="double" value="%g" />' % ai[1])
                rospy.logwarn ('    <param name="galvodirector/a0i" type="double" value="%g" />' % ai[0])
                rospy.logwarn ('    <param name="galvodirector/b3i" type="double" value="%g" />' % bi[3])
                rospy.logwarn ('    <param name="galvodirector/b2i" type="double" value="%g" />' % bi[2])
                rospy.logwarn ('    <param name="galvodirector/b1i" type="double" value="%g" />' % bi[1])
                rospy.logwarn ('    <param name="galvodirector/b0i" type="double" value="%g" />' % bi[0])
    
                rospy.logwarn('-------------------')
        
    
        

if __name__ == '__main__':
    rospy.init_node('GalvoCalibrator')
    gc = GalvoCalibrator()
    gc.Main()


