#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import copy
import numpy as N
import tf
import threading

from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from geometry_msgs.msg import Point, Transform
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header
from patterngen.msg import MsgPattern
from patterngen.srv import *


class NullClass:
    pass
    

###############################################################################
###############################################################################
###############################################################################
# The class GalvoDirector subscribes to ArenaState, and draws the commanded
# pattern into the commanded frame, e.g. draw a grid at the origin of the 
# "Fly1" frame, or draw a circle at the origin of the "Arena" frame.
#
# Uses the PatternGen service to compute the various point locations. 
# Publishes GalvoDriver/pointcloud for the galvos to scan.
#
###############################################################################
###############################################################################
###############################################################################
class GalvoDirector:

    def __init__(self):
        self.initialized = False
        self.lock = threading.Lock()
        
        self.tfrx = tf.TransformListener()

        # Messages
        #self.subTF = rospy.Subscriber('tf', Transform, self.Transform_callback)
        queue_size_arenastate = rospy.get_param('tracking/queue_size_arenastate', 1)
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=queue_size_arenastate)

        self.subGalvoCommand = rospy.Subscriber('GalvoDirector/command', MsgGalvoCommand, self.GalvoCommand_callback, queue_size=2)
        self.pubGalvoPointCloud = rospy.Publisher('GalvoDriver/pointcloud', PointCloud, latch=True)
        self.pubGalvoPointCloudMm = rospy.Publisher('GalvoDriver/pointcloudmm', PointCloud, latch=True)

        # Attach to services.
        try:
            rospy.wait_for_service('GetPatternPoints')
            self.GetPatternPoints = rospy.ServiceProxy('GetPatternPoints', SrvGetPatternPoints)
        except (rospy.ServiceException, IOError), e:
            print "Service GetPatternPoints not found: %s" % e


        rospy.core.add_client_shutdown_hook(self.Preshutdown_callback)
        rospy.on_shutdown(self.OnShutdown_callback)
        
        # Calibration data (median values) to convert millimeters to volts, from "roslaunch galvodirector calibrator.launch".
        self.mx = rospy.get_param('galvodirector/mx', 0.0) #0.05111587
        self.bx = rospy.get_param('galvodirector/bx', 0.0) #-0.90610801
        self.my = rospy.get_param('galvodirector/my', 0.0) #-0.05003545
        self.by = rospy.get_param('galvodirector/by', 0.0) #3.15307329
        
        self.xBeamsink = rospy.get_param('galvodirector/xBeamsink', 0.0) # volts
        self.yBeamsink = rospy.get_param('galvodirector/yBeamsink', 0.0) # volts
        self.enable_laser = False
        
        self.arenastate = ArenaState()
        self.pointcloudtemplate_list = []
        self.pointcloud_list = []
        self.frameidPosition_list = []
        self.units = 'millimeters'
        
        self.pointcloudBeamsink = PointCloud(header=Header(frame_id='Arena', 
                                                           stamp=rospy.Time.now()),
                                             points=[Point(x=self.xBeamsink, 
                                                           y=self.yBeamsink, 
                                                           z=0.0)],
                                             channels=[ChannelFloat32(name='intensity',
                                                                      values=[0.0])])
        self.pointcloudBeamsinkMm = self.UnitsFromVoltsPointcloud(copy.deepcopy(self.pointcloudBeamsink))
                     
        
        
        self.timePrev = rospy.Time.now().to_sec()
        
        self.initialized = True


    def Transform_callback(self, tr):
        # If any of the frames in frameidPosition_list are updated, then publish the pointcloud.
        for x in tr.transforms:
            if (x.child_frame_id in self.frameidPosition_list):
                self.PublishPointcloud()
                break
            #rospy.logwarn ('Transform_callback() dt=%0.5f' % (rospy.Time.now().to_sec()-self.timePrev))  # Some of these are slow, e.g. 0.08, 0.1
            #self.timePrev = rospy.Time.now().to_sec()


    def ArenaState_callback(self, arenastate):
        with self.lock:
            self.arenastate = arenastate
            self.PublishPointcloud()
        
    
    def Preshutdown_callback(self, reason=None):
        self.enable_laser = False
        self.PublishPointcloud()
        #self.MoveToBeamsink()
        
        
    def OnShutdown_callback(self):
        pass


    # GalvoCommand_callback()
    # Receive the list of patterns, and store their pointcloud templates (centered at origin).
    #
    def GalvoCommand_callback(self, command):
        if (self.initialized):
            with self.lock:
                self.enable_laser = command.enable_laser
                self.units = command.units # Units apply to all the patterns.
                #rospy.logwarn('enable_laser: %s' % self.enable_laser)
                
                # If patterns were given, then load them into the pointcloudtemplates.
                if len(command.pattern_list) > 0:
                    # Regenerate all the patterns requested.  This "template" is the computed points centered at (0,0).
                    self.pointcloudtemplate_list = []
                    self.frameidPosition_list = []
                    
                    for iPattern in range(len(command.pattern_list)):
                        self.frameidPosition_list.append(command.pattern_list[iPattern].frameidPosition) # Save all the target frames for the TF callback.
                        
                        # Compute pattern points, if necessary.
                        if len(command.pattern_list[iPattern].points)==0:
                            gpp = self.GetPatternPoints(SrvGetPatternPointsRequest(pattern=command.pattern_list[iPattern]))
                            pattern = gpp.pattern
                        else:
                            pattern = command.pattern_list[iPattern]
                        
                        # Store the pointcloud templates.
                        self.pointcloudtemplate_list.append(self.PointCloudFromPoints(pattern.frameidPosition, pattern.points))
                    
                self.PublishPointcloud()


    # GetMaxPatternRate()
    # Get the fastest pattern update rate.
    #
    def GetMaxPatternRate(self, pattern_list):
        hzPatternMax = 0
        for pattern in pattern_list:
            hzPatternMax = max(hzPatternMax, pattern.hzPattern)

        return hzPatternMax
    
    
        
    # PublishPointcloud()
    # If laser enabled, then transform the pointcloudtemplates to their respective frames,
    # If laser disabled, then use the beamsink pointcloud.
    #
    # Publish points to the galvo driver.    
    #
    def PublishPointcloud(self):        

        if self.initialized:
            if self.enable_laser:
                self.pointcloud_list = []
                if len(self.pointcloudtemplate_list)>0 and len(self.arenastate.flies)>0:
                    for i in range(len(self.pointcloudtemplate_list)):
                        pointcloud_template = self.pointcloudtemplate_list[i]
                        #t1 = rospy.Time.now().to_sec()
                        
                        try:
                            pointcloud_template.header.stamp = self.tfrx.getLatestCommonTime('Arena', pointcloud_template.header.frame_id)
                        except tf.Exception:
                            pointcloud_template.header.stamp = self.arenastate.flies[0].header.stamp # BUG: Need to make this use the correct fly #.
    
                        #t2 = rospy.Time.now().to_sec()
                        if self.tfrx.canTransform('Arena', 
                                                  pointcloud_template.header.frame_id, 
                                                  pointcloud_template.header.stamp):
                            try:
                                pointcloud = self.tfrx.transformPointCloud('Arena', pointcloud_template)
                            except tf.Exception, e:
                                rospy.logwarn('Exception transforming pointcloud frame %s->%s: %s' % (pointcloud_template.header.frame_id, 'Arena', e))
                            else:
                                self.pointcloud_list.append(pointcloud)
                        else:
                            rospy.logwarn ('Cannot transform from frame %s at %s' % (pointcloud_template.header.frame_id, pointcloud_template.header.stamp))
        
                        #t3 = rospy.Time.now().to_sec()
                        #rospy.logwarn('GalvoDirector, stamp=%s, wait dt=%0.5f, transform dt=%0.5f' % (pointcloud_template.header.stamp,(t2-t1),(t3-t2))) # BUG: Occasional 0.1 sec times.
                        #rospy.logwarn ('now,pointcloud_template,%s,%s' % (rospy.Time.now(), pointcloud_template.header.stamp))
        
                        #rospy.logwarn('tfrx.getLatestCommonTime()=%s, stamp=%s' % (self.tfrx.getLatestCommonTime('Arena', pointcloud_template.header.frame_id),pointcloud_template.header.stamp))
    
    
                    # Publish a pointcloud in volts and mm.
                    pointcloudmm = self.GetUnifiedPointcloud(self.pointcloud_list)
                    pointcloudv = self.VoltsFromUnitsPointcloud(copy.deepcopy(pointcloudmm))
                    self.pubGalvoPointCloud.publish(pointcloudv)
                    self.pubGalvoPointCloudMm.publish(pointcloudmm)
            
            else: # not self.enable_laser
                
                # Point laser at the beam sink.
                self.pointcloudBeamsink.header.stamp=rospy.Time.now()
                self.pubGalvoPointCloud.publish(self.pointcloudBeamsink)
                self.pointcloudBeamsinkMm.header.stamp=rospy.Time.now()
                self.pubGalvoPointCloudMm.publish(self.pointcloudBeamsinkMm)
        

    # GetUnifiedPointcloud()
    # Combine multiple pointclouds into one pointcloud.
    #
    def GetUnifiedPointcloud(self, pointcloud_list):
        pointcloudUnified = PointCloud()
        pointcloudUnified.points = []
        pointcloudUnified.channels = []
        
        if len(pointcloud_list) > 0:
            pointcloudUnified.header = pointcloud_list[0].header  # They all have different headers, but just use the first one.
            pointcloudUnified.channels.append(ChannelFloat32(name='intensity', values=[]))
                                              
            for pointcloud in pointcloud_list:
                pointcloudUnified.points.extend(pointcloud.points)
                pointcloudUnified.channels[0].values.extend(pointcloud.channels[0].values)
        
        return pointcloudUnified
            
        
    # AddPatternToTemplates()
    # Append the requested pattern to the list of pointcloud templates.
    #
    def AddPatternToTemplates(self, req_gpp):
        if (self.initialized):
            resp_gpp = self.GetPatternPoints(req_gpp)
            pointcloud = self.PointCloudFromPoints('Arena', resp_gpp.pattern.points)
            self.pointcloudtemplate_list.append(pointcloud)
            #rospy.logwarn(resp_gpp.pattern.points)


    # VoltsFromUnitsPointcloud()
    # Inplace convert the pointcloud units to volts.
    #
    def VoltsFromUnitsPointcloud(self, pointcloud):
        if self.units == 'millimeters':
            for point in pointcloud.points:
                point.x *= self.mx
                point.x += self.bx

                point.y *= self.my
                point.y += self.by
                
                # Clip to 10
                point.x = min(point.x,+10.0)
                point.x = max(point.x,-10.0)
                point.y = min(point.y,+10.0)
                point.y = max(point.y,-10.0)
    
        elif self.units=='volts':
            pass

            
        return pointcloud
    

    # UnitsFromVoltsPointcloud()
    # Inplace convert the pointcloud units to volts.
    #
    def UnitsFromVoltsPointcloud(self, pointcloud):
        if self.units == 'millimeters':
            for point in pointcloud.points:
                point.x -= self.bx
                point.x /= self.mx

                point.y -= self.by
                point.y /= self.my
                
        elif self.units=='volts':
            pass

            
        return pointcloud
    

    # PointCloudFromPoints()
    # Reformat the given points as a pointcloud.
    #
    def PointCloudFromPoints(self, frameidPosition, points): # BUG: add frameidAngle            
        points_xy = [] 
        intensity = []
        for i in range(len(points)):
            points_xy.append(Point(x=points[i].x,
                                   y=points[i].y,
                                   z=0.0))
            intensity.append(1.0)# i.e. LASERON   #points[i].z)
            
        if len(intensity)>0:
            intensity[0] = 0.0   # Make sure laser is off when going to start of pattern.
        

        pointcloud = PointCloud(header=Header(frame_id=frameidPosition, stamp=rospy.Time.now()),
                                points=points_xy,
                                channels=[ChannelFloat32(name='intensity',
                                                         values=intensity),])
        return pointcloud                
            

    def Main(self):
        rospy.spin()
            


if __name__ == '__main__':
    rospy.init_node('GalvoDirector')
    gd = GalvoDirector()
    gd.Main()
    


