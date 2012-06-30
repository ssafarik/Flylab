#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import tf
import numpy as N
import threading

from galvodirector.msg import MsgGalvoCommand
from tracking.msg import ArenaState
from plate_tf.srv import PlateCameraConversion
from geometry_msgs.msg import Point, PointStamped, Polygon, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header
from flycore.msg import MsgFrameState
from patterngen.msg import MsgPattern
from patterngen.srv import *


class NullClass:
    pass
    

###############################################################################
###############################################################################
###############################################################################
# The class GalvoDirector subscribes to ArenaState, and uses the PatternGen 
# service to compute the various point locations. 
# Publishes galvo_points for the galvos to scan.
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
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback, queue_size=2)
        self.subGalvoCommand = rospy.Subscriber('GalvoDirector/command', MsgGalvoCommand, self.GalvoCommand_callback, queue_size=2)
        self.pubGalvoPointCloud = rospy.Publisher('GalvoDriver/pointcloud', PointCloud)

        # Attach to services.
        try:
            rospy.wait_for_service('GetPatternPoints')
            self.GetPatternPoints = rospy.ServiceProxy('GetPatternPoints', SrvGetPatternPoints)
        except (rospy.ServiceException, IOError), e:
            print "Service GetPatternPoints not found: %s" % e

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.frameid_target_list = []
        self.pointcloudtemplate_list = []
        self.pointcloud_list = []
        self.units = 'millimeters'
        
        # Calibration data from Calibrator.py
        self.mx =  0.0602
        self.bx = -0.581
        self.my = -0.0582
        self.by =  3.490

        self.initialized = True


    def OnShutdown_callback(self):
        pass


    # GalvoCommand_callback()
    # Receive the list of patterns, and store their pointcloud templates (centered at origin).
    #
    def GalvoCommand_callback(self, command):
        if (self.initialized):
            with self.lock:
                # Regenerate all the patterns requested.  This "template" is the computed points centered at (0,0).
                self.pointcloudtemplate_list = []
                self.frameid_target_list = []
                
                for iPattern in range(len(command.pattern_list)):
                    
                    # If no pattern points, then compute them.
                    if len(command.pattern_list[iPattern].points)==0:
                        gpp = self.GetPatternPoints(SrvGetPatternPointsRequest(pattern=command.pattern_list[iPattern]))
                        pattern = gpp.pattern
                    else:
                        pattern = command.pattern_list[iPattern]
                    
                    # Store the pointcloud for later.
                    self.pointcloudtemplate_list.append(self.PointCloudFromPoints(pattern.frame_id, pattern.points))
                    self.frameid_target_list.append(command.frameid_target_list[iPattern])
                    self.units = command.units # Units apply to all the patterns.
                        
                self.PublishPointcloud()


    def ArenaState_callback(self, arenastate):
        with self.lock:
            self.arenastate = arenastate
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
    # Transform the pointcloudtemplates onto their respective frames,
    # then post it to the driver.
    #
    def PublishPointcloud(self):
        if self.initialized:
            self.pointcloud_list = []
            if len(self.pointcloudtemplate_list) > 0:
                for i in range(len(self.pointcloudtemplate_list)):
                    frame_id_target = self.frameid_target_list[i]
                    pointcloud_template = self.pointcloudtemplate_list[i]

                    try:
                        pointcloud_template.header.stamp = self.arenastate.flies[0].header.stamp
                    except:
                        pointcloud_template.header.stamp = rospy.Time.now()

                    try:
                        self.tfrx.waitForTransform(frame_id_target, 
                                                   pointcloud_template.header.frame_id, 
                                                   pointcloud_template.header.stamp, 
                                                   rospy.Duration(1.0))
                    except tf.Exception, e:
                        rospy.logwarn('Exception waiting for transform pointcloud=%s, frames %s -> %s: %s' % (pointcloud_template.header, pointcloud_template.header.frame_id, frame_id_target, e))
                        
                    try:
                        pointcloud = self.tfrx.transformPointCloud(frame_id_target, pointcloud_template)
                    except tf.Exception, e:
                        rospy.logwarn('Exception transforming pointcloud=%s, frames %s -> %s: %s' % (pointcloud_template.header, pointcloud_template.header.frame_id, frame_id_target, e))
                    else:
                        self.pointcloud_list.append(pointcloud)
            
                self.pubGalvoPointCloud.publish(self.VoltsFromUnitsPointcloud(self.GetUnifiedPointcloud(self.pointcloud_list)))
        

    # GetUnifiedPointcloud()
    # Combine multiple pointclouds into one pointcloud.
    #
    def GetUnifiedPointcloud(self, pointcloud_list):
        pointcloudUnified = PointCloud()
        pointcloudUnified.points = []
        pointcloudUnified.channels = []
        
        if len(pointcloud_list) > 0:
            pointcloudUnified.header = pointcloud_list[0].header  # They all have different headers, but just use the first one.
            for pointcloud in pointcloud_list:
                pointcloudUnified.points.extend(pointcloud.points)
                pointcloudUnified.channels.extend(pointcloud.channels)
        
        return pointcloudUnified
            
        
    # AddPatternToTemplates()
    # Append the requested pattern to the list of pointcloud templates.
    #
    def AddPatternToTemplates(self, req_gpp):
        if (self.initialized):
            resp_gpp = self.GetPatternPoints(req_gpp)
            pointcloud = self.PointCloudFromPoints('Plate', resp_gpp.pattern.points)
            self.pointcloudtemplate_list.append(pointcloud)
            self.frameid_target_list.append('Fly1')
            #rospy.logwarn(resp_gpp.pattern.points)


    # VoltsFromUnitsPointcloud()
    # Convert the pointcloud units into volts.
    #
    def VoltsFromUnitsPointcloud(self, pointcloud):
        if self.units == 'millimeters':
            for point in pointcloud.points:
                point.x *= self.mx
                point.x += self.bx

                point.y *= self.my
                point.y += self.by
    
        elif self.units=='volts':
            pass

            
        return pointcloud
    

    # PointCloudFromPoints()
    # Reformat the given points as a pointcloud.
    #
    def PointCloudFromPoints(self, frame_id, points):            
        points_xy = [] 
        intensity = []
        for i in range(len(points)):
            points_xy.append(Point(x=points[i].x,
                                   y=points[i].y,
                                   z=0.0))
            intensity.append(1.0)# i.e. LASERON   #points[i].z)
            
        if len(intensity)>0:
            intensity[0] = 0.0   # Make sure laser is off when going to start of pattern.
        

        pointcloud = PointCloud(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                                points=points_xy,
                                channels=[ChannelFloat32(name='intensity',
                                                         values=intensity),])
        return pointcloud                
            

    def Main(self):
#        while not rospy.is_shutdown():
#            self.PublishPointcloud() 
#            self.rosRate.sleep()
        rospy.spin()
            


if __name__ == '__main__':
    rospy.init_node('GalvoDirector')
    gd = GalvoDirector()
    gd.Main()
    


