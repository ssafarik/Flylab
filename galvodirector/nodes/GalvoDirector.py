#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('galvodirector')
import rospy
import tf
import numpy as N
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

        self.tfrx = tf.TransformListener()

        # Messages
        self.subArenaState = rospy.Subscriber('ArenaState', ArenaState, self.ArenaState_callback)
        self.subGalvoCommand = rospy.Subscriber('GalvoDirector/command', MsgGalvoCommand, self.GalvoCommand_callback)
        self.pubGalvoPointCloud = rospy.Publisher('GalvoDriver/pointcloud', PointCloud)

        # Attach to services.
        try:
            rospy.wait_for_service('GetPatternPoints')
            self.GetPatternPoints = rospy.ServiceProxy('GetPatternPoints', SrvGetPatternPoints)
        except (rospy.ServiceException, IOError), e:
            print "Service GetPatternPoints not found: %s" % e

        rospy.on_shutdown(self.OnShutdown_callback)
        
        self.req_gpp = SrvGetPatternPointsRequest()
        self.req_gpp.pattern.mode       = 'byshape'
        self.req_gpp.pattern.shape      = 'flylogo'
        self.req_gpp.pattern.frame      = 'Plate'
        self.req_gpp.pattern.hzPattern  = 1.0
        self.req_gpp.pattern.hzPoint    = 100.0
        self.req_gpp.pattern.count      = 1
        self.req_gpp.pattern.points     = []
        self.req_gpp.pattern.radius     = 20 # At the moment this is volts.
        self.req_gpp.pattern.preempt    = False
        
        self.frameid_target_list = []
        self.pointcloudtemplate_list = []
        self.pointcloud_list = []

        self.initialized = True
        self.AddPatternToTemplates(self.req_gpp)


    def OnShutdown_callback(self):
        self.req_gpp.pattern.shape      = 'flylogo'
        #self.SendPattern(self.req_gpp)


    # GalvoCommand_callback()
    # Receive the list of patterns, and store their pointcloud templates (centered at origin).
    #
    def GalvoCommand_callback(self, command):
        if (self.initialized):
            # Regenerate all the patterns requested.  This "template" is the computed points centered at (0,0).
            self.pointcloudtemplate_list = []
            self.frameid_target_list = []
            
            for iPattern in range(len(command.pattern_list)):
                
                # If no points, then compute them.
                if len(command.pattern_list[i].points)==0:
                    gpp = self.GetPatternPoints(GetPatternPointsRequest(pattern=command.pattern_list[i]))
                    points = gpp.pattern.points
                else:
                    points = pattern.points

                points[0].z = 0.0   # Laser line to next pattern is off.
    
                # Store the pointcloud for later.
                self.pointcloudtemplate_list.append(self.PointCloudFromPoints(pattern.frame_id, points))
                self.frameid_target_list.append(command.frameid_target_list[i])

    
    # Each time we get an ArenaState message, transform the pointcloudtemplates onto their respective frames,
    # then update the DAQ.
    def ArenaState_callback(self, arenastate):
        if self.initialized:
            self.pointcloud_list = []
            if len(self.pointcloudtemplate_list) > 0:
                for i in range(len(self.pointcloudtemplate_list)):
                    self.pointcloudtemplate_list[i].header.stamp = rospy.Time.now()
                    try:
                        self.tfrx.waitForTransform(self.frameid_target_list[i], 
                                                   self.pointcloudtemplate_list[i].header.frame_id, 
                                                   self.pointcloudtemplate_list[i].header.stamp, 
                                                   rospy.Duration(1.0))
                    except tf.Exception, e:
                        rospy.logwarn('Exception waiting for transform pointcloud=%s, frames %s -> %s: %s' % (self.pointcloudtemplate_list[i].header, self.pointcloudtemplate_list[i].header.frame_id, self.frameid_target_list[i], e))
                        
                    try:
                        self.pointcloud_list.append(self.tfrx.transformPointCloud(self.frameid_target_list[i], self.pointcloudtemplate_list[i]))
                    except tf.Exception, e:
                        rospy.logwarn('Exception transforming pointcloud=%s, frames %s -> %s: %s' % (self.pointcloudtemplate_list[i].header, self.pointcloudtemplate_list[i].header.frame_id, self.frameid_target_list[i], e))
            
                self.pubGalvoPointCloud.publish(self.RescalePointcloud(self.GetUnifiedPointcloud(self.pointcloud_list)))
        

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
        if (self.initialized):# and (len(self.pointcloud_list)>0):
            resp_gpp = self.GetPatternPoints(req_gpp)
            pointcloud = self.PointCloudFromPoints('Plate', resp_gpp.pattern.points)
            self.pointcloudtemplate_list.append(pointcloud)
            self.frameid_target_list.append('Fly1')
            #rospy.logwarn(resp_gpp.pattern.points)


    def RescalePointcloud(self, pointcloud):
        for point in pointcloud.points:
            point.y += 120

            point.x *= 0.02
            point.y *= 0.02
            
        return pointcloud
    

    def PointCloudFromPoints(self, frame_id, points):            
        points_xy = [] 
        intensity = []
        for i in range(len(points)):
            points_xy.append(Point(x=points[i].x,
                                   y=points[i].y,
                                   z=0.0))
            intensity.append(points[i].z)

        pointcloud = PointCloud(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                                points=points_xy,
                                channels=[ChannelFloat32(name='intensity',
                                                         values=intensity),])
        return pointcloud                
            
        

if __name__ == '__main__':
    rospy.init_node('GalvoDirector')
    gd = GalvoDirector()

    rospy.spin()


