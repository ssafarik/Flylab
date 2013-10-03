#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('experiments')
import rospy
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker



class Statefilter:
    def __init__(self):
        self.pubMarker = rospy.Publisher('visualization_marker', Marker) 
        
            
    # PublishStatefilterMarkers()
    # Publish markers to show where the state filter regions are located.
    #  
    def PublishMarkers(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                positionLo_dict = poseLo_dict['position']
                positionHi_dict = poseHi_dict['position']
                (xLo,xHi) = (-9999,+9999)
                (yLo,yHi) = (-9999,+9999)
                (zLo,zHi) = (0,0.1)
                if ('x' in positionLo_dict):
                    xLo = positionLo_dict['x'] 
                    xHi = positionHi_dict['x']
                if ('y' in positionLo_dict):
                    yLo = positionLo_dict['y'] 
                    yHi = positionHi_dict['y']
                if ('z' in positionLo_dict):
                    zLo = positionLo_dict['z'] 
                    zHi = positionHi_dict['z']
    
                if ('x' in positionLo_dict) or ('y' in positionLo_dict) or ('z' in positionLo_dict):
                    markerTarget = Marker(header=Header(stamp = state.header.stamp,
                                                        frame_id='/Arena'),
                                          ns='statefilter',
                                          id=1,
                                          type=Marker.CUBE,
                                          action=0,
    #                                          type=Marker.LINE_STRIP
    #                                          points=[Point(x=xLo,y=yLo,z=zLo),Point(x=xHi,y=yLo,z=zLo),Point(x=xHi,y=yHi,z=zLo),Point(x=xLo,y=yHi,z=zLo),Point(x=xLo,y=yLo,z=zLo)],
    #                                          scale=Vector3(x=0.2, y=0.0, z=0.0),
                                          pose=Pose(position=Point(x=(xLo+xHi)/2, 
                                                                   y=(yLo+yHi)/2, 
                                                                   z=(zLo+zHi)/2)),
                                          scale=Vector3(x=xHi-xLo,
                                                        y=yHi-yLo,
                                                        z=zHi-zLo),
                                          color=ColorRGBA(a=0.1,
                                                          r=1.0,
                                                          g=1.0,
                                                          b=1.0),
                                          lifetime=rospy.Duration(1.0))
                    self.pubMarker.publish(markerTarget)
            
            
        
        
    # InStatefilterRange()
    # Check if the given state falls in the given region.
    # For statefilterCriteria=="inclusive", if the given state is within all the terms, then the filter returns True.
    # For statefilterCriteria=="exclusive", if the given state is within all the terms, then the filter returns False.
    #
    # We have to manually go through each of the entries in the dict, rather than
    # using a MsgFrameState, since we need the dict to only contain the entries
    # we care about, and the MsgFrameState always contains them all.
    #  
    def InRange(self, state, statefilterLo_dict, statefilterHi_dict, statefilterCriteria):
        rv = True
        if 'pose' in statefilterLo_dict:
            poseLo_dict = statefilterLo_dict['pose'] 
            poseHi_dict = statefilterHi_dict['pose']
             
            if 'position' in poseLo_dict:
                 positionLo_dict = poseLo_dict['position']
                 positionHi_dict = poseHi_dict['position']
                 if 'x' in positionLo_dict:
                     xLo = positionLo_dict['x'] 
                     xHi = positionHi_dict['x']
                     if (state.pose.position.x < xLo) or (xHi < state.pose.position.x):
                         rv = False   
                 if 'y' in positionLo_dict:
                     yLo = positionLo_dict['y'] 
                     yHi = positionHi_dict['y']
                     if (state.pose.position.y < yLo) or (yHi < state.pose.position.y):
                         rv = False   
                 if 'z' in positionLo_dict:
                     zLo = positionLo_dict['z'] 
                     zHi = positionHi_dict['z']
                     if (state.pose.position.z < zLo) or (zHi < state.pose.position.z):
                         rv = False   
                         
            if 'orientation' in poseLo_dict:
                 orientationLo_dict = poseLo_dict['orientation']
                 orientationHi_dict = poseHi_dict['orientation']
                 if 'x' in orientationLo_dict:
                     xLo = orientationLo_dict['x'] 
                     xHi = orientationHi_dict['x']
                     if (state.pose.orientation.x < xLo) or (xHi < state.pose.orientation.x):
                         rv = False   
                 if 'y' in orientationLo_dict:
                     yLo = orientationLo_dict['y'] 
                     yHi = orientationHi_dict['y']
                     if (state.pose.orientation.y < yLo) or (yHi < state.pose.orientation.y):
                         rv = False   
                 if 'z' in orientationLo_dict:
                     zLo = orientationLo_dict['z'] 
                     zHi = orientationHi_dict['z']
                     if (state.pose.orientation.z < zLo) or (zHi < state.pose.orientation.z):
                         rv = False   
                 if 'w' in orientationLo_dict:
                     wLo = orientationLo_dict['w'] 
                     wHi = orientationHi_dict['w']
                     if (state.pose.orientation.w < wLo) or (wHi < state.pose.orientation.w):
                         rv = False   
    
        if 'velocity' in statefilterLo_dict:
            velocityLo_dict = statefilterLo_dict['velocity'] 
            velocityHi_dict = statefilterHi_dict['velocity']
             
            if 'linear' in velocityLo_dict:
                 linearLo_dict = velocityLo_dict['linear']
                 linearHi_dict = velocityHi_dict['linear']
                 if 'x' in linearLo_dict:
                     xLo = linearLo_dict['x'] 
                     xHi = linearHi_dict['x']
                     if (state.velocity.linear.x < xLo) or (xHi < state.velocity.linear.x):
                         rv = False   
                 if 'y' in linearLo_dict:
                     yLo = linearLo_dict['y'] 
                     yHi = linearHi_dict['y']
                     if (state.velocity.linear.y < yLo) or (yHi < state.velocity.linear.y):
                         rv = False   
                 if 'z' in linearLo_dict:
                     zLo = linearLo_dict['z'] 
                     zHi = linearHi_dict['z']
                     if (state.velocity.linear.z < zLo) or (zHi < state.velocity.linear.z):
                         rv = False   
                         
            if 'angular' in velocityLo_dict:
                 angularLo_dict = velocityLo_dict['angular']
                 angularHi_dict = velocityHi_dict['angular']
                 if 'x' in angularLo_dict:
                     xLo = angularLo_dict['x'] 
                     xHi = angularHi_dict['x']
                     if (state.velocity.angular.x < xLo) or (xHi < state.velocity.angular.x):
                         rv = False   
                 if 'y' in angularLo_dict:
                     yLo = angularLo_dict['y'] 
                     yHi = angularHi_dict['y']
                     if (state.velocity.angular.y < yLo) or (yHi < state.velocity.angular.y):
                         rv = False   
                 if 'z' in angularLo_dict:
                     zLo = angularLo_dict['z'] 
                     zHi = angularHi_dict['z']
                     if (state.velocity.angular.z < zLo) or (zHi < state.velocity.angular.z):
                         rv = False
                     
        if 'speed' in statefilterLo_dict:
            speedLo = statefilterLo_dict['speed'] 
            speedHi = statefilterHi_dict['speed']
            if (state.speed < speedLo) or (speedHi < state.speed):
                rv = False   
    
        if (statefilterCriteria=="exclusive"):
            rv = not rv
            
        return rv
    
    
