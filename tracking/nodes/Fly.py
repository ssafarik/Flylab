#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import copy
import rospy
import tf
import numpy as N
from tracking.msg import *
from arena_tf.srv import *
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState
from pythonmodules import filters
from pythonmodules import CircleFunctions


        
###############################################################################
###############################################################################
###############################################################################
# class Fly()
# Implements a fly or robot object in a 2D arena. 
# Updates its state based on an image "contour" from a single camera.
#
class Fly:
    def __init__(self, name=None, tfrx=None):
        self.initialized = False
        self.name = name

        self.tfrx = tfrx
        self.tfbx = tf.TransformBroadcaster()
        self.pubMarker = rospy.Publisher('visualization_marker', Marker)
        

        self.kfState = filters.KalmanFilter()
        self.lpAngleContour = filters.LowPassHalfCircleFilter(RC=rospy.get_param('tracking/rcFilterAngle', 0.1))
        self.lpAngleContour.SetValue(0.0)
        self.angleContourPrev = 0.0
        self.lpOffsetMag = filters.LowPassFilter(RC=1.0)
        self.lpOffsetMag.SetValue(0.0)
        self.lpOffsetAng = filters.LowPassCircleFilter(RC=0.001)
        self.lpOffsetAng.SetValue(0.0)
        self.maxOffset = 10.0
        self.ptOffset = Point(x=0, y=0, z=0)  # Vector from computed position to contour position (for robots only).
        self.angleOffsetPrev = 0.0
        self.stampPrev = rospy.Time.now()
        self.unwind = 0.0
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.
        self.dtForecast = rospy.get_param('tracking/dtForecast',0.15)
        
        # Orientation detection stuff.
        self.angleOfTravelRecent = 0.0
        self.lpFlip = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFilterFlip', 3.0))
        self.lpFlip.SetValue(0.0)
        self.contour = None
        self.speedThresholdForTravel = rospy.get_param ('tracking/speedThresholdForTravel', 5.0) # Speed that counts as "traveling".
        self.lpSpeed = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFilterSpeed', 0.2))
        self.lpSpeed.SetValue(0.0)
        self.speed = 0.0
        
        self.lpWx = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFilterAngularVel', 0.05))
        self.lpWy = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFilterAngularVel', 0.05))
        self.lpWz = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFilterAngularVel', 0.05))
        self.lpWx.SetValue(0.0)
        self.lpWy.SetValue(0.0)
        self.lpWz.SetValue(0.0)
        
        self.isVisible = False
        self.isDead = False # TO DO: dead fly detection.

        self.state = MsgFrameState()
        self.state.header.frame_id = 'Arena'
        self.state.pose.position.x = 0.0
        self.state.pose.position.y = 0.0
        self.state.pose.position.z = 0.0
        q = tf.transformations.quaternion_about_axis(0.0, (0,0,1))
        self.state.pose.orientation.x = q[0]    # Note: The orientation is ambiguous as to head/tail.  
        self.state.pose.orientation.y = q[1]    #       Check the self.lpFlip sign to resolve it:  use self.ResolvedAngleFromContour().
        self.state.pose.orientation.z = q[2]    #
        self.state.pose.orientation.w = q[3]    #
        self.state.velocity.linear.x = 0.0
        self.state.velocity.linear.y = 0.0
        self.state.velocity.linear.z = 0.0
        self.state.velocity.angular.x = 0.0
        self.state.velocity.angular.y = 0.0
        self.state.velocity.angular.z = 0.0


        self.eccMin = 999.9
        self.eccMax = 0.0
        self.eccSum = 1.0
        self.eccCount = 1
        
        self.areaMin = 999.9
        self.areaMax = 0.0
        self.areaSum = 0.0
        self.areaCount = 1
        
        self.robot_width = rospy.get_param ('robot/width', 1.0)
        self.robot_length = rospy.get_param ('robot/length', 1.0)
        self.robot_height = rospy.get_param ('robot/height', 1.0)
        
        rospy.logwarn('Fly() object added, name=%s' % name)
        self.timePrev = rospy.Time.now().to_sec()
        self.theta = 0.0
        self.initialized = True


    def PolarFromXy(self, x, y):
        mag = N.sqrt(x**2 + y**2)
        ang = N.arctan2(y, x)
        return (mag, ang)
    
    def XyFromPolar(self, mag, ang):
        x = mag * N.cos(ang)
        y = mag * N.sin(ang)
        return (x, y)
    
    
    def YawFromQuaternion(self, q):
        rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        return rpy[2]


    # SetAngleOfTravel()
    # If the speed surpasses a threshold, then update the direction of travel.
    def SetAngleOfTravel(self):
        angleOfTravel = N.arctan2(self.state.velocity.linear.y, self.state.velocity.linear.x) % (2.0*N.pi)
        if self.speed > self.speedThresholdForTravel: 
            self.angleOfTravelRecent = angleOfTravel


    # GetNextFlipUpdate()
    #   Using self.contour, self.state, and self.angleOfTravelRecent,
    #   we determine a value to use for updating the "flip" filter,
    #   which varies on [-1,+1], and the sign of which determines if to flip.
    #
    # Returns the chosen flip value.
    #   
    def GetNextFlipUpdate(self):
        # Get the prior flip value.
        flipvalueMagPrev = self.lpFlip.GetValue()
        if flipvalueMagPrev is None:
            flipvalueMagPrev = 0.0


        angleContour         = self.lpAngleContour.GetValue()
        angleContour_flipped = angleContour + N.pi
        
        # Compare distances between angles of (travel - contour) and (travel - flippedcontour)        
        dist         = N.abs(CircleFunctions.circle_dist(angleContour,         self.angleOfTravelRecent))
        dist_flipped = N.abs(CircleFunctions.circle_dist(angleContour_flipped, self.angleOfTravelRecent))
        
        # Choose the better orientation.
        if dist < dist_flipped:
            flipvalueSign =  1.0 # Not flipped
        else:
            flipvalueSign = -1.0 # Flipped


        if self.contour.ecc is not None:
            eccmetric = (self.contour.ecc + 1/self.contour.ecc) - 1 #self.contour.ecc#
        else:
            eccmetric = 1.0
        #rospy.logwarn('%s: eccmetric=%0.2f' % (self.name, eccmetric))


        flipvalueMag = flipvalueMagPrev#N.abs(flipvalueMagPrev)
        if (eccmetric > 1.5):
            # Weight the prev/new values based on speed.                
            #alpha = 10.0/(10.0+self.speed)
            #flipvalueMag = (alpha * N.abs(flipvalueMagPrev)) + ((1.0-alpha) * 1.0) 
    
            # Either use +-1, or use the previous value.
            if (self.speed > self.speedThresholdForTravel):
                flipvalueMag = 1.0

        
        flipvalueUpdate = flipvalueSign * flipvalueMag
        
        #if 'Fly1' in self.name:
        #    rospy.logwarn('%s: flipvalue %0.2f, %0.2f, %0.2f' % (self.name, flipvalueMagPrev,flipvalueWeighted,flipvalueNew))


        return flipvalueUpdate
    

        
    # UpdateFlipState()
    #   Choose among the various possibilities:
    #   lpFlip>0  -->  angleResolved = (angle of contour)
    #     OR
    #   lpFlip<=0 -->  angleResolved = (angle of contour + pi)
    #
    # Updates the flip state in self.lpFlip
    #   
    def UpdateFlipState(self):
        # Update the flip filter.
        flipvaluePre = self.GetNextFlipUpdate()
            
        if (self.speed > self.speedThresholdForTravel):
            flipvaluePost = self.lpFlip.Update(flipvaluePre, self.contour.header.stamp.to_sec())
                
        # Contour angle only ranges on [-pi,-0].  If it wraps, then change the lpFlip sign.
        d = N.abs(CircleFunctions.circle_dist(self.lpAngleContour.GetValue(), self.angleContourPrev))
        if (d > (N.pi/2.0)):
            self.lpFlip.SetValue(-self.lpFlip.GetValue())


    def GetResolvedAngle(self):
        angleF = self.lpAngleContour.GetValue()
                
        if self.lpFlip.GetValue()<0 and ('Robot' not in self.name):
            angleResolved = (angleF + N.pi) % (2.0*N.pi)
        else:
            angleResolved = angleF


        return angleResolved
            
                            

    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contour, ptComputed):
        if self.initialized:
            self.angleContourPrev = self.lpAngleContour.GetValue()
            self.contour = contour
            
            # Update the position & orientation filters
            self.isVisible = False            
            if (self.contour.x is not None) and \
               (self.contour.y is not None) and \
               (self.contour.angle is not None) and \
               (self.contour.area is not None) and \
               (self.contour.ecc is not None):
                # Update min/max ecc & area.
                if contour.ecc < self.eccMin:
                    self.eccMin = contour.ecc
                if self.eccMax < contour.ecc:
                    self.eccMax = contour.ecc
                if contour.area < self.areaMin:
                    self.areaMin = contour.area
                if self.areaMax < contour.area:
                    self.areaMax = contour.area
                self.eccSum += contour.ecc
                self.eccCount += 1
                self.areaSum += contour.area
                self.areaCount += 1
                
                self.isVisible = True
                (xKalman,yKalman,vxKalman,vyKalman) = self.kfState.Update((self.contour.x, self.contour.y), contour.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                #(x,y) = (self.contour.x,self.contour.y) # Unfiltered.
                self.lpAngleContour.Update(self.contour.angle, contour.header.stamp.to_sec())#self.contour.header.stamp.to_sec())
                
                
                if N.abs(self.contour.x)>9999 or N.abs(xKalman)>9999:
                    rospy.logwarn ('FLY LARGE CONTOUR, x,x=%s, %s.  Check your background image, lighting, and the parameter tracking/diff_threshold.' % (self.contour.x, xKalman))


            else: # We don't have a contour.
                #rospy.logwarn('FLY No contour seen for %s; check your nFlies parameter.' % self.name)
                (xKalman, yKalman, vxKalman, vyKalman) = self.kfState.Update(None, contour.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)

                
            # If good data, then use it.
            if (xKalman is not None) and (yKalman is not None):
                x = xKalman
                y = yKalman
                z = zKalman
                vx = vxKalman
                vy = vyKalman
                vz = vzKalman
            else: # Use the unfiltered data for the case where the filters return None results.
                rospy.logwarn('Object %s not yet initialized at %s, %s' % (self.name, contour.header.stamp.to_sec(), [self.contour.x, self.contour.y]))
                x = 0.0#self.contour.x
                y = 0.0#self.contour.y
                z = 0.0
                vx = 0.0
                vy = 0.0
                vz = 0.0

                
            # Store the latest state.
            self.state.header.stamp = self.contour.header.stamp
            self.state.pose.position.x = x
            self.state.pose.position.y = y
            self.state.pose.position.z = z
            #self.state.pose.orientation comes from self.GetResolvedAngle() later.
            self.state.velocity.linear.x = vx
            self.state.velocity.linear.y = vy
            self.state.velocity.linear.z = vz

            # Get the angular velocities.
            if self.isVisible:
                try:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = self.tfrx.lookupTwist(self.name, self.state.header.frame_id, self.state.header.stamp-self.dtVelocity, self.dtVelocity)
                except (tf.Exception, AttributeError), e:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                    #rospy.logwarn('lookupTwist() Exception: %s' % e)
            else:
                ((vx2,vy2,vz2),(wx,wy,wz)) = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

            # Fix glitches due to flip orientation.
            if wz>6.27:
                wz = self.lpWz.GetValue()
                
            self.state.velocity.angular.x = self.lpWx.Update(wx, self.state.header.stamp.to_sec())
            self.state.velocity.angular.y = self.lpWy.Update(wy, self.state.header.stamp.to_sec())
            self.state.velocity.angular.z = self.lpWz.Update(wz, self.state.header.stamp.to_sec())
                
            speedPre = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y, self.state.velocity.linear.z])
            self.speed = self.lpSpeed.Update(speedPre, self.state.header.stamp.to_sec())
                
#            if 'Fly1' in self.name:
#                #rospy.logwarn ('FLY vel=%s' % [((vx2,vy2,vz2),(wx,wy,wz))])
#                rospy.logwarn ('FLY speed=%0.2f, vel=(%0.2f,%0.2f,%0.2f)' % (self.speed, self.state.velocity.linear.x, self.state.velocity.linear.y, self.state.velocity.linear.z))
#                #rospy.logwarn('speed=%0.2f, flip=%0.2f, stamp=%s' % (self.speed, self.lpFlip.GetValue(), self.state.header.stamp))

            # Update the most recent angle of travel.
            self.SetAngleOfTravel()
            self.UpdateFlipState()
            angle = self.GetResolvedAngle()
            
            (self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w) = tf.transformations.quaternion_about_axis(angle, (0,0,1))
                
            # Update the tool offset.
            if ptComputed is not None:
#                    marker = Marker(header=Header(stamp=rospy.Time.now(),
#                                                        frame_id='Arena'),
#                                          ns='kalman',
#                                          id=4,
#                                          type=Marker.SPHERE,
#                                          action=0,
#                                          pose=Pose(position=Point(x=x, 
#                                                                   y=y, 
#                                                                   z=z)),
#                                          scale=Vector3(x=3.0,
#                                                        y=3.0,
#                                                        z=3.0),
#                                          color=ColorRGBA(a=0.5,
#                                                          r=0.5,
#                                                          g=0.5,
#                                                          b=0.5),
#                                          lifetime=rospy.Duration(1.0))
#                    self.pubMarker.publish(marker)
#                    marker = Marker(header=Header(stamp=rospy.Time.now(),
#                                                        frame_id='Arena'),
#                                          ns='computed',
#                                          id=5,
#                                          type=Marker.SPHERE,
#                                          action=0,
#                                          pose=Pose(position=Point(x=ptComputed.x, 
#                                                                   y=ptComputed.y, 
#                                                                   z=ptComputed.z)),
#                                          scale=Vector3(x=3.0,
#                                                        y=3.0,
#                                                        z=3.0),
#                                          color=ColorRGBA(a=0.5,
#                                                          r=0.1,
#                                                          g=0.1,
#                                                          b=1.0),
#                                          lifetime=rospy.Duration(1.0))
#                    self.pubMarker.publish(marker)

                # The offset.
                xOffset = x-ptComputed.x
                yOffset = y-ptComputed.y
                
                # Filter the offset as magnitude & angle.
                (magOffset,angleOffset)=self.PolarFromXy(xOffset,yOffset)
                angleOffset += self.unwind
                if angleOffset-self.angleOffsetPrev > N.pi:
                    self.unwind -= 2.0*N.pi
                    angleOffset -= 2.0*N.pi
                elif angleOffset-self.angleOffsetPrev < -N.pi:
                    self.unwind += 2.0*N.pi# Use the unfiltered data for the case where the filters return None results.
                    angleOffset += 2.0*N.pi
                self.angleOffsetPrev = angleOffset
                
                angleOffsetF = self.lpOffsetAng.Update(angleOffset, contour.header.stamp.to_sec())
                (self.ptOffset.x,self.ptOffset.y) = self.XyFromPolar(N.clip(self.lpOffsetMag.Update(magOffset, contour.header.stamp.to_sec()), -self.maxOffset, self.maxOffset),
                                                                     angleOffsetF)
            else:
                self.ptOffset = Point(x=0, y=0, z=0)
            

                
            # Send the Raw transform.
            if self.isVisible:
                self.tfbx.sendTransform((self.contour.x, 
                                         self.contour.y, 
                                         0.0),
                                        tf.transformations.quaternion_about_axis(self.contour.angle, (0,0,1)),
                                        self.contour.header.stamp,
                                        self.name+"Contour",
                                        "Arena")
            
            # Send the Filtered transform.
            if self.state.pose.position.x is not None:
                q = self.state.pose.orientation
                self.tfbx.sendTransform((self.state.pose.position.x, 
                                         self.state.pose.position.y, 
                                         self.state.pose.position.z),
                                        (q.x, q.y, q.z, q.w),
                                        self.state.header.stamp,
                                        self.name,
                                        self.state.header.frame_id)
                

            # Send the Forecast transform.
            if self.state.pose.position.x is not None:
                #self.dtForecast = rospy.get_param('tracking/dtForecast',0.25)
                poseForecast = Pose()#copy.copy(self.state.pose)
                poseForecast.position.x = self.state.pose.position.x + self.state.velocity.linear.x * self.dtForecast
                poseForecast.position.y = self.state.pose.position.y + self.state.velocity.linear.y * self.dtForecast
                poseForecast.position.z = self.state.pose.position.z + self.state.velocity.linear.z * self.dtForecast
                q = self.state.pose.orientation
                self.tfbx.sendTransform((poseForecast.position.x, 
                                         poseForecast.position.y, 
                                         poseForecast.position.z),
                                        (q.x, q.y, q.z, q.w),
                                        self.state.header.stamp,
                                        self.name+'Forecast',
                                        self.state.header.frame_id)
                

            # Publish a 3D model marker for the robot.
            if 'Robot' in self.name:
                markerRobot = Marker(header=Header(stamp = self.state.header.stamp,
                                                    frame_id='Arena'),
                                      ns='robot',
                                      id=1,
                                      type=Marker.CYLINDER,
                                      action=0,
                                      pose=Pose(position=Point(x=self.state.pose.position.x, 
                                                               y=self.state.pose.position.y, 
                                                               z=self.state.pose.position.z),
                                                orientation=self.state.pose.orientation),
                                      scale=Vector3(x=self.robot_width,
                                                    y=self.robot_length,
                                                    z=self.robot_height),
                                      color=ColorRGBA(a=0.7,
                                                      r=0.5,
                                                      g=0.5,
                                                      b=0.5),
                                      lifetime=rospy.Duration(0.5))
                self.pubMarker.publish(markerRobot)

        
    

