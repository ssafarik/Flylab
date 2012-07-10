#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import copy
import rospy
import tf
import numpy as N
from tracking.msg import *
from plate_tf.srv import *
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState
import filters
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
        self.lpAngleF = filters.LowPassHalfCircleFilter(RC=rospy.get_param('tracking/rcAngleFilter', 0.1))
        #self.lpOffsetX = filters.LowPassFilter(RC=0.1)
        #self.lpOffsetY = filters.LowPassFilter(RC=0.1)
        self.lpOffsetMag = filters.LowPassFilter(RC=1.0)
        self.lpOffsetAng = filters.LowPassCircleFilter(RC=0.001)
        self.ptOffset = Point(x=0, y=0, z=0)  # Vector from computed position to contour position (for robots only).
        self.ptPPrev = Point(x=0, y=0, z=0)
        self.angPrev = 0.0
        self.angle = 0.0
        self.stampPrev = rospy.Time.now()
        self.unwind = 0.0
        self.dtVelocity = rospy.Duration(rospy.get_param('tracking/dtVelocity', 0.2)) # Interval over which to calculate velocity.
        
        # Orientation detection stuff.
        self.angleOfTravelRecent = None
        self.flip = False
        self.lpFlip = filters.LowPassFilter(RC=rospy.get_param('tracking/rcFlipFilter', 3.0))
        self.contour = None
        self.speedThresholdForTravel = rospy.get_param ('tracking/speedThresholdForTravel', 5.0) # Speed that counts as "traveling".
        self.maxOffset = 10.0
        
        self.isVisible = False
        self.isDead = False # TO DO: dead fly detection.

        self.state = MsgFrameState()
        self.state.header.frame_id = 'Plate'
        self.state.pose.position.x = 0.0
        self.state.pose.position.y = 0.0
        self.state.pose.position.z = 0.0
        q = tf.transformations.quaternion_about_axis(0.0, (0,0,1))
        self.state.pose.orientation.x = q[0]    # Note: The orientation is ambiguous as to head/tail.  
        self.state.pose.orientation.y = q[1]    #       Check the self.flip status to resolve it:  use self.ResolvedAngleFromContour().
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
        speed = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y])
        if speed > self.speedThresholdForTravel: 
            self.angleOfTravelRecent = angleOfTravel


    # GetNextFlipValue()
    #   Using self.contour, self.state, and self.angleOfTravelRecent,
    #   we determine a value to use for updating the "flip" filter,
    #   which varies on [-1,+1], and the sign of which determines if to flip.
    #
    # Returns the chosen flip value.
    #   
    def GetNextFlipValue(self):
        # Get the prior flip value.
        flipValuePrev = self.lpFlip.GetValue()
        if flipValuePrev is None:
            flipValuePrev = 0.0

        # angleContour is ambiguous mod pi radians
        eccmetric = (self.contour.ecc + 1/self.contour.ecc) - 1
        #rospy.loginfo('%s: eccmetric=%0.2f' % (self.name, eccmetric))
        if (eccmetric > 1.5):
            angleContour         = self.contour.angle #self.YawFromQuaternion(self.state.pose.orientation)
            angleContour_flipped = angleContour + N.pi
            
            # Most recent angle of travel.
            if self.angleOfTravelRecent is not None:
                angleOfTravel = self.angleOfTravelRecent
            else:
                angleOfTravel = angleContour
                
                
            # Compare distances between angles of (travel - orientation) and (travel - flippedorientation)        
            dist         = N.abs(CircleFunctions.circle_dist(angleContour,         angleOfTravel))
            dist_flipped = N.abs(CircleFunctions.circle_dist(angleContour_flipped, angleOfTravel))
            
            # Vote for flipped or non-flipped.
            if dist < dist_flipped:
                flipValueNew = -1.0 # Not flipped
            else:
                flipValueNew =  1.0 # Flipped
        else:
            flipValueNew = flipValuePrev

        # Weight the prev/new values based on speed.                
        speed = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y])
        alpha = 10.0/(10.0+speed)
        flipValueWeighted = (alpha * flipValuePrev) + ((1.0-alpha) * flipValueNew) 

        
        #if 'Fly1' in self.name:
        #    rospy.logwarn('%s: flipValue %0.2f, %0.2f, %0.2f' % (self.name, flipValuePrev,flipValueWeighted,flipValueNew))


        return flipValueWeighted
    

        
    # SetFlipState()
    #   Choose among the various possibilities:
    #   flip==True  -->  angleResolved = (angle of contour)
    #     OR
    #   flip==False -->  angleResolved = (angle of contour + pi)
    #
    # Updates the flip status in self.flip
    #   
    def SetFlipState(self):
        # Update the flip filter, and convert it to a flip state.
        flipValuePre = self.GetNextFlipValue()
        flipValuePost = self.lpFlip.Update(flipValuePre, self.contour.header.stamp.to_sec())
        if flipValuePost > 0.0:
            flipNew = True
        else:
            flipNew = False

        speed = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y])
        if speed > 3.0: # Deadband.
            self.flip = flipNew    
                
        # contour angle only ranges on [-pi,-0].  If wrapped, then change the flip state.
        #if 'Fly1' in self.name:
        #    rospy.logwarn('self.flip=%s'%self.flip)
        if (self.contourPrev is not None) and (isinstance(self.contourPrev.angle,float)) and (self.contour is not None) and (isinstance(self.contour.angle,float)):
            #rospy.logwarn('contour.angle=%s, contourPrev.angle=%s'%(self.contour.angle, self.contourPrev.angle))
            d = N.abs(CircleFunctions.circle_dist(self.contour.angle, self.contourPrev.angle))
            if (d > (N.pi/2.0)):
                #if 'Fly1' in self.name:
                #    rospy.logwarn('%s: wrap %0.2f, %0.2f, d=%0.2f'%(self.name, self.contour.angle, self.contourPrev.angle,d))
                self.lpFlip.SetValue(-self.lpFlip.GetValue())
                self.flip = not self.flip

                

    def GetResolvedAngle(self):
        angleF = self.lpAngleF.GetValue()
                
        if self.flip and ('Robot' not in self.name):
            angleResolved = (angleF + N.pi) % (2.0*N.pi)
        else:
            angleResolved = angleF


        return angleResolved
            
                            

    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contour, ptComputed):
        if self.initialized:
            if (contour is not None):
                time = contour.header.stamp.to_sec()
            else:
                time = rospy.Time.now().to_sec()
                
            self.contourPrev = self.contour
            self.contour = contour
            
            # Update the position & orientation filters
            self.isVisible = False            
            if (self.contour is not None):
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
                
                if (self.contour.x is not None) and (self.contour.y is not None) and (self.contour.angle is not None):
                    self.isVisible = True
                    (x,y,vx,vy) = self.kfState.Update((self.contour.x, self.contour.y), time)
                    (z, vz) = (0.0, 0.0)
                    #(x,y) = (self.contour.x,self.contour.y) # Unfiltered.
                    angleF = self.lpAngleF.Update(self.contour.angle, time)#self.contour.header.stamp.to_sec())
                    
                    
                    if N.abs(self.contour.x) > 9999 or N.abs(x)>9999:
                        rospy.logwarn ('FLY LARGE CONTOUR, x,x=%s, %s.  Check the parameter camera/diff_threshold.' % (self.contour.x, x))


                # Use the unfiltered data for the case where the filters return None results.
                if self.isVisible and ((x is None) or (y is None)):
                    rospy.logwarn('FLY Kalman filter returned \'None\' at %s, %s' % ([self.contour.x, self.contour.y], time))
                    x = self.contour.x
                    y = self.contour.y
                    z = 0.0
                    vx = 0.0
                    vy = 0.0
                    vz = 0.0

            else: # self.contour is None
                #rospy.logwarn('FLY No contour seen for %s; check your nFlies parameter.' % self.name)
                (x,y,vx,vy) = self.kfState.Update(None, time)
                (z, vz) = (0.0, 0.0)
                
                
            if self.isVisible:
                # Store the latest state.
                self.state.header.stamp = self.contour.header.stamp
                self.state.pose.position.x = x
                self.state.pose.position.y = y
                self.state.pose.position.z = z
                #self.state.pose.orientation comes from self.ResolveOrientation() later.
                try:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = self.tfrx.lookupTwist(self.name, self.state.header.frame_id, self.state.header.stamp-self.dtVelocity, self.dtVelocity)
                except (tf.Exception, AttributeError), e:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = ((0,0,0),(0,0,0))
                    #rospy.logwarn('lookupTwist() Exception: %s' % e)
                    
                self.state.velocity.linear.x = vx
                self.state.velocity.linear.y = vy
                self.state.velocity.linear.z = vz
                self.state.velocity.angular.x = wx
                self.state.velocity.angular.y = wy
                self.state.velocity.angular.z = wz

                # Update the most recent angle of travel.
                self.SetAngleOfTravel()
                self.SetFlipState()
                self.angle = self.GetResolvedAngle()
                #if (self.contourPrev is not None):
                #    if 'Fly1' in self.name:
                #        rospy.logwarn('%s: angle=%0.2f %0.2f, %0.2f'%(self.name, self.angle, self.contourPrev.angle, self.contour.angle))
                
                (self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w) = \
                    tf.transformations.quaternion_about_axis(self.angle, (0,0,1))
                #rospy.loginfo('angles=[%0.2f, %0.2f]' % (angleResolved, self.angle))

                #rospy.loginfo ('CI %s ecc=%s,%s, area=%s,%s, speed=%0.3f' % (self.name, contour.ecc,[self.eccMin,self.eccMax], contour.area,[self.areaMin,self.areaMax], speed))
                
                # Update the tool offset.
                if ptComputed is not None:
#                    marker = Marker(header=Header(stamp=rospy.Time.now(),
#                                                        frame_id='Plate'),
#                                          ns='kalman',
#                                          id=4,
#                                          type=2, #SPHERE,
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
#                                                        frame_id='Plate'),
#                                          ns='computed',
#                                          id=5,
#                                          type=2, #SPHERE,
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

                    # PID control of the offset.
                    ptP = Point()
                    ptI = Point()
                    ptD = Point()
                    ptP.x = x-ptComputed.x
                    ptP.y = y-ptComputed.y
                    ptI.x = ptI.x + ptP.x
                    ptI.y = ptI.y + ptP.y
                    ptD.x = ptP.x - self.ptPPrev.x
                    ptD.y = ptP.y - self.ptPPrev.y
                    kP = rospy.get_param('tooloffset/kP', 1.0)
                    kI = rospy.get_param('tooloffset/kI', 0.0)
                    kD = rospy.get_param('tooloffset/kD', 0.0)
                    xPID = kP*ptP.x + kI*ptI.x + kD*ptD.x
                    yPID = kP*ptP.y + kI*ptI.y + kD*ptD.y
                    
                    # Filter the offset as magnitude & angle.
                    (mag,ang)=self.PolarFromXy(xPID,yPID)
                    #rospy.logwarn('ang=%0.2f' % ang)
                    ang += self.unwind
                    if ang-self.angPrev > N.pi:
                        self.unwind -= 2.0*N.pi
                        ang -= 2.0*N.pi
                    elif ang-self.angPrev < -N.pi:
                        self.unwind += 2.0*N.pi
                        ang += 2.0*N.pi
                    self.angPrev = ang
                    
                    ang_filtered = self.lpOffsetAng.Update(ang, time)
                    #rospy.logwarn('ang=%0.2f, ang_filtered=%0.2f' % (ang,ang_filtered))
                    (self.ptOffset.x,self.ptOffset.y) = self.XyFromPolar(N.clip(self.lpOffsetMag.Update(mag, time), -self.maxOffset, self.maxOffset),
                                                                         ang_filtered)
                    self.ptPPrev.x = ptP.x
                    self.ptPPrev.y = ptP.y
                else:
                    self.ptOffset = Point(x=0, y=0, z=0)
            

                
                # Send the Raw transform.
                self.tfbx.sendTransform((self.contour.x, 
                                         self.contour.y, 
                                         0.0),
                                        tf.transformations.quaternion_about_axis(self.contour.angle, (0,0,1)),
                                        self.contour.header.stamp,
                                        self.name+"Contour",
                                        "Plate")

                
                # Send the Filtered transform.
                q = self.state.pose.orientation
                self.tfbx.sendTransform((self.state.pose.position.x, 
                                         self.state.pose.position.y, 
                                         self.state.pose.position.z),
                                        (q.x, q.y, q.z, q.w),
                                        self.state.header.stamp,
                                        self.name,
                                        self.state.header.frame_id)
                #rospy.logwarn('sendTransform(frame=%s, stamp=%s, now-stamp=%s)' % (self.name, self.state.header.stamp, rospy.Time.now()-self.state.header.stamp))
                #rospy.logwarn ('now,transform,%s,%s' % (rospy.Time.now(), self.state.header.stamp))

                # Publish a 3D model marker for the robot.
                if 'Robot' in self.name:
                    markerRobot = Marker(header=Header(stamp = self.state.header.stamp,
                                                        frame_id='Plate'),
                                          ns='robot',
                                          id=1,
                                          type=3, #cylinder,
                                          action=0,
                                          pose=Pose(position=Point(x=self.state.pose.position.x, 
                                                                   y=self.state.pose.position.y, 
                                                                   z=self.state.pose.position.z)),
                                          scale=Vector3(x=self.robot_width,
                                                        y=self.robot_length,
                                                        z=self.robot_height),
                                          color=ColorRGBA(a=0.7,
                                                          r=0.5,
                                                          g=0.5,
                                                          b=0.5),
                                          lifetime=rospy.Duration(0.5))
                    self.pubMarker.publish(markerRobot)

            
        

