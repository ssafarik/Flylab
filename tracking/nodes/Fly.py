#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import copy
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
import cv
import cv2
import numpy as N
import threading
from tracking.msg import *
from arena_tf.srv import *
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Header, ColorRGBA, Float32
from tracking.msg import Contourinfo
from visualization_msgs.msg import Marker
from flycore.msg import MsgFrameState
from pythonmodules import filters, CircleFunctions


globalNonessential = False   # Publish nonessential stuff?
globalLock = threading.Lock()

        
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

        self.cvbridge = CvBridge()
        self.tfrx = tfrx
        self.tfbx = tf.TransformBroadcaster()
        self.pubMarker       = rospy.Publisher('visualization_marker', Marker)
        
        # Nonessential stuff to publish.
        if globalNonessential:
            self.pubImageRoiMean  = rospy.Publisher(self.name+"/image_mean", Image)
            self.pubImageRoi      = rospy.Publisher(self.name+"/image", Image)
            self.pubImageRoiReg   = rospy.Publisher(self.name+"/image_reg", Image)
            self.pubImageRoiWings = rospy.Publisher(self.name+"/image_wings", Image)
            self.pubImageMask     = rospy.Publisher(self.name+"/image_mask", Image)
            self.pubImageMaskBody = rospy.Publisher(self.name+"/image_mask_body", Image)
            self.pubLeftMetric    = rospy.Publisher(self.name+'/leftmetric', Float32)
            self.pubLeft          = rospy.Publisher(self.name+'/left', Float32)
            self.pubRightMetric   = rospy.Publisher(self.name+'/rightmetric', Float32)
            self.pubRight         = rospy.Publisher(self.name+'/right', Float32)
        
        global globalLock
        with globalLock:
            rcFilterAngle           = rospy.get_param('tracking/rcFilterAngle', 0.1)
            dtVelocity              = rospy.get_param('tracking/dtVelocity', 0.2)
            dtForecast              = rospy.get_param('tracking/dtForecast',0.15)
            rcFilterFlip            = rospy.get_param('tracking/rcFilterFlip', 3.0)
            rcFilterSpeed           = rospy.get_param('tracking/rcFilterSpeed', 0.2)
            speedThresholdForTravel = rospy.get_param ('tracking/speedThresholdForTravel', 5.0)
            rcFilterAngularVel      = rospy.get_param('tracking/rcFilterAngularVel', 0.05)
            self.widthRoi           = rospy.get_param ('tracking/roi/width', 15)
            self.heightRoi          = rospy.get_param ('tracking/roi/height', 15)
            self.lengthBody         = rospy.get_param ('tracking/lengthBody', 9)
            self.widthBody          = rospy.get_param ('tracking/widthBody', 5)
            self.robot_width        = rospy.get_param ('robot/width', 1.0)
            self.robot_length       = rospy.get_param ('robot/length', 1.0)
            self.robot_height       = rospy.get_param ('robot/height', 1.0)
            self.alphaForeground    = rospy.get_param('tracking/alphaForeground', 0.01)
            self.thresholdForeground= rospy.get_param('tracking/thresholdForeground', 25.0)

        self.kfState = filters.KalmanFilter()
        self.lpAngleContour = filters.LowPassHalfCircleFilter(RC=rcFilterAngle)
        self.lpAngleContour.SetValue(0.0)
        self.apAngleContour = filters.LowPassHalfCircleFilter(RC=0.0)
        self.apAngleContour.SetValue(0.0)
        self.lpOffsetMag = filters.LowPassFilter(RC=1.0)
        self.lpOffsetMag.SetValue(0.0)
        self.lpOffsetAng = filters.LowPassCircleFilter(RC=0.001)
        self.lpOffsetAng.SetValue(0.0)
        self.maxOffset = 10.0
        self.ptOffset = Point(x=0, y=0, z=0)  # Vector from computed position to contour position (for robots only).
        self.angleOffsetPrev = 0.0
        self.stampPrev = rospy.Time.now()
        self.unwind = 0.0
        self.dtVelocity = rospy.Duration(dtVelocity) # Interval over which to calculate velocity.
        self.dtForecast = dtForecast
        
        # Orientation detection stuff.
        self.lpFlip = filters.LowPassFilter(RC=rcFilterFlip)
        self.lpFlip.SetValue(0.0) # (0.5) TEST
        self.lpSpeed = filters.LowPassFilter(RC=rcFilterSpeed)
        self.lpSpeed.SetValue(0.0)
        self.angleOfTravelRecent = 0.0
        self.contourinfo = Contourinfo()
        self.speedThresholdForTravel = speedThresholdForTravel # Speed that counts as "traveling".
        self.speed = 0.0
        self.angleContourUnwrapped = 0.0

        
        self.lpWx = filters.LowPassFilter(RC=rcFilterAngularVel)
        self.lpWy = filters.LowPassFilter(RC=rcFilterAngularVel)
        self.lpWz = filters.LowPassFilter(RC=rcFilterAngularVel)
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
        self.state.wings.left.angle   = 0.0
        self.state.wings.right.angle  = 0.0

        self.eccMin = 999.9
        self.eccMax = 0.0
        self.eccSum = 1.0
        self.eccCount = 1
        
        self.areaMin = 999.9
        self.areaMax = 0.0
        self.areaSum = 0.0
        self.areaCount = 1
        
        # Wing angle stuff.
        self.npfRoiMean = None
        self.npMaskWings = None
        self.npMaskCircle = N.zeros([self.heightRoi, self.widthRoi],dtype=N.uint8)
        cv2.circle(self.npMaskCircle,
                  (int(self.heightRoi/2), int(self.widthRoi/2)),
                  int(self.heightRoi/2), 
                  255, 
                  cv.CV_FILLED)
        self.metricMeanLeft  = None
        self.metricMeanRight = None
        
        
        
        # Super-resolution image.
        self.heightSuper = 100*self.heightRoi
        self.widthSuper = 100*self.widthRoi
        self.npfSuper = N.zeros([self.heightSuper, self.widthSuper]) 
        
        
        
        rospy.logwarn('FLY object added, name=%s' % name)
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


    # GetNextFlipValue()
    #   Using self.contourinfo, self.state, and self.angleOfTravelRecent,
    #   we determine a value to use for updating the "flip" filter,
    #   which varies on [-1,+1], and the sign of which determines if to flip.
    #
    # Returns the chosen flip value.
    #   
    def GetNextFlipValue(self):
        # Get the prior flip value.
        flipvalueMagPrev = self.lpFlip.GetValue()
        if flipvalueMagPrev is None:
            flipvalueMagPrev = 0.0


        angleContour         = self.lpAngleContour.GetValue()
        
        # Compare distances between angles of (travel - contour) and (travel - flippedcontour)        
        dist         = N.abs(CircleFunctions.DistanceCircle(angleContour,        self.angleOfTravelRecent))
        dist_flipped = N.abs(CircleFunctions.DistanceCircle(angleContour + N.pi, self.angleOfTravelRecent))
        
        # Choose the better orientation.
        if dist < dist_flipped:
            flipvalueSign =  1.0 # Not flipped
        else:
            flipvalueSign = -1.0 # Flipped


        if self.contourinfo.ecc is not None:
            eccmetric = (self.contourinfo.ecc + 1/self.contourinfo.ecc) - 1 #self.contourinfo.ecc#
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

        
        flipvalue = flipvalueSign * flipvalueMag
        
        #if 'Fly1' in self.name:
        #    rospy.logwarn('%s: flipvalue %0.2f, %0.2f, %0.2f' % (self.name, flipvalueMagPrev,flipvalueWeighted,flipvalueNew))


        return flipvalue
    

        
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
        if (self.speed > self.speedThresholdForTravel):
            flipvaluePre = self.GetNextFlipValue()
            flipvaluePost = self.lpFlip.Update(flipvaluePre, self.contourinfo.header.stamp.to_sec())
                
#            # Contour angle only ranges on [-pi,-0].  If it wraps, then change the lpFlip sign.
#            d = N.abs(CircleFunctions.DistanceCircle(self.lpAngleContour.GetValue(), self.lpAngleContour.GetValuePrev()))
#            if (d > (N.pi/2.0)):
#                self.lpFlip.SetValue(-self.lpFlip.GetValue())
#                rospy.logwarn('WRAP*****************************************')

            

    def ResolveAngle(self, angle):
        if (angle is not None):
            if self.lpFlip.GetValue()<0 and ('Robot' not in self.name):
                angleResolved = (angle + N.pi) % (2.0*N.pi)
            else:
                angleResolved = angle
#                if 'Fly2' in self.name:
#                    rospy.logwarn('=======================')
        else:
            angleResolved = None

        return angleResolved
            
            
    def ResolveRawAngle(self):
        angle = self.apAngleContour.GetValue()
        angleResolved = self.ResolveAngle(angle)

        return angleResolved
            
            
    def ResolveFilteredAngle(self):
        angle = self.lpAngleContour.GetValue()
        angleResolved = self.ResolveAngle(angle)

        return angleResolved
            
            
    # UpdateWingMask()
    # Create a mask image that only passes the fly body and both wings in any position.
    # FUTURE: should make the body/wing dimensions automatically calculated from the fly mean image, npfRoiMean.
    #
    def UpdateWingMask(self, npfRoiMean):
        #global globalLock
        
        self.npMaskWings = N.zeros([self.heightRoi, self.widthRoi], dtype=N.uint8)


        # Coordinates of the body ellipse.
        centerBody = (self.widthRoi/2, 
                      self.heightRoi/2)
        sizeBody = (self.lengthBody, 
                    self.widthBody)
        angleBody = 0.0
        
        # Left wing ellipse.
        sizeLeft = (self.lengthBody*10/10, 
                    self.lengthBody*7/10)
        centerLeft = (centerBody[0] - sizeLeft[0]/2 + self.lengthBody*1/10,
                      centerBody[1] - sizeLeft[1]/2 - 1)
        angleLeft = 0.0

        # Right wing ellipse.
        sizeRight = (self.lengthBody*10/10, 
                     self.lengthBody*7/10)
        centerRight = (centerBody[0] - sizeRight[0]/2 + self.lengthBody*1/10,
                       centerBody[1] + sizeRight[1]/2)
        angleRight = 0.0


        # Draw ellipses on the mask.
        #cv2.ellipse(self.npMaskWings,
        #            (centerBody, sizeBody, angleBody),
        #            255, cv.CV_FILLED)
        cv2.ellipse(self.npMaskWings,
                    (centerLeft, sizeLeft, angleLeft*180.0/N.pi),
                    255, cv.CV_FILLED)
        cv2.ellipse(self.npMaskWings,
                    (centerRight, sizeRight, angleRight*180.0/N.pi),
                    255, cv.CV_FILLED)
            
        if (npfRoiMean is not None):
            # Mean Fly Body Mask.
            #with globalLock:
            #    self.thresholdForeground = rospy.get_param('tracking/thresholdForeground', 25.0)
            (threshOut, npMaskBody) = cv2.threshold(npfRoiMean.astype(N.uint8), self.thresholdForeground, 255, cv2.THRESH_BINARY_INV)
            self.npMaskWings = cv2.bitwise_and(self.npMaskWings, npMaskBody)
        
    
            # Publish non-essential stuff.
            if globalNonessential:
                npMaskWings1 = copy.copy(self.npMaskWings)
                npMaskWings1.resize(npMaskWings1.size)
                imgMaskWings  = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npMaskWings), 'passthrough')
                imgMaskWings.data = list(npMaskWings1)
                self.pubImageMask.publish(imgMaskWings)
        
                npMaskBody1 = copy.copy(npMaskBody)
                npMaskBody1.resize(npMaskBody1.size)
                imgMaskBody  = self.cvbridge.cv_to_imgmsg(cv.fromarray(npMaskBody), 'passthrough')
                imgMaskBody.data = list(npMaskBody1)
                self.pubImageMaskBody.publish(imgMaskBody)
    
        
    def UpdateFlyMean(self, npRoiReg):
        #global globalLock
        #with globalLock:
        #    self.alphaForeground = rospy.get_param('tracking/alphaForeground', 0.01)

        if (self.npfRoiMean is None):
            self.npfRoiMean = N.float32(npRoiReg)
        cv2.accumulateWeighted(N.float32(npRoiReg), self.npfRoiMean, self.alphaForeground)

        
    def UpdateFlySuperresolution(self, npRoi, moments):
        pass
    
        
    # Rotate the image to 0 degrees.                    
    def RegisterImageRoi(self, npRoi):
        if (self.angleContourUnwrapped is not None):
            # Center of mass.
            moments = cv2.moments(npRoi)
            xCOM  = moments['m10']/moments['m00']
            yCOM  = moments['m01']/moments['m00']
            
            # Rotate the fly image to 0-degrees.
            angleR = self.ResolveRawAngle()
            T = cv2.getRotationMatrix2D((xCOM,yCOM), -angleR*180.0/N.pi, 1.0)
            npRoiReg = cv2.warpAffine(npRoi, T, (0,0))#(self.widthRoi, self.heightRoi))
            
            # Super-resolution fly image.
            self.UpdateFlySuperresolution(npRoi, moments)

        else:
            npRoiReg = npRoi
            
        return npRoiReg
        

    # GetWingAngles()
    # Compute the left & right wing angles from the given contourinfo angle and image.
    #
    def GetWingAngles(self, contourinfo):
        global globalLock

        npRoiIn = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(contourinfo.imgRoi, "passthrough")))
        npRoi = cv2.bitwise_and(npRoiIn, self.npMaskCircle)
        npRoiReg = self.RegisterImageRoi(npRoi)

        # Update the wing mask.
        self.UpdateWingMask(self.npfRoiMean)

        # Moving-Average fly image.
        self.UpdateFlyMean(npRoiReg)

        # Mask the input ROI.
        npRoiReg = cv2.bitwise_and(npRoiReg, self.npMaskWings)
        
        
        # Background Fly Subtraction.
        diff = npRoiReg.astype(N.float32) - self.npfRoiMean*1.0 # Only partially subtract the mean.
        npWings = N.clip(diff, 0, N.iinfo(N.uint8).max).astype(N.uint8)
        #npWings = npRoiReg 
        
        # Mask the wings.
#        npWings = cv2.bitwise_and(npWings, self.npMaskWings)
                        
        # Create images with only pixels of left wing (i.e. image top) or right wing (i.e. image bottom).
        npWingLeft   = N.vstack((npWings[:self.heightRoi/2, :],
                                 N.zeros([self.heightRoi/2+1, self.widthRoi])))
        npWingRight  = N.vstack((N.zeros([self.heightRoi/2+1, self.widthRoi]), 
                                 npWings[(self.heightRoi/2+1):, :]))
        
        # Wing moments.
        momentsRoi = cv2.moments(npRoiReg)
        momentsLeft  = cv2.moments(npWingLeft)
        momentsRight = cv2.moments(npWingRight)

        # Pixel locations of body & wings.
        try:
            xBody  = momentsRoi['m10']/momentsRoi['m00']
            yBody  = momentsRoi['m01']/momentsRoi['m00']
        except ZeroDivisionError:
            xBody = 0.0
            yBody = 0.0
            
        try:
            xLeft  = momentsLeft['m10']/momentsLeft['m00'] - (xBody + self.lengthBody/4)
            yLeft  = momentsLeft['m01']/momentsLeft['m00'] - yBody
        except ZeroDivisionError:
            xLeft = 0.0
            yLeft = 0.0
            
        try:
            xRight = momentsRight['m10']/momentsRight['m00'] - (xBody + self.lengthBody/4)
            yRight = momentsRight['m01']/momentsRight['m00'] - yBody + 1
        except ZeroDivisionError:
            xRight = 0.0
            yRight = 0.0

        xBodyBg = xBody
        yBodyBg = yBody


        # Metrics to distinguish wing extension from noise.
        metricLeft = momentsLeft['m00']
        metricRight = momentsRight['m00']

        # Metric mean, for thresholding.
        if (self.metricMeanLeft is not None):
            a = 0.01
            self.metricMeanLeft  = (1-a)*self.metricMeanLeft  + a*metricLeft 
            self.metricMeanRight = (1-a)*self.metricMeanRight + a*metricRight 
        else:
            self.metricMeanLeft  = metricLeft 
            self.metricMeanRight = metricRight 
            
        
        
        if (metricLeft < 3*self.metricMeanLeft):
            angleLeft = N.pi
        else:
            angleLeft  = N.arctan2(yLeft, xLeft)+N.pi
            
        if (metricRight < 3*self.metricMeanRight):
            angleRight = N.pi
        else:
            angleRight = N.arctan2(yRight, xRight)+N.pi
            
            if globalNonessential:
                npToUse = npWingRight
                if (momentsLeft['m00'] != 0):
                    npRoiReg[momentsLeft['m01']/momentsLeft['m00'],   
                             momentsLeft['m10']/momentsLeft['m00']] = 255.0      # Set pixel at wing center.
                    npToUse[momentsLeft['m01']/momentsLeft['m00'],   
                            momentsLeft['m10']/momentsLeft['m00']] = 255.0      # Set pixel at wing center.
                if (momentsRight['m00'] != 0):
                    npRoiReg[momentsRight['m01']/momentsRight['m00']+1, 
                             momentsRight['m10']/momentsRight['m00']] = 255.0
                    npToUse[momentsRight['m01']/momentsRight['m00']+1, 
                            momentsRight['m10']/momentsRight['m00']] = 255.0
            


        # Nonessential stuff to publish.
        if globalNonessential:
            self.imgRoiReg  = self.cvbridge.cv_to_imgmsg(cv.fromarray(npRoiReg), 'passthrough') 
            self.imgRoiMean = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.uint8(self.npfRoiMean)), 'passthrough')

#            npRoiReg[momentsLeft['m01']/momentsLeft['m00'],   
#                     momentsLeft['m10']/momentsLeft['m00']] = 255.0      # Set pixel at wing center.
#            npRoiReg[momentsRight['m01']/momentsRight['m00']+1, 
#                     momentsRight['m10']/momentsRight['m00']] = 255.0
            
            npToUse = npWingRight
#            npToUse[momentsLeft['m01']/momentsLeft['m00'],   
#                    momentsLeft['m10']/momentsLeft['m00']] = 255.0      # Set pixel at wing center.
#            npToUse[momentsRight['m01']/momentsRight['m00']+1, 
#                    momentsRight['m10']/momentsRight['m00']] = 255.0
            imgWings = copy.copy(self.imgRoiReg)
            npWings1 = npToUse.astype(N.uint8).reshape(npToUse.size)
            imgWings.data = list(npWings1)
            self.pubImageRoi.publish(contourinfo.imgRoi)
            self.pubImageRoiReg.publish(self.imgRoiReg)
            self.pubImageRoiMean.publish(self.imgRoiMean)
            self.pubImageRoiWings.publish(imgWings)
            self.pubLeftMetric.publish(metricLeft)
            self.pubRightMetric.publish(metricRight)
            self.pubLeft.publish(angleLeft)
            self.pubRight.publish(angleRight)

        return (angleLeft, angleRight)
    
    
    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contourinfo, posesComputedExternal):
        if self.initialized:
            self.contourinfo = contourinfo
            
            # Use the computed end-effector orientation.
            if posesComputedExternal is not None:
                posesComputed = self.tfrx.transformPose('Arena', posesComputedExternal)
                q = posesComputed.pose.orientation
                rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                angle = rpy[2]
            else:
                angle = contourinfo.angle
                
            

            # Update the position & orientation filters
            self.isVisible = False            
            if (self.contourinfo.x is not None) and \
               (self.contourinfo.y is not None) and \
               (self.contourinfo.angle is not None) and \
               (not N.isnan(self.contourinfo.angle)) and \
               (self.contourinfo.area is not None) and \
               (self.contourinfo.ecc is not None):
                
                # Update min/max ecc & area.
                if contourinfo.ecc < self.eccMin:
                    self.eccMin = contourinfo.ecc
                if self.eccMax < contourinfo.ecc:
                    self.eccMax = contourinfo.ecc
                if contourinfo.area < self.areaMin:
                    self.areaMin = contourinfo.area
                if self.areaMax < contourinfo.area:
                    self.areaMax = contourinfo.area
                self.eccSum += contourinfo.ecc
                self.eccCount += 1
                self.areaSum += contourinfo.area
                self.areaCount += 1
                
                self.isVisible = True
                (xKalman,yKalman,vxKalman,vyKalman) = self.kfState.Update((self.contourinfo.x, self.contourinfo.y), contourinfo.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                #(xKalman,yKalman) = (self.contourinfo.x,self.contourinfo.y) # Unfiltered.
                self.lpAngleContour.Update(angle, contourinfo.header.stamp.to_sec())
                self.apAngleContour.Update(angle, contourinfo.header.stamp.to_sec())

                (angleLeft, angleRight) = self.GetWingAngles(contourinfo)
                
                
                if N.abs(self.contourinfo.x)>9999 or N.abs(xKalman)>9999:
                    rospy.logwarn ('FLY LARGE CONTOUR, x,x=%s, %s.  Check your background image, lighting, and the parameter tracking/diff_threshold.' % (self.contourinfo.x, xKalman))


            else: # We don't have a contourinfo.
                #rospy.logwarn('FLY No contour seen for %s; check your nFlies parameter.' % self.name)
                (xKalman, yKalman, vxKalman, vyKalman) = self.kfState.Update(None, contourinfo.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                (angleLeft, angleRight) = (N.pi, N.pi)

                
            # If good data, then use it.
            if (xKalman is not None) and (yKalman is not None):
                x = xKalman
                y = yKalman
                z = zKalman
                vx = vxKalman
                vy = vyKalman
                vz = vzKalman
            else: # Use the unfiltered data for the case where the filters return None results.
                rospy.logwarn('FLY Object %s not yet initialized at %s, %s' % (self.name, contourinfo.header.stamp.to_sec(), [self.contourinfo.x, self.contourinfo.y]))
                x = 0.0#self.contourinfo.x
                y = 0.0#self.contourinfo.y
                z = 0.0
                vx = 0.0
                vy = 0.0
                vz = 0.0

                
            # Store the latest state.
            self.state.header.stamp = self.contourinfo.header.stamp
            self.state.pose.position.x = x
            self.state.pose.position.y = y
            self.state.pose.position.z = z
            #self.state.pose.orientation comes from self.ResolveFilteredAngle() later.
            self.state.velocity.linear.x = vx
            self.state.velocity.linear.y = vy
            self.state.velocity.linear.z = vz
            self.state.wings.left.angle = angleLeft
            self.state.wings.right.angle = angleRight


            # Get the angular velocities.
            if self.isVisible:
                try:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = self.tfrx.lookupTwist(self.name, self.state.header.frame_id, self.state.header.stamp-self.dtVelocity, self.dtVelocity)
                except (tf.Exception, AttributeError), e:
                    ((vx2,vy2,vz2),(wx,wy,wz)) = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            else:
                ((vx2,vy2,vz2),(wx,wy,wz)) = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

            # Fix glitches due to flip orientation.
            if wz>6.28:
                wz = self.lpWz.GetValue()
                
            self.state.velocity.angular.x = self.lpWx.Update(wx, self.state.header.stamp.to_sec())
            self.state.velocity.angular.y = self.lpWy.Update(wy, self.state.header.stamp.to_sec())
            self.state.velocity.angular.z = self.lpWz.Update(wz, self.state.header.stamp.to_sec())
                
            speedPre = N.linalg.norm([self.state.velocity.linear.x, self.state.velocity.linear.y, self.state.velocity.linear.z])
            self.speed = self.lpSpeed.Update(speedPre, self.state.header.stamp.to_sec())
                

            # Update the most recent angle of travel.
            self.SetAngleOfTravel()
            self.UpdateFlipState()
            if 'Robot' not in self.name:
                angle = self.ResolveFilteredAngle()
            else:
                pass # angle already set from above.
            
            (self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w) = tf.transformations.quaternion_about_axis(angle, (0,0,1))
                

                
            # Send the Raw transform.
            if self.isVisible and (not N.isnan(self.contourinfo.angle)):
                self.tfbx.sendTransform((self.contourinfo.x, 
                                         self.contourinfo.y, 
                                         0.0),
                                        tf.transformations.quaternion_about_axis(self.contourinfo.angle, (0,0,1)),
                                        self.contourinfo.header.stamp,
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
                #with globalLock:
                #    self.dtForecast = rospy.get_param('tracking/dtForecast',0.25)
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

        
    

