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


globalNonessentialWings = False   # Publish nonessential stuff?
globalNonessentialSuperRes = False   # Publish nonessential stuff?
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
        if globalNonessentialWings:
            self.pubImageRoiMean    = rospy.Publisher(self.name+"/image_mean", Image)
            self.pubImageRoi        = rospy.Publisher(self.name+"/image", Image)
            self.pubImageRoiReg     = rospy.Publisher(self.name+"/image_reg", Image)
            self.pubImageRoiWings   = rospy.Publisher(self.name+"/image_wings", Image)
            self.pubImageMask       = rospy.Publisher(self.name+"/image_mask", Image)
            self.pubImageMaskBody   = rospy.Publisher(self.name+"/image_mask_body", Image)
            self.pubImageSuper      = rospy.Publisher(self.name+'/image_super', Image)
            self.pubLeftMetric      = rospy.Publisher(self.name+'/leftmetric', Float32)
            self.pubLeftMetricMean  = rospy.Publisher(self.name+'/leftmetricmean', Float32)
            self.pubLeft            = rospy.Publisher(self.name+'/left', Float32)
            self.pubRightMetric     = rospy.Publisher(self.name+'/rightmetric', Float32)
            self.pubRight           = rospy.Publisher(self.name+'/right', Float32)
        if globalNonessentialSuperRes:
            self.pubImageSuper      = rospy.Publisher(self.name+'/image_super', Image)
        
        global globalLock
        with globalLock:
            rcFilterAngle                = rospy.get_param('tracking/rcFilterAngle', 0.1)
            dtVelocity                   = rospy.get_param('tracking/dtVelocity', 0.2)
            self.dtForecast              = rospy.get_param('tracking/dtForecast',0.15)
            rcFilterFlip                 = rospy.get_param('tracking/rcFilterFlip', 3.0)
            rcFilterSpeed                = rospy.get_param('tracking/rcFilterSpeed', 0.2)
            self.speedThresholdForTravel = rospy.get_param('tracking/speedThresholdForTravel', 5.0)
            rcFilterAngularVel           = rospy.get_param('tracking/rcFilterAngularVel', 0.05)
            self.widthRoi                = rospy.get_param('tracking/roi/width', 15)
            self.heightRoi               = rospy.get_param('tracking/roi/height', 15)
            self.lengthBody              = rospy.get_param('tracking/lengthBody', 9)
            self.widthBody               = rospy.get_param('tracking/widthBody', 5)
            self.robot_width             = rospy.get_param('robot/width', 1.0)
            self.robot_length            = rospy.get_param('robot/length', 1.0)
            self.robot_height            = rospy.get_param('robot/height', 1.0)
            self.rcForeground            = rospy.get_param('tracking/rcForeground', 1.0)
            self.thresholdForeground     = rospy.get_param('tracking/thresholdForeground', 25.0)

        self.kfState = filters.KalmanFilter()
        self.lpAngle = filters.LowPassCircleFilter(RC=rcFilterAngle)
        self.lpAngle.SetValue(0.0)
        self.lpAngleContour = filters.LowPassHalfCircleFilter(RC=rcFilterAngle)
        self.lpAngleContour.SetValue(0.0)
        self.apAngleContour = filters.LowPassHalfCircleFilter(RC=0.0)
        self.apAngleContour.SetValue(0.0)
        self.stampPrev = rospy.Time.now()
        self.dtVelocity = rospy.Duration(dtVelocity) # Interval over which to calculate velocity.
        self.stampPrev = rospy.Time.now()
        
        # Orientation detection stuff.
        self.lpFlip = filters.LowPassFilter(RC=rcFilterFlip)
        self.lpFlip.SetValue(0.0) # (0.5) TEST
        self.lpSpeed = filters.LowPassFilter(RC=rcFilterSpeed)
        self.lpSpeed.SetValue(0.0)
        self.angleOfTravelRecent = 0.0
        self.contourinfo = Contourinfo()
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
        self.state.pose.orientation.y = q[1]    #       Check the self.lpFlip sign to resolve it:  use self.ResolvedAngle().
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
        self.eccMean = None
        self.eccMax = 0.0
        
        self.areaMin = 999.9
        self.areaMean = None
        self.areaMax = 0.0
        
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
        flipvalue = self.lpFlip.GetValue()
        if (flipvalue is not None):
            magFlipvalue = N.abs(flipvalue)
            signFlipvalue = N.sign(flipvalue)
        else:
            magFlipvalue = 0.0
            signFlipvalue = 1.0

        if signFlipvalue==0.0:
            signFlipvalue = 1.0


        angle = self.GetResolvedAngleFiltered()
        
        # Compare distances between angles of (travel - prevorientation) and (travel - flippedprevorientation)        
        dist_current = N.abs(CircleFunctions.DistanceCircle(angle,        self.angleOfTravelRecent))
        dist_flipped = N.abs(CircleFunctions.DistanceCircle(angle + N.pi, self.angleOfTravelRecent))

        

        if self.contourinfo.ecc is not None:
            eccmetric = (self.contourinfo.ecc + 1/self.contourinfo.ecc) - 1 #self.contourinfo.ecc#
        else:
            eccmetric = 1.0
        #rospy.logwarn('%s: eccmetric=%0.2f' % (self.name, eccmetric))

        if (self.speed > self.speedThresholdForTravel):
            if (eccmetric > 1.5):
                magFlipvalueNew = 1.0

                # Choose the better orientation.
                if (dist_flipped < dist_current):
                    signFlipvalue = -signFlipvalue # Flipped

            else:
                magFlipvalueNew = magFlipvalue 

        else:
            magFlipvalueNew = magFlipvalue 

        
        flipvalueNew = signFlipvalue * magFlipvalueNew
        
        #if 'Fly1' in self.name:
        #    rospy.logwarn('flipvalue Old,New:  % 3.2f, % 3.2f' % (flipvalue, flipvalueNew))
        #    rospy.logwarn('angle,travel: % 3.2f, % 3.2f   dist,flipped: % 3.2f, % 3.2f' % (angle, self.angleOfTravelRecent, dist_current, dist_flipped))


        return flipvalueNew
    

        
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
        flipvaluePre = self.lpFlip.GetValue()
        
        if (self.speed > self.speedThresholdForTravel):
            flipvalueNew = self.GetNextFlipValue()
        else:
            flipvalueNew = self.lpFlip.GetValue()

        flipvaluePost = self.lpFlip.Update(flipvalueNew, self.contourinfo.header.stamp.to_sec())

            
                
    def ResolveAngle(self, angle):
        if (angle is not None):
            if self.lpFlip.GetValue()<0 and ('Robot' not in self.name):
                angleResolved = (angle + N.pi) % (2.0*N.pi)
            else:
                angleResolved = copy.copy(angle)
        else:
            angleResolved = None

        return angleResolved
            
            
    def GetResolvedAngleRaw(self):
        angle = self.apAngleContour.GetValue()
        angleResolved = self.ResolveAngle(angle)

        return angleResolved
            
            
    def GetResolvedAngleFiltered(self):
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
            if globalNonessentialWings:
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
        #    self.rcForeground = rospy.get_param('tracking/rcForeground', 0.01)
        alphaForeground = 1 - N.exp(-self.dt.to_sec() / self.rcForeground)

        if (self.npfRoiMean is None):
            self.npfRoiMean = N.float32(npRoiReg)
        cv2.accumulateWeighted(N.float32(npRoiReg), self.npfRoiMean, alphaForeground)

        
    def UpdateFlySuperresolution(self, npRoi, moments):
        if globalNonessentialSuperRes:
            if (moments['m00'] != 0.0):
                xCOM  = moments['m10']/moments['m00']
                yCOM  = moments['m01']/moments['m00']
                
#                 rospy.logwarn('-------------------------------------------------------')
                # Rotate the fly image to 0-degrees, and the size of the super image.
                angleR = self.GetResolvedAngleRaw()
                scale = self.widthSuper/npRoi.shape[1]
                xCenter = self.widthSuper/2+xCOM
                yCenter = self.heightSuper/2+yCOM
                #T = cv2.getRotationMatrix2D((xCOM,yCOM), -angleR*180.0/N.pi, scale)
                #T = N.array([[scale*N.cos(angleR), -scale*N.sin(angleR), xCenter], [scale*N.sin(angleR), scale*N.cos(angleR), yCenter]])
                T = cv2.getRotationMatrix2D((xCOM,yCOM), 0, self.widthSuper/npRoi.shape[1])
                L = T.tolist()
                L.append([0.0, 0.0, 1.0])
                T2 = N.array(L)

#                 self.npfSuper = cv2.warpAffine(npRoi, T, (self.widthSuper,self.heightSuper))
                for x in range(npRoi.shape[1]):
                    for y in range(npRoi.shape[0]):
                        (x2,y2,z2) = N.dot(T2, N.array([x,y,1]))
#                        (x2,y2) = N.dot(T, N.array([x,y,1]))
                        x2 += xCenter
                        y2 += yCenter
                        if (0<=x2<self.widthSuper) and (0<=y2<self.heightSuper): 
                            self.npfSuper[int(x2),int(y2)] = npRoi[x,y]
                            #rospy.logwarn('(xCOM,yCOM)=(%d,%d), angle=%3.2f, (%d,%d) -> (%d,%d): %d' % (xCOM,yCOM,angleR,x,y,int(x2),int(y2), npRoi[x,y]))
    
                imgSuper  = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.uint8(self.npfSuper)), 'passthrough')
                self.pubImageSuper.publish(imgSuper)

        
        
    # Rotate the image to 0 degrees.                    
    def RegisterImageRoi(self, npRoi):
        npRoiReg = npRoi
        if (self.angleContourUnwrapped is not None):
            # Center of mass.
            moments = cv2.moments(npRoi)
            if (moments['m00'] != 0.0):
                xCOM  = moments['m10']/moments['m00']
                yCOM  = moments['m01']/moments['m00']
                
                # Rotate the fly image to 0-degrees.
                angleR = self.GetResolvedAngleRaw()
                T = cv2.getRotationMatrix2D((xCOM,yCOM), -angleR*180.0/N.pi, 1.0)
                npRoiReg = cv2.warpAffine(npRoi, T, (0,0))#(self.widthRoi, self.heightRoi))
                
                # Super-resolution fly image.
                self.UpdateFlySuperresolution(npRoiReg, moments)
            
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
            a = 0.001
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
            
            if globalNonessentialWings:
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
        if globalNonessentialWings:
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
            self.pubLeftMetricMean.publish(3*self.metricMeanLeft)
            self.pubRightMetric.publish(metricRight)
            self.pubLeft.publish(angleLeft)
            self.pubRight.publish(angleRight)

        return (angleLeft, angleRight)
    
    
    # Update()
    # Update the current state using the visual position and the computed position (if applicable)
    def Update(self, contourinfo, posesComputedExternal):
        if self.initialized:
            self.contourinfo = contourinfo
            
            self.dt = self.contourinfo.header.stamp - self.stampPrev
            self.stampPrev = self.contourinfo.header.stamp
            
            # The computed orientation overrides the visual orientation.
            if posesComputedExternal is not None:
                try: 
                    posesComputed = self.tfrx.transformPose('Arena', posesComputedExternal)
                    q = posesComputed.pose.orientation
                    rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                except tf.Exception, e:
                    rospy.logwarn('FLY Exception in Update(): %s' % e)
                    angleSensed = contourinfo.angle
                else:
                    angleSensed = rpy[2]
            else:
                angleSensed = contourinfo.angle

                
            

            # Update the position & orientation filters
            self.isVisible = False            
            if (self.contourinfo.x is not None) and \
               (self.contourinfo.y is not None) and \
               (angleSensed is not None) and \
               (not N.isnan(angleSensed)) and \
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
                
                a = self.dt.to_sec() / 60
                if (self.areaMean is not None):
                    self.areaMean = (1-a)*self.areaMean + a*contourinfo.area
                elif (not N.isnan(contourinfo.area)):
                    self.areaMean = contourinfo.area 

                if (self.eccMean is not None):
                    self.eccMean = (1-a)*self.eccMean + a*contourinfo.ecc
                elif (not N.isnan(contourinfo.ecc)):
                    self.eccMean = contourinfo.ecc 
                    
                self.isVisible = True
                (xKalman,yKalman,vxKalman,vyKalman) = self.kfState.Update((self.contourinfo.x, self.contourinfo.y), contourinfo.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                #(xKalman,yKalman) = (self.contourinfo.x,self.contourinfo.y) # Unfiltered.
                self.lpAngleContour.Update(angleSensed, contourinfo.header.stamp.to_sec())
                self.apAngleContour.Update(angleSensed, contourinfo.header.stamp.to_sec())

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
            #self.state.pose.orientation comes from self.GetResolvedAngleFiltered() later.
            self.state.velocity.linear.x = vx
            self.state.velocity.linear.y = vy
            self.state.velocity.linear.z = vz
            self.state.wings.left.angle = angleLeft
            self.state.wings.right.angle = angleRight

            if 'Robot' not in self.name:
                angle = self.GetResolvedAngleFiltered()
            else:
                angle = angleSensed
            (self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w) = tf.transformations.quaternion_about_axis(angle, (0,0,1))


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

            
            # Update the most recent angle of travel, and flip state.
            self.SetAngleOfTravel()
            self.UpdateFlipState()
            
                
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
                                      ns=self.name,
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

        
    

