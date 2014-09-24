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
from pythonmodules import filters, CircleFunctions, SetDict


globalLock2 = threading.Lock()

        
###############################################################################
###############################################################################
###############################################################################
# class Fly()
# Implements a fly or robot object in a 2D arena. 
# Updates its state based on an image 'contour' from a single camera.
#
class Fly:
    def __init__(self, name=None, tfrx=None, lock=None):
        global globalLock2
        if (lock is not None):
            self.lock = lock
        else:
            self.lock = globalLock2
            

        self.initialized = False
        self.updated = False
        self.name = name

        # Read all the params.
        with self.lock:
            self.params = rospy.get_param('/', {})

        defaults = {'tracking':{'dtVelocity': 0.2,
                                'dtForecast': 0.15,
                                'rcFilterFlip':3.0,
                                'rcFilterSpeed':0.2,
                                'rcFilterAngle':0.1,
                                'rcFilterAngularVel':0.1,
                                'rcForeground':1.0,
                                'speedThresholdForTravel':5.0,
                                'robot':{'width':1.0,
                                         'length':1.0,
                                         'height':1.0},
                                'roi':{'width':15,
                                       'height':15},
                                'lengthBody':9,
                                'widthBody':5,
                                'thresholdBody':40.0,
                                'scalarMeanFlySubtraction':1.0,
                                'nStdDevWings':2.0,
                                'rcWingMean':10.0,                  # The time constant for exponentially weighted wing mean.
                                'rcWingStdDev':100.0,               # The time constant for exponentially weighted wing std dev.
                                'nonessentialPublish':False,
                                'nonessentialCalcSuper':False}
                    }
        SetDict.SetWithPreserve(self.params, defaults)

        self.cvbridge = CvBridge()
        self.tfrx = tfrx
        self.tfbx = tf.TransformBroadcaster()
        self.pubMarker       = rospy.Publisher('visualization_marker', Marker)
        
        # Nonessential stuff to publish.
        if self.params['tracking']['nonessentialPublish']:
            self.pubImageRoiMean        = rospy.Publisher(self.name+'/image_mean', Image)
            self.pubImageRoi            = rospy.Publisher(self.name+'/image', Image)
            self.pubImageRoiReg         = rospy.Publisher(self.name+'/image_reg', Image)
            self.pubImageRoiWings       = rospy.Publisher(self.name+'/image_wings', Image)
            self.pubImageMaskWings      = rospy.Publisher(self.name+'/image_mask_wings', Image)
            self.pubImageMaskBody       = rospy.Publisher(self.name+'/image_mask_body', Image)
            
            self.pubLeft                = rospy.Publisher(self.name+'/leftwing', Float32)
            self.pubLeftIntensity       = rospy.Publisher(self.name+'/leftintensity', Float32)
            self.pubLeftMeanIntensity   = rospy.Publisher(self.name+'/leftmean', Float32)
            self.pubLeftMeanPlusStdDev  = rospy.Publisher(self.name+'/leftmeanplusstddev', Float32)
            self.pubLeftMeanMinusStdDev = rospy.Publisher(self.name+'/leftmeanminusstddev', Float32)
            self.pubLeftDev             = rospy.Publisher(self.name+'/leftdev', Float32)

            self.pubRight               = rospy.Publisher(self.name+'/rightwing', Float32)
            self.pubRightIntensity      = rospy.Publisher(self.name+'/rightintensity', Float32)
            self.pubRightMeanIntensity  = rospy.Publisher(self.name+'/rightmean', Float32)
            self.pubRightMeanPlusStdDev = rospy.Publisher(self.name+'/rightmeanplusstddev', Float32)
            self.pubRightMeanMinusStdDev= rospy.Publisher(self.name+'/rightmeanminusstddev', Float32)
            self.pubRightDev            = rospy.Publisher(self.name+'/rightdev', Float32)
            
            self.pubImageSuper          = rospy.Publisher(self.name+'/image_super', Image)
            self.pubImageSuperCount     = rospy.Publisher(self.name+'/image_supercount', Image)
            self.pubImageSuperGrad      = rospy.Publisher(self.name+'/image_supergrad', Image)

        rospy.logwarn('(%s) Filter time constants: rcFilterAngle=%s, rcFilterAngularVel=%s' % (self.name, self.params['tracking']['rcFilterAngle'], self.params['tracking']['rcFilterAngularVel']))
        self.kfState = filters.KalmanFilter()
        self.lpAngle = filters.LowPassCircleFilter(RC=self.params['tracking']['rcFilterAngle'])
        self.lpAngle.SetValue(0.0)
        self.lpAngleContour = filters.LowPassHalfCircleFilter(RC=self.params['tracking']['rcFilterAngle'])
        self.lpAngleContour.SetValue(0.0)
        self.apAngleContour = filters.LowPassHalfCircleFilter(RC=0.0)
        self.apAngleContour.SetValue(0.0)
        self.stampPrev = rospy.Time.now()
        self.dtVelocity = rospy.Duration(self.params['tracking']['dtVelocity']) # Interval over which to calculate velocity.
        self.stampPrev = rospy.Time.now()
        
        # Orientation detection stuff.
        self.lpFlip = filters.LowPassFilter(RC=self.params['tracking']['rcFilterFlip'])
        self.lpFlip.SetValue(0.0) # (0.5) TEST
        self.lpSpeed = filters.LowPassFilter(RC=self.params['tracking']['rcFilterSpeed'])
        self.lpSpeed.SetValue(0.0)
        self.angleOfTravelRecent = 0.0
        self.contourinfo = Contourinfo()
        self.speed = 0.0

        
        self.lpWx = filters.LowPassFilter(RC=self.params['tracking']['rcFilterAngularVel'])
        self.lpWy = filters.LowPassFilter(RC=self.params['tracking']['rcFilterAngularVel'])
        self.lpWz = filters.LowPassFilter(RC=self.params['tracking']['rcFilterAngularVel'])
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
        self.eccMean = 0.0
        self.eccMax = 0.0
        
        self.areaMin = 999.9
        self.areaMean = 1.0
        self.areaMax = 0.0
        
        # Wing angle stuff.
        self.npfRoiMean = None
        self.npfRoiSum = None
        self.npMaskWings = None
        self.npMaskCircle = N.zeros([self.params['tracking']['roi']['height'], self.params['tracking']['roi']['width']],dtype=N.uint8)
        cv2.circle(self.npMaskCircle,
                  (int(self.params['tracking']['roi']['height']/2), int(self.params['tracking']['roi']['width']/2)),
                  int(self.params['tracking']['roi']['height']/2), 
                  255, 
                  cv.CV_FILLED)
        self.meanIntensityLeft  = None
        self.meanIntensityRight = None
        self.varIntensityLeft  = None
        self.varIntensityRight = None
        self.sumIntensityLeft  = 0.0
        self.sumIntensityRight = 0.0
        self.sumSqDevIntensityLeft = 0.0
        self.stddevIntensityLeft  = 0.0 
        self.sumSqDevIntensityRight = 0.0
        self.stddevIntensityRight  = 0.0 
        self.count = 0
        
        
        # Super-resolution image.
        if self.params['tracking']['nonessentialCalcSuper']:
            self.heightSuper = 16*self.params['tracking']['roi']['height'] # The resolution increase for super-res.
            self.widthSuper = 16*self.params['tracking']['roi']['width']
            self.npfSuper      = N.zeros([self.heightSuper, self.widthSuper], dtype=N.float) 
            self.npfSuperSum   = N.zeros([self.heightSuper, self.widthSuper], dtype=N.float) 
            self.npfSuperCount = N.zeros([self.heightSuper, self.widthSuper], dtype=N.float) 
        
        
        
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
        if self.speed > self.params['tracking']['speedThresholdForTravel']: 
            self.angleOfTravelRecent = angleOfTravel


    # GetNextFlipValue()
    #   Using self.contourinfo, self.state, and self.angleOfTravelRecent,
    #   we determine a value to use for updating the 'flip' filter,
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

        

        if (self.speed > self.params['tracking']['speedThresholdForTravel']):
            if (self.contourinfo.ecc > 0.6):
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
        
        if (self.speed > self.params['tracking']['speedThresholdForTravel']):
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
        q = 1
        
        if (q==1):  # Use a carefully constructed wing-area mask.
            self.npMaskWings = N.zeros([self.params['tracking']['roi']['height'], self.params['tracking']['roi']['width']], dtype=N.uint8)
    
    
            # Coordinates of the body ellipse.
            centerBody = (self.params['tracking']['roi']['width']/2, 
                          self.params['tracking']['roi']['height']/2)
            sizeBody = (self.params['tracking']['lengthBody'], 
                        self.params['tracking']['widthBody'])
            angleBody = 0.0
            
            # Left wing ellipse.
            sizeLeft = (self.params['tracking']['lengthBody']*10/10, 
                        self.params['tracking']['widthBody']*20/10)
            centerLeft = (centerBody[0] - sizeLeft[0]/2 + self.params['tracking']['lengthBody']*2/10,
                          centerBody[1] - sizeLeft[1]/2 - 1)
            angleLeft = 0.0
    
            # Right wing ellipse.
            sizeRight = (self.params['tracking']['lengthBody']*10/10, 
                         self.params['tracking']['widthBody']*20/10)
            centerRight = (centerBody[0] - sizeRight[0]/2 + self.params['tracking']['lengthBody']*2/10,
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
                (threshOut, npMaskBody) = cv2.threshold(npfRoiMean.astype(N.uint8), self.params['tracking']['thresholdBody'], 255, cv2.THRESH_BINARY_INV)
                self.npMaskWings = cv2.bitwise_and(self.npMaskWings, npMaskBody)
            
                # Publish the body mask.
                if self.params['tracking']['nonessentialPublish']:
                    npMaskBody1 = copy.copy(npMaskBody)
                    npMaskBody1.resize(npMaskBody1.size)
                    imgMaskBody  = self.cvbridge.cv_to_imgmsg(cv.fromarray(npMaskBody), 'passthrough')
                    imgMaskBody.data = list(npMaskBody1)
                    self.pubImageMaskBody.publish(imgMaskBody)
                    
        
        elif (q==2):    # Use all pixels.
            self.npMaskWings = 255*N.ones([self.params['tracking']['roi']['height'], self.params['tracking']['roi']['width']], dtype=N.uint8)
            
        elif (q==3):    # Only use the left pixels.
            w1 = int(6.0*float(self.params['tracking']['roi']['width'])/10.0)     # Width of left side.
            w2 = int(self.params['tracking']['roi']['width']) - w1     # Width of right side.
            self.npMaskWings = 255*N.hstack([N.ones([self.params['tracking']['roi']['height'], w1], dtype=N.uint8),
                                             N.zeros([self.params['tracking']['roi']['height'], w2], dtype=N.uint8)])
            
        # Publish the wing mask.
        if self.params['tracking']['nonessentialPublish']:
            npMaskWings1 = copy.copy(self.npMaskWings)
            npMaskWings1.resize(npMaskWings1.size)
            imgMaskWings  = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npMaskWings), 'passthrough')
            imgMaskWings.data = list(npMaskWings1)
            self.pubImageMaskWings.publish(imgMaskWings)
    
    
        
    def UpdateRoiMean(self, npRoiReg):
        if (self.count > 100) and (self.npfRoiMean is not None):
            # Exponentially weighted mean.
            alphaForeground = 1 - N.exp(-self.dt.to_sec() / self.params['tracking']['rcForeground'])
            cv2.accumulateWeighted(N.float64(npRoiReg), self.npfRoiMean, alphaForeground)
        else:
            # Use straight mean for first N values.
            if (self.npfRoiSum is None):
                self.npfRoiSum = N.zeros(npRoiReg.shape, dtype=N.float)
                
            self.npfRoiSum += N.float32(npRoiReg)
            self.npfRoiMean = self.npfRoiSum / self.count

        

    def UpdateRoiSuperresolution(self, npRoi, moments):
        if self.params['tracking']['nonessentialCalcSuper']:
            if (moments['m00'] != 0.0):
                xCOM  = moments['m10']/moments['m00']
                yCOM  = moments['m01']/moments['m00']

                
                # Rotate the fly image to 0-degrees, and the size of the super image.
                angleR = self.GetResolvedAngleFiltered()
                scale = float(self.widthSuper)/float(npRoi.shape[1])
                x2Center = float(self.widthSuper)/2.0
                y2Center = float(self.heightSuper)/2.0
                
                # The transform for ROI rotation & scale.
                T = cv2.getRotationMatrix2D((xCOM,yCOM), -angleR*180.0/N.pi, scale)
                L = T.tolist()
                L.append([0.0, 0.0, 1.0])
                T1 = N.array(L)
                
                # The transform to the center of the super image.
                Ttrans = N.array([[1, 0, x2Center-float(self.params['tracking']['roi']['width'])/2.0],
                                  [0, 1, y2Center-float(self.params['tracking']['roi']['height'])/2.0],
                                  [0, 0, 1],
                                  ], dtype=N.float)
                
                # The complete transform.
                T2 = N.dot(Ttrans,T1)
                
                
                # Update the Super image from the ROI pixels.
                method='sample_perpixelaveraging'
                
                if (method=='sample_perpixelaveraging'):
                    # This method only updates those few pixels in the ROI, and averages the super pixels on a per pixel basis.
                    npfSuperNow = N.zeros([self.heightSuper, self.widthSuper], dtype=N.float) 
                    for x in range(npRoi.shape[1]):
                        for y in range(npRoi.shape[0]):
                            (x2,y2,z2) = N.dot(T2, N.array([x, y, 1],dtype=float))
                            if (0.0<=x2<float(self.widthSuper)) and (0.0<=y2<float(self.heightSuper)): 
                                self.npfSuperCount[int(y2),int(x2)] += 1.0
                                self.npfSuperSum[int(y2),int(x2)] += npRoi[y,x]
                                
                    self.npfSuper = self.npfSuperSum / self.npfSuperCount

                                
                elif (method=='sample_wholeimageaveraging'):
                    # This method only updates those few pixels in the ROI, and does a moving average of the whole super image.
                    npfSuperNow = N.zeros([self.heightSuper, self.widthSuper], dtype=N.float) 
                    for x in range(npRoi.shape[1]):
                        for y in range(npRoi.shape[0]):
                            (x2,y2,z2) = N.dot(T2, N.array([x, y, 1],dtype=float))
                            if (0<=x2<self.widthSuper) and (0<=y2<self.heightSuper): 
                                npfSuperNow[int(y2),int(x2)] = npRoi[y,x]
                                
                    a = 1 - N.exp(-self.dt.to_sec() / self.params['tracking']['rcForeground'])
                    #self.npfSuper = (1-a)*self.npfSuper + a*npfSuperNow
                    self.npfSuper = N.clip((1-a)*self.npfSuper + 0.5*npfSuperNow, 0.0, 255.0)
                    
                                
                elif (method=='interpolate'): 
                    # This method uses a moving average of an interpolated resized image.
                    npfSuperNow = cv2.warpAffine(npRoi, T2[:-1,:], (self.widthSuper,self.heightSuper), flags=cv2.INTER_LANCZOS4)
                    
                    a = 1 - N.exp(-self.dt.to_sec() / self.params['tracking']['rcForeground'])
                    self.npfSuper = (1-a)*self.npfSuper + a*npfSuperNow


                if self.params['tracking']['nonessentialPublish']:
                    imgSuper  = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.uint8(self.npfSuper)), 'passthrough')
                    self.pubImageSuper.publish(imgSuper)
                    
                    imgSuperCount  = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.uint8((self.npfSuperCount/N.max(self.npfSuperCount))*255.0)), 'passthrough')
                    self.pubImageSuperCount.publish(imgSuperCount)

#                     (mask, npfGradient) = cv2.calcMotionGradient(self.npfSuper, 0.1, 255.0)
#                     imgSuperGrad  = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.uint8(npfGradient/360.0*255.0)), 'passthrough')
#                     self.pubImageSuperGrad.publish(imgSuperGrad)
        
        
    # Rotate the image to 0 degrees.                    
    def RegisterImageRoi(self, npRoi, moments):
        npRoiReg = npRoi

        # Center of mass.
        if (moments['m00'] != 0.0):
            xCOM  = moments['m10']/moments['m00']
            yCOM  = moments['m01']/moments['m00']
            
            # Rotate the fly image to 0-degrees.
            angleR = self.GetResolvedAngleFiltered()
            T = cv2.getRotationMatrix2D((xCOM,yCOM), -angleR*180.0/N.pi, 1.0)
            npRoiReg = cv2.warpAffine(npRoi, T, (0,0))
        
        
        # Correct for centering error after the transformation.
        momentsRoiReg = cv2.moments(npRoiReg)
        if (momentsRoiReg['m00'] != 0.0):
            xCOM  = momentsRoiReg['m10']/momentsRoiReg['m00']
            yCOM  = momentsRoiReg['m01']/momentsRoiReg['m00']
            
            # Rotate the fly image to 0-degrees.
            T = N.array([[1, 0, float(self.params['tracking']['roi']['width'])/2.0-xCOM],
                         [0, 1, float(self.params['tracking']['roi']['height'])/2.0-yCOM]], dtype=float)
            npRoiReg = cv2.warpAffine(npRoiReg, T, (0,0))

            
        return npRoiReg
        

    # GetWingAngles()
    # Compute the left & right wing angles from the given contourinfo angle and image.
    #
    def GetWingAngles(self, contourinfo):
        if (contourinfo is not None) and (contourinfo.imgRoi is not None) and (len(contourinfo.imgRoi.data)>0):
            # Convert ROS image to numpy.
            npRoiIn = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(contourinfo.imgRoi, 'passthrough')))
            
            # Apply the fly-rotation mask.
            npRoi = cv2.bitwise_and(npRoiIn, self.npMaskCircle)
            
            # Orient the fly to 0 degrees.
            moments = cv2.moments(npRoi)
            npRoiReg = self.RegisterImageRoi(npRoi, moments)
            
    
            # Update various images.
            self.UpdateRoiSuperresolution(npRoi, moments)
            self.UpdateWingMask(self.npfRoiMean)
            self.UpdateRoiMean(npRoiReg)
    
            
            # Create a 'wing foreground', i.e. Background Fly Subtraction.
            diff = npRoiReg.astype(N.float32) - self.npfRoiMean*self.params['tracking']['scalarMeanFlySubtraction'] # Subtract a multiple of the average fly.
            npWings = N.clip(diff, 0, N.iinfo(N.uint8).max).astype(N.uint8)
            #npWings = npRoiReg 
            
            # Mask the input ROI.
            #npRoiReg = cv2.bitwise_and(npRoiReg, self.npMaskWings)
            
            # Mask the wing foreground.
            npWings = cv2.bitwise_and(npWings, self.npMaskWings)
                            
            # Create images with only pixels of left wing (i.e. image top) or right wing (i.e. image bottom).
            npWingLeft   = N.vstack((npWings[:float(self.params['tracking']['roi']['height'])/2.0, :],
                                     N.zeros([float(self.params['tracking']['roi']['height'])/2.0+1.0, self.params['tracking']['roi']['width']],dtype=N.uint8)))
            npWingRight  = N.vstack((N.zeros([float(self.params['tracking']['roi']['height'])/2.0+1.0, self.params['tracking']['roi']['width']],dtype=N.uint8), 
                                     npWings[(float(self.params['tracking']['roi']['height'])/2.0+1.0):, :]))
            
            # Wing moments.
            momentsRoiReg = cv2.moments(npRoiReg)
            momentsLeft  = cv2.moments(npWingLeft)
            momentsRight = cv2.moments(npWingRight)
    
            # Coordinates of body, and wings relative to wing hinge.
            try:
                xBody  = (momentsRoiReg['m10']/momentsRoiReg['m00'])
                yBody  = (momentsRoiReg['m01']/momentsRoiReg['m00'])
            except ZeroDivisionError:
                xBody = float(self.params['tracking']['roi']['width'])/2.0
                yBody = float(self.params['tracking']['roi']['height'])/2.0
                
            try:
                xLeftAbs  = (momentsLeft['m10']/momentsLeft['m00'])   # Absolute pixel location.
                yLeftAbs  = (momentsLeft['m01']/momentsLeft['m00'])
            except ZeroDivisionError:
                xLeftAbs = (xBody + self.params['tracking']['lengthBody']/4.0)
                yLeftAbs = yBody
                
            xLeft = xLeftAbs - (xBody + self.params['tracking']['lengthBody']/4.0)      # Hinge-relative location.
            yLeft = yLeftAbs - yBody
            
                
            try:
                xRightAbs = (momentsRight['m10']/momentsRight['m00'])   # Absolute pixel location.
                yRightAbs = (momentsRight['m01']/momentsRight['m00'])
            except ZeroDivisionError:
                xRightAbs = (xBody + self.params['tracking']['lengthBody']/4)
                yRightAbs = yBody
                
            xRight = xRightAbs - (xBody + self.params['tracking']['lengthBody']/4.0)      # Hinge-relative location.
            yRight = yRightAbs - yBody
            
    
    #        if self.params['tracking']['nonessentialPublish']:
    #            # Mark a pixel for the body.
    #            x = N.round(xBody).astype(N.uint8)
    #            y = N.round(yBody).astype(N.uint8)
    #            npRoiReg[y, x] = 255.0
    
            
            # Wing intensity, after masking and mean fly subtraction.
            intensityLeft = momentsLeft['m00']
            intensityRight = momentsRight['m00']
    
    
            # Wing intensity mean
            self.sumIntensityLeft += intensityLeft
            self.sumIntensityRight += intensityRight
            if (self.count > 100) and (self.meanIntensityLeft is not None):
                # Exponentially weighted mean.
                a = 1 - N.exp(-self.dt.to_sec() / self.params['tracking']['rcWingMean'])
                self.meanIntensityLeft  = (1-a)*self.meanIntensityLeft  + a*intensityLeft 
                self.meanIntensityRight = (1-a)*self.meanIntensityRight + a*intensityRight 
            else:
                # Use straight mean for first N values.
                self.meanIntensityLeft  = self.sumIntensityLeft / self.count 
                self.meanIntensityRight = self.sumIntensityRight / self.count 
    
            
            # Wing intensity deviation.
            devIntensityLeft = (intensityLeft-self.meanIntensityLeft)
            devIntensityRight = (intensityRight-self.meanIntensityRight)
    
            if (self.count > 100) and (self.varIntensityLeft is not None):
                # Exponentially weighted variance.
                a = 1 - N.exp(-self.dt.to_sec() / self.params['tracking']['rcWingStdDev'])
                self.varIntensityLeft  = (1-a)*self.varIntensityLeft  + a*devIntensityLeft**2
                self.varIntensityRight  = (1-a)*self.varIntensityRight  + a*devIntensityRight**2
            else:
                # Use straight variance for first N values.
                self.sumSqDevIntensityLeft  += devIntensityLeft**2 
                self.sumSqDevIntensityRight += devIntensityRight**2
                self.varIntensityLeft = self.sumSqDevIntensityLeft/self.count
                self.varIntensityRight = self.sumSqDevIntensityRight/self.count
        
            # Standard deviation.
            self.stddevIntensityLeft  = N.sqrt(self.varIntensityLeft) 
            self.stddevIntensityRight = N.sqrt(self.varIntensityRight) 
    
            #if ('Fly01' in self.name):
            #    rospy.logwarn('intensity: %9.4f, mean: %9.4f, stddev: %9.4f' % (intensityLeft, self.meanIntensityLeft, self.stddevIntensityLeft))
            
            npToUse = npWings #npWingRight
    
            #if (N.abs(devIntensityLeft) < self.params['tracking']['nStdDevWings']*self.stddevIntensityLeft):
            if (       devIntensityLeft  < self.params['tracking']['nStdDevWings']*self.stddevIntensityLeft):
                angleLeft = N.pi
            else:
                angleLeft = N.arctan2(yLeft, xLeft)+N.pi
    
                if self.params['tracking']['nonessentialPublish']:
                    
                    # Mark a pixel for the left wing.
                    if (momentsLeft['m00'] != 0):
                        x = N.round(xLeftAbs).astype(N.uint8)
                        y = N.round(yLeftAbs).astype(N.uint8)
                        npRoiReg[y, x] = 255.0
                        npToUse[y,x] = 255.0
    
                
            #if (N.abs(devIntensityRight) < self.params['tracking']['nStdDevWings']*self.stddevIntensityRight):
            if (       devIntensityRight  < self.params['tracking']['nStdDevWings']*self.stddevIntensityRight):
                angleRight = N.pi
            else:
                angleRight = N.arctan2(yRight, xRight)+N.pi
                
                if self.params['tracking']['nonessentialPublish']:
                    if (momentsRight['m00'] != 0):
                    
                    # Mark a pixel for the right wing.
                        x = N.round(xRightAbs).astype(N.uint8)
                        y = N.round(yRightAbs).astype(N.uint8)
                        npRoiReg[y, x] = 255.0
                        npToUse[y,x] = 255.0
                
            #npToUse[N.round(yBody).astype(N.uint8), N.round(xBody).astype(N.uint8)] = 255.0      # Set pixel at body center.
            #npRoiReg[N.round(yBody).astype(N.uint8), N.round(xBody).astype(N.uint8)] = 255.0      # Set pixel at bosy center.
    
            # Nonessential stuff to publish.
            if self.params['tracking']['nonessentialPublish']:
                self.imgRoiReg  = self.cvbridge.cv_to_imgmsg(cv.fromarray(npRoiReg), 'passthrough') 
                self.imgRoiMean = self.cvbridge.cv_to_imgmsg(cv.fromarray(N.clip(self.npfRoiMean*self.params['tracking']['scalarMeanFlySubtraction'], 0, N.iinfo(N.uint8).max).astype(N.uint8)), 'passthrough')
    
                imgWings = copy.copy(self.imgRoiReg)
                npWings1 = npToUse.astype(N.uint8).reshape(npToUse.size)
                imgWings.data = list(10*npWings1) # Make the wings more visible for debugging.
                self.pubImageRoi.publish(contourinfo.imgRoi)
                self.pubImageRoiReg.publish(self.imgRoiReg)
                self.pubImageRoiMean.publish(self.imgRoiMean)
                self.pubImageRoiWings.publish(imgWings)
                
                self.pubLeft.publish(angleLeft)
                self.pubLeftIntensity.publish(intensityLeft)
                self.pubLeftMeanIntensity.publish(self.meanIntensityLeft)
                self.pubLeftMeanPlusStdDev.publish(self.meanIntensityLeft + self.params['tracking']['nStdDevWings']*self.stddevIntensityLeft)
                self.pubLeftMeanMinusStdDev.publish(self.meanIntensityLeft - self.params['tracking']['nStdDevWings']*self.stddevIntensityLeft)
                self.pubLeftDev.publish(devIntensityLeft)
                
                self.pubRight.publish(angleRight)
                self.pubRightIntensity.publish(-intensityRight)
                self.pubRightMeanIntensity.publish(-self.meanIntensityRight)
                self.pubRightMeanPlusStdDev.publish(-(self.meanIntensityRight + self.params['tracking']['nStdDevWings']*self.stddevIntensityRight))
                self.pubRightMeanMinusStdDev.publish(-(self.meanIntensityRight - self.params['tracking']['nStdDevWings']*self.stddevIntensityRight))
                self.pubRightDev.publish(-devIntensityRight)
        else:
            angleLeft = N.pi
            angleRight = N.pi
                

        return (angleLeft, angleRight)
    
    
    # Update()
    # Update the fly:  current state using the visual position and the computed position (if applicable), mean fly, superres, etc.
    def Update(self, contourinfo):
        if (self.initialized):
            self.count += 1
            
            #rospy.logwarn(contourinfo)    
            if (contourinfo is not None) and (not N.isnan(contourinfo.x)) and (not N.isnan(contourinfo.y)) and (not N.isnan(contourinfo.angle)):
                bValidContourinfo = True
            else:
                bValidContourinfo = False

                # Use null contourinfo.    
                contourinfoNone = Contourinfo()
                contourinfoNone.header = Header()
                contourinfoNone.x = None
                contourinfoNone.y = None
                contourinfoNone.angle = None
                contourinfoNone.area = None
                contourinfoNone.ecc = None
                contourinfoNone.imgRoi = None
                contourinfo = contourinfoNone

            self.contourinfo = contourinfo
            
            self.dt = self.contourinfo.header.stamp - self.stampPrev
            self.stampPrev = self.contourinfo.header.stamp
            
            #with self.lock:
            #    SetDict.SetWithOverwrite(self.params, rospy.get_param('/',{}))
            
            

            # Update the position & orientation filters
            self.isVisible = False            
            if (bValidContourinfo):
                
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
                if (self.updated):
                    self.areaMean = (1-a)*self.areaMean + a*contourinfo.area
                    self.eccMean = (1-a)*self.eccMean + a*contourinfo.ecc
                else:
                    #if (not N.isnan(contourinfo.area)):
                    self.areaMean = contourinfo.area 

                    #if (not N.isnan(contourinfo.ecc)):
                    self.eccMean = contourinfo.ecc 
                    
                    
                self.isVisible = True
                (xKalman,yKalman,vxKalman,vyKalman) = self.kfState.Update((self.contourinfo.x, self.contourinfo.y), contourinfo.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                #(xKalman,yKalman) = (self.contourinfo.x,self.contourinfo.y) # Unfiltered.

                self.lpAngleContour.Update(contourinfo.angle, contourinfo.header.stamp.to_sec())
                self.apAngleContour.Update(contourinfo.angle, contourinfo.header.stamp.to_sec())

                (angleLeft, angleRight) = self.GetWingAngles(contourinfo)
                
                
                if (N.abs(self.contourinfo.x)>9999) or (N.abs(xKalman)>9999):
                    rospy.logwarn ('FLY LARGE CONTOUR, x,x=%s, %s.  Check your background image, lighting, and the parameter tracking/diff_threshold.' % (self.contourinfo.x, xKalman))


            else: # We don't have a contourinfo.
                #rospy.logwarn('FLY No contour seen for %s; check your nFlies parameter.' % self.name)
                (xKalman, yKalman, vxKalman, vyKalman) = self.kfState.Update(None, contourinfo.header.stamp.to_sec())
                (zKalman, vzKalman) = (0.0, 0.0)
                (angleLeft, angleRight) = (N.pi, N.pi)

            if (contourinfo.angle is None):
                contourinfo.angle = 0.0
                
            # If good data, then use it.
            if (xKalman is not None) and (yKalman is not None):
                x = xKalman
                y = yKalman
                z = zKalman
                vx = vxKalman
                vy = vyKalman
                vz = vzKalman
            else: # Use the unfiltered data for the case where the filters return None results.
                rospy.logwarn('FLY Object %s not yet initialized, %s' % (self.name, [self.contourinfo.x, self.contourinfo.y]))
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

            # HACK
            self.state.pose.position.x = -4.1
            self.state.pose.position.y = -4.0
            # HACK

            if 'Robot' not in self.name:
                angle = self.GetResolvedAngleFiltered()
            else:
                angle = contourinfo.angle
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
            
                
            if (bValidContourinfo):
                # Send the Raw transform.
                if self.isVisible:
                    self.tfbx.sendTransform((self.contourinfo.x, 
                                             self.contourinfo.y, 
                                             0.0),
                                            tf.transformations.quaternion_about_axis(self.contourinfo.angle, (0,0,1)),
                                            self.contourinfo.header.stamp,
                                            self.name+'Contour',
                                            'Arena')
                
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
                    poseForecast = Pose()#copy.copy(self.state.pose)
                    poseForecast.position.x = self.state.pose.position.x + self.state.velocity.linear.x * self.params['tracking']['dtForecast']
                    poseForecast.position.y = self.state.pose.position.y + self.state.velocity.linear.y * self.params['tracking']['dtForecast']
                    poseForecast.position.z = self.state.pose.position.z + self.state.velocity.linear.z * self.params['tracking']['dtForecast']
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
                                          scale=Vector3(x=self.params['tracking']['robot']['width'],
                                                        y=self.params['tracking']['robot']['length'],
                                                        z=self.params['tracking']['robot']['height']),
                                          color=ColorRGBA(a=0.7,
                                                          r=0.5,
                                                          g=0.5,
                                                          b=0.5),
                                          lifetime=rospy.Duration(0.5))
                    self.pubMarker.publish(markerRobot)
                
            self.updated = True
        
    

