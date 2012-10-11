#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import copy
import tf
import numpy as N
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tracking.msg import ContourInfo
from plate_tf.srv import PlateCameraConversion

FILENAME_BACKGROUND="/cameras/background.png"



###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourInfo message, and several intermediary images.
#
class ContourGenerator:

    def __init__(self):
        
        # Image Initialization
        self.preinit = False
        self.initialized = False
        
        self.bUseBackgroundSubtraction  = rospy.get_param('tracking/usebackgroundsubtraction', True)    # Set False to turn off bg subtraction.        
        self.bUseTransforms             = rospy.get_param('tracking/usetransforms', True)               # Set False to stay in camera coordinates.
        
        # Messages
        self.camerainfo = None
        queue_size_images = rospy.get_param('tracking/queue_size_images', 1)
        self.subCameraInfo       = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        self.subImageRect        = rospy.Subscriber("camera/image_rect", Image, self.Image_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True)
        #self.subImageRaw         = rospy.Subscriber("camera/image_raw", Image, self.ImageRaw_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        
        self.pubImageProcessed   = rospy.Publisher("camera/image_processed", Image)
        self.pubImageBackground  = rospy.Publisher("camera/image_background", Image)
        self.pubImageForeground  = rospy.Publisher("camera/image_foreground", Image)
        self.pubImageThreshold   = rospy.Publisher("camera/image_threshold", Image)
        self.pubImageRviz        = rospy.Publisher("camera_rviz/image_rect", Image)
        self.pubCamerainfoRviz   = rospy.Publisher("camera_rviz/camera_info", CameraInfo)
        self.pubContourInfo      = rospy.Publisher("ContourInfo", ContourInfo)
        
        if self.bUseBackgroundSubtraction:
            self.tfrx = tf.TransformListener()
        
        # Contour Info
        self.contourinfo = ContourInfo()
        self.x0_list = []
        self.y0_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        self.nContours = 0
        self.nContoursMax = rospy.get_param("tracking/nContoursMax", 20)
        self.areaContourMin = rospy.get_param("tracking/areaContourMin", 0.0)
        self.minEccForDisplay = 1.75
        self.minSumImage = 100
        self.distanceDuplicateContour = rospy.get_param('tracking/distanceDuplicateContour', 0.1)        
        # Robot Info
        #self.robot_visible = bool(rospy.get_param("robot/visible", "true"))
        #if (not self.robot_visible) and (1 < self.nContoursMax):
        if (1 < self.nContoursMax):
            self.nContoursMax -= 1
        
        self.header = None
        self.cvimage = None
        
        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.cvstorage = cv.CreateMemStorage()
        self.cvbridge = CvBridge()
        
        # Coordinate Systems
        self.frameidOutput = rospy.get_param("frameid_contours", "ImageRect")
        
        self.ptsOriginImage = PointStamped()
        self.ptsOriginImage.header.frame_id = "ImageRect"
        self.ptsOriginImage.point.x = 0
        self.ptsOriginImage.point.y = 0
        
        self.ptsOriginPlate = PointStamped()
        self.ptsOriginPlate.header.frame_id = "Plate"
        self.ptsOriginPlate.point.x = 0
        self.ptsOriginPlate.point.y = 0
        

        # Mask Info
        self.radiusMask = int(rospy.get_param("camera/mask/radius", 9999))

        self.timePrev = rospy.Time.now().to_sec()
        self.preinit = True
        

    def InitializeImages(self):
        if (self.camerainfo is not None) and (self.cvimage is not None):
            self.sizeImageRect = cv.GetSize(self.cvimage)
            
            # ROI Setup
            self.rectImage = (int(self.ptsOriginImage.point.x),
                              int(self.ptsOriginImage.point.y),
                              int(self.camerainfo.width),
                              int(self.camerainfo.height))
            
            
            self.cvimageProcessed         = cv.CreateImage(self.sizeImageRect,                              cv.IPL_DEPTH_8U,3)
            self.cvimageProcessed2        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,3)
            self.cvimageProcessedFlip     = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,3)
            self.cvimageMask              = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageBackground        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageForeground        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageThreshold         = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageThreshold2        = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            #self.cvimageDilate            = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            #self.cvimageErode             = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageZeros             = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageContour           = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,1)
            self.cvimageContourDisplay    = cv.CreateImage((self.camerainfo.width, self.camerainfo.height), cv.IPL_DEPTH_8U,3)

            cv.SetImageROI(self.cvimageProcessed, self.rectImage)
            cv.Zero(self.cvimageZeros)
            
            if self.bUseTransforms:
                b = False
                while not b:
                    try:
                        self.tfrx.waitForTransform('Plate', 
                                                   self.ptsOriginImage.header.frame_id, 
                                                   self.ptsOriginImage.header.stamp, 
                                                   rospy.Duration(1.0))
                        
                        self.ptsOriginMask = self.tfrx.transformPoint('Plate', self.ptsOriginImage)
                        self.ptsOriginMask.point.x = -self.ptsOriginMask.point.x
                        self.ptsOriginMask.point.y = -self.ptsOriginMask.point.y
                        #rospy.logwarn(self.ptsOriginMask.point)
                        b = True
                    except tf.Exception, e:
                        rospy.logwarn('Exception transforming mask frames %s->Plate:  %s' % (self.ptsOriginImage.header.frame_id, e))
            else:
                self.ptsOriginMask = PointStamped(point=Point(x=0, y=0))
                        
            cv.Zero(self.cvimageMask)
            cv.Circle(self.cvimageMask,
                      (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            #rospy.logwarn('ROI=%s' % [int(self.ptsOriginPlate.point.x),int(self.ptsOriginPlate.point.y), int(self.radiusMask)])
            
            # Create Background image
            # First image is background unless one can be loaded
            #q = cv.GetSize(self.cvimage)
            #rospy.logwarn('size1=%s' % [q[0],q[1]])
            cv.SetImageROI(self.cvimage, self.rectImage)
            
            #q = cv.GetSize(self.cvimage)
            #rospy.logwarn('size2=%s' % [q[0],q[1]])
            if self.bUseBackgroundSubtraction:
                try:
                    self.cvimageBackground = cv.LoadImage(FILENAME_BACKGROUND, cv.CV_LOAD_IMAGE_GRAYSCALE)
                except:
                    rospy.logwarn ('Saving new background image %s' % FILENAME_BACKGROUND)
                    cv.And(self.cvimage, self.cvimageMask, self.cvimageBackground)
                    cv.SaveImage(FILENAME_BACKGROUND, self.cvimageBackground)
                    #q = cv.GetSize(self.cvimageBackground)
                    #rospy.logwarn('size3=%s' % [q[0],q[1]])
              
            self.initialized = True
            

    def CameraInfo_callback (self, msgCameraInfo):
        if self.preinit:
            self.camerainfo = msgCameraInfo
            
            if not self.initialized:
                self.InitializeImages()
            

    
    def MmFromPixels (self, xIn):
        response = self.camera_from_plate(xIn, xIn)
        return (response.Xdst, response.Ydst)
        
        
    def PixelsFromMm (self, xIn):
        if self.bUseTransforms:
            ptsIn = PointStamped()
            ptsIn.header.frame_id = "Plate"
            ptsIn.point.x = 0.0
            ptsIn.point.y = 0.0
            ptsOrigin = self.tfrx.transformPoint("ImageRect", ptsIn)        
            
            ptsIn.point.x = xIn
            ptsIn.point.y = xIn
            ptsOut = self.tfrx.transformPoint("ImageRect", ptsIn)
            xOut = (ptsOut.point.x - ptsOrigin.point.x)
            
        else:
            xOut = xIn 
                
        return xOut
        
        
    def DrawAngleLine(self, cvimage, x0, y0, angle, ecc, length):
        if (not N.isnan(angle)) and (self.minEccForDisplay < ecc):
            height, width = cv.GetSize(cvimage)
            y0 = height-y0
            
            # line segment for orientation
            xmin = 0
            ymin = 0
            xmax = width-1
            ymax = height-1
    
            r = length/2
            x1 = x0 - r * N.cos(angle)
            y1 = y0 - r * N.sin(angle)
            x2 = x0 + r * N.cos(angle)
            y2 = y0 + r * N.sin(angle)
            # BUG: What happens if x,y are offscreen?
            
            
            #rospy.logwarn ('IP %s, %s, %s, %s, %s, %s, %s, %s' % (x1, width, xg, xo, x0, y0, angle, ecc))
            cv.Line(cvimage, 
                    (int(x1),int(y1)),
                    (int(x2),int(y2)), 
                    cv.CV_RGB(0,self.color_max,0))

    
    # Given the various image moments, compute the angle and eccentricity.
    # Angle is set to NaN when the image is circular. 
    # Eigenvalues/vectors of
    # [ u20, -u11,
    #  -u11,  u02]
    #
    def FindAngleEcc(self, u20, u11, u02):
        angle = float('NaN')
        ecc = 1.0
        
        if u11 != 0: # Div by zero.
            inside = 4*u11*u11 + u20*u20 - 2*u20*u02 + u02*u02
            if inside >= 0: # Complex answer.
                inside = N.sqrt(inside)
                
                # Eigenvalues & eigenvectors.
                L1 = (u20+u02-inside)/2 
                L2 = (u20+u02+inside)/2
                V1 = (u02-u20+inside)/(2*u11)
                V2 = (u02-u20-inside)/(2*u11)
                
                rise = 1
                try:
                    if L2 < L1:
                        run = -V1
                        ecc = L1/L2 # N.sqrt(1-(L1/L2)*(L1/L2))
                      
                    else:
                        run = -V2
                        ecc = L2/L1
                      
                    angle = -N.arctan2(rise, run)
                    #rospy.loginfo ('IP angle=%s, rise/run=%s/%s' % (angle, rise,run))

                except:
                    rospy.logwarn ('Exception in FindAngleEcc()')
                    pass
        
        #if N.isnan(angle):
        #    angle = 0.0
            #rospy.logwarn('isnan(angle): %0.4f/%0.4f=%s' % (rise,run,angle))

#        A = N.array([[u20, u11],[u11, u20]])
#        (l,V) = N.linalg.eig(A)
#        L = N.diag(l)
#        iMinor = N.argmin(abs(l))
#        iMajor = N.argmax(abs(l))
#        axisMinor = V.T[iMinor]
#        axisMajor = V.T[iMajor]
#        rospy.logwarn('axisMajor=%s' % axisMajor)
#        angleMajor = N.angle(N.complex(axisMajor[0],axisMajor[1]))
#        angle = angleMajor
#        
#        ecc = N.sqrt(1-l[1]/l[0])
#        #rospy.logwarn('ecc: %0.2f, %0.2f' % (ecc1,ecc))
              
        return angle, ecc
        

    def ContourFromMoments(self, moments):
        m00 = cv.GetSpatialMoment(moments,0,0)
        m10 = cv.GetSpatialMoment(moments,1,0)
        m01 = cv.GetSpatialMoment(moments,0,1)
        
        if m00 != 0.0:
            x = m10/m00
            y = m01/m00
        
        else: # There was just one pixel in the contour.
            x = None
            y = None
            #rospy.logwarn ('zero moments')
          
        u11 = cv.GetCentralMoment(moments,1,1)
        u20 = cv.GetCentralMoment(moments,2,0)
        u02 = cv.GetCentralMoment(moments,0,2)
        area = m00
        angle, ecc = self.FindAngleEcc(u20, u11, u02)
        #rospy.logwarn('u: %s, %s, %s, %s' % (m00, u20, u11, u02))

#        rospy.logwarn ('Moments: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, xy=(%s, %s)' % (m00, m10, m01, u20, u11, u02, x, y))

        return x, y, area, angle, ecc
    
    
    def AppendContour(self, cvseqContours):
        moments = cv.Moments(cvseqContours)  # Sometimes the contour contains just one pixel, resulting in moments=(0,0,0,0,0,0)
        (x, y, area, angle, ecc) = self.ContourFromMoments(moments)
        if (x is None) or (y is None):
            (x,y) = cvseqContours[0]
            area = 0.0001 # one pixel's worth.
            angle = 99.9
            ecc = 1.0
            
        # Save contour info
        ptsContour = PointStamped()
        ptsContour.header.frame_id = "ImageRect"
        ptsContour.point.x = x
        ptsContour.point.y = y
#        if (x is None) or (y is None):
#            for pt in cvseqContours:
#                rospy.logwarn(pt)
#            rospy.logwarn ('+++++++')
            
        if x is not None:
            try:
                if self.bUseTransforms:
                    self.ptsOutput = self.tfrx.transformPoint(self.frameidOutput, ptsContour)
                else:
                    self.ptsOutput = ptsContour
                    
                self.x0_list.append(self.ptsOutput.point.x)
                self.y0_list.append(self.ptsOutput.point.y)
                self.angle_list.append(angle)
                self.area_list.append(area)
                self.ecc_list.append(ecc)
                self.nContours += 1

            except tf.Exception, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))
                self.ptsOutput = PointStamped()
            except TypeError, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))
                
        


    def ContourinfoFromImage(self, cvimage):
        self.x0_list = []
        self.y0_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        
        # Find contours
        sumImage = cv.Sum(cvimage)
        if self.minSumImage < sumImage[0]:
            cvseqContours = cv.FindContours(cvimage, self.cvstorage, mode=cv.CV_RETR_CCOMP) # Modifies cvimage.
        else:
            cvseqContours = None

#        rospy.logwarn('==================')

        # Process contours
        self.nContours = 0
        if cvseqContours is not None:
            while True:
#                for pt in cvseqContours:        
#                    rospy.logwarn ('contour point=%s' % str(pt))
                    
                self.AppendContour(cvseqContours) # self.nContours++ gets incremented inside function.
#                rospy.logwarn('------------------')
                    
                if (cvseqContours.h_next()) and (self.nContours < self.nContoursMax):
                    cvseqContours = cvseqContours.h_next()
                else:
                    break
        
            # Reset cvseqContours to the start.
            while cvseqContours.h_prev():
                cvseqContours = cvseqContours.h_prev()
            
        
        # Put contours into contourinfo.
        contourinfo = ContourInfo()
        contourinfo.header.stamp = self.header.stamp
        contourinfo.header.frame_id = self.frameidOutput # i.e. Camera
        
        if self.nContours > 0:
            contourinfo.x = self.x0_list
            contourinfo.y = self.y0_list
            contourinfo.angle = self.angle_list
            contourinfo.area = self.area_list
            contourinfo.ecc = self.ecc_list
        
            
        # Remove duplicates and too-small contours.
        if self.nContours > 0:
            # Repackage the data.
            contours = []
            for iContour in range(self.nContours):
                if (self.areaContourMin <= contourinfo.area[iContour]):
                    contours.append([contourinfo.x[iContour], 
                                     contourinfo.y[iContour], 
                                     contourinfo.angle[iContour], 
                                     contourinfo.area[iContour], 
                                     contourinfo.ecc[iContour]])
            
            # Remove the dups.
            contours = sorted(tuple(contours))
            contours = [x for i, x in enumerate(contours) if (not i) or (N.linalg.norm(N.array(x[0:2])-N.array(contours[i-1][0:2])) > self.distanceDuplicateContour)]
        
            # Repackage the de-duped data.
            self.nContours = len(contours)
            contourinfo.x = []
            contourinfo.y = []
            contourinfo.angle = []
            contourinfo.area = []
            contourinfo.ecc = []
            for iContour in range(self.nContours):
                contourinfo.x.append(contours[iContour][0])
                contourinfo.y.append(contours[iContour][1])
                contourinfo.angle.append(contours[iContour][2])
                contourinfo.area.append(contours[iContour][3])
                contourinfo.ecc.append(contours[iContour][4])
            
        return contourinfo, cvseqContours    
        

    def ImageRaw_callback(self, image):
        rospy.logwarn('ImageRaw_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
        self.timePrev = rospy.Time.now().to_sec()


    def Image_callback(self, image):
#        rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
#        self.timePrev = rospy.Time.now().to_sec()

        if not self.preinit:
            return
        self.header = image.header
        
        # Convert ROS image to OpenCV image
        try:
            self.cvimage = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, "passthrough"))
            #cvimageIn = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, "passthrough"))
            #cv.CvtColor(cvimageIn, self.cvimage, cv.CV_RGB2GRAY)
        except CvBridgeError, e:
            rospy.logwarn ('Exception %s' % e)
        
        if not self.initialized:
            self.InitializeImages()

        if self.initialized:        
            radiusMask = int(rospy.get_param("camera/mask/radius", 9999)) 
            if radiusMask != self.radiusMask:
                self.radiusMask = radiusMask 
                cv.Zero(self.cvimageMask)
                cv.Circle(self.cvimageMask,
                          (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                          int(self.radiusMask), 
                          self.color_max, 
                          cv.CV_FILLED)
            
            cv.SetImageROI(self.cvimage, self.rectImage)
    
            
            # Check for new diff_threshold value
            self.diff_threshold = rospy.get_param("tracking/diff_threshold", 50)
            
            # Apply mask and Subtract background
            if self.bUseBackgroundSubtraction:
                #q1 = cv.GetSize(self.cvimage)
                #q2 = cv.GetSize(self.cvimageMask)
                #rospy.logwarn ('%s' % [[q1[0],q1[1]],[q2[0],q2[1]]])
                cv.And(self.cvimage, self.cvimageMask, self.cvimage)
                cv.AbsDiff(self.cvimage, self.cvimageBackground, self.cvimageForeground)
            else:
                cv.Copy(self.cvimage, self.cvimageForeground)
                #self.cvimageForeground = self.cvimage
                #cv.CvtColor(self.cvimage, self.cvimageForeground, cv.CV_RGB2GRAY)
            
            # Threshold
            cv.Threshold(self.cvimageForeground, 
                         self.cvimageThreshold, 
                         self.diff_threshold, 
                         self.max_8U, 
                         #cv.CV_THRESH_BINARY)
                         cv.CV_THRESH_TOZERO)

            
            # Get the ContourInfo.
            #cv.Copy(self.cvimageThreshold, self.cvimageThreshold2)
            (self.contourinfo, self.cvseqContours) = self.ContourinfoFromImage(self.cvimageThreshold)
            self.pubContourInfo.publish(self.contourinfo)
            
            # Convert to color for display image
            if self.pubImageProcessed.get_num_connections() > 0:
                cv.CvtColor(self.cvimage, self.cvimageProcessed, cv.CV_GRAY2RGB)
                
                # Draw contours on Processed image.
                if self.cvseqContours:
                    cv.DrawContours(self.cvimageProcessed, self.cvseqContours, cv.CV_RGB(0,0,self.color_max), cv.CV_RGB(0,self.color_max,0), 1, 1)
                
                
            
            # Publish processed image
            if self.pubImageProcessed.get_num_connections() > 0:
                try:
                    cv.Copy(self.cvimageProcessed, self.cvimageProcessed2)
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageProcessed2, "passthrough")
                    image2.header = image.header
                    image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
                    #rospy.logwarn(image2.encoding)
                    self.pubImageProcessed.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
            
            # Publish background image
            if (self.pubImageBackground.get_num_connections() > 0) and (self.bUseBackgroundSubtraction):
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageBackground, "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageBackground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
            
            # Publish thresholded image
            if self.pubImageThreshold.get_num_connections() > 0:
                try:
                    cv.Add(self.cvimageThreshold, self.cvimageForeground, self.cvimage)
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimage, "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageThreshold.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
              
            # Publish foreground image
            if self.pubImageForeground.get_num_connections() > 0:
                #cv.Threshold(self.cvimageForeground, 
                #             self.cvimageForeground, 
                #             self.diff_threshold, 
                #             self.max_8U, 
                #             cv.CV_THRESH_TOZERO)
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(self.cvimageForeground, "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageForeground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
              
            # Publish a special image for use in rviz.
            cv.Flip(self.cvimageProcessed,self.cvimageProcessedFlip,0)
            image2 = self.cvbridge.cv_to_imgmsg(self.cvimageProcessedFlip, "passthrough")
            image2.header = image.header
            image2.header.frame_id = 'Plate'
            image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
            camerainfo2 = copy.copy(self.camerainfo)
            camerainfo2.header.frame_id = 'Plate'
            k11=rospy.get_param('/k11', 1.0)
            k13=rospy.get_param('/k13', 0.0)
            k22=rospy.get_param('/k22', 1.0)
            k23=rospy.get_param('/k23', 0.0)
            k33=rospy.get_param('/k33', 1.0)
            p11=rospy.get_param('/p11', -0.023)
            p13=rospy.get_param('/p13', 0.0)
            p22=rospy.get_param('/p22', +0.023)
            p23=rospy.get_param('/p23', 0.0)
            p33=rospy.get_param('/p33', 1.0)
            camerainfo2.D = (0.0, 0.0, 0.0, 0.0, 0.0)
            camerainfo2.K = (k11, 0.0, k13, \
                             0.0, k22, k23, \
                             0.0, 0.0, k33)
            camerainfo2.R = (1.0, 0.0, 0.0, \
                             0.0, 1.0, 0.0, \
                             0.0, 0.0, 1.0)
            camerainfo2.P = (p11, 0.0,                    self.ptsOriginMask.point.x, 0.0, \
                             0.0, p22, camerainfo2.height-self.ptsOriginMask.point.y, 0.0, \
                             0.0, 0.0,                                           1.0, 0.0)
            self.pubCamerainfoRviz.publish(camerainfo2)
            self.pubImageRviz.publish(image2)
            del image2



    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('ContourGenerator') #, anonymous=True)
    try:
        cg = ContourGenerator()
        cg.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
