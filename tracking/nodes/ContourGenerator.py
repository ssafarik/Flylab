#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import cv2
import copy
import tf
import numpy as N
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from flycore.msg import TrackingCommand
from tracking.msg import ContourInfo
from plate_tf.srv import PlateCameraConversion




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
        self.subTrackingCommand  = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        
        self.pubImageProcessed   = rospy.Publisher("camera/image_processed", Image)
        self.pubImageBackground  = rospy.Publisher("camera/image_background", Image)
        self.pubImageForeground  = rospy.Publisher("camera/image_foreground", Image)
        self.pubImageThreshold   = rospy.Publisher("camera/image_threshold", Image)
        self.pubImageRviz        = rospy.Publisher("camera_rviz/image_rect", Image)
        self.pubCamerainfoRviz   = rospy.Publisher("camera_rviz/camera_info", CameraInfo)
        self.pubContourInfo      = rospy.Publisher("ContourInfo", ContourInfo)
        
        if self.bUseTransforms:
            self.tfrx = tf.TransformListener()
        
        self.filenameBackground = rospy.get_param('tracking/filenameBackground', '/cameras/background.png')
        self.alphaBackground = rospy.get_param('tracking/alphaBackground', 0.001) # Alpha value for moving average background.
#        if self.bUseBackgroundSubtraction:
#            self.bghistory = 100
#            self.bgnmixtures = 2
#            self.bgratio = 0.5
#            self.bg = cv2.BackgroundSubtractorMOG(self.bghistory, self.bgnmixtures, self.bgratio)

        
        # Contour Info
        self.contourinfo_lists = ContourInfo()
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
        if (1 < self.nContoursMax):
            self.nContoursMax -= 1
        
        self.header = None
        self.npCamera = None
        
        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
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
        if (self.camerainfo is not None) and (self.npCamera is not None):

            # Initialize the images (in opencv Mat format).
            #self.matProcessed         = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC3)
            #self.matProcessedFlip     = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC3)
            #self.matMask              = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC1)
            #self.matBackground        = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC1)
            #self.matForeground        = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC1)
            #self.matThreshold         = cv.CreateMat(self.camerainfo.height, self.camerainfo.width, cv.CV_8UC1)
            
            self.npProcessed         = N.zeros([self.camerainfo.height, self.camerainfo.width, 3], dtype=N.uint8)
            self.npProcessedFlip     = N.zeros([self.camerainfo.height, self.camerainfo.width, 3], dtype=N.uint8)
            self.npMask              = N.zeros([self.camerainfo.height, self.camerainfo.width], dtype=N.uint8)
            self.npForeground        = N.zeros([self.camerainfo.height, self.camerainfo.width], dtype=N.uint8)
            self.npBackground        = N.zeros([self.camerainfo.height, self.camerainfo.width], dtype=N.uint8)
            self.npThreshold         = N.zeros([self.camerainfo.height, self.camerainfo.width], dtype=N.uint8)

            
            
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
                        b = True
                    except tf.Exception, e:
                        rospy.logwarn('Exception transforming mask frames %s->Plate:  %s' % (self.ptsOriginImage.header.frame_id, e))
            else:
                self.ptsOriginMask = PointStamped(point=Point(x=0, y=0))
                        
            #cv.Zero(self.matMask)
            cv2.circle(self.npMask,
                      (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            
            # Create Background image
            # First image is background unless one can be loaded
            if self.bUseBackgroundSubtraction:
                try:
                    matBackground = cv.LoadImageM(self.filenameBackground, cv.CV_LOAD_IMAGE_GRAYSCALE)
                    self.npBackground  = N.uint8(matBackground)
                    self.npfBackground = N.float32(matBackground)
                except:
                    rospy.logwarn ('Saving new background image %s' % self.filenameBackground)
                    self.npBackground = cv2.bitwise_and(self.npCamera, self.npMask)
                    cv.SaveImage(self.filenameBackground, cv.fromarray(self.npBackground))
              
                #self.histModel = cv2.calcHist([N.uint8(self.matBackground)], [0], N.uint8(self.matMask), [255], [0,255])
                #rospy.logwarn (self.histModel)
                self.npBuffer = N.zeros([self.npCamera.shape[0], self.npCamera.shape[1]], dtype=N.uint8) #N.uint8(self.matCamera)
                
            self.initialized = True
            

    def CameraInfo_callback (self, msgCameraInfo):
        if self.preinit:
            self.camerainfo = msgCameraInfo
            
            if not self.initialized:
                self.InitializeImages()
            
            
    # TrackingCommand_callback()
    # Receives various commands to change the tracking behavior.
    # See TrackingCommand.msg for details.
    #
    def TrackingCommand_callback(self, trackingcommand):
        if trackingcommand.command == 'savebackground':
            rospy.logwarn ('Saving new background image %s' % self.filenameBackground)
            cv.SaveImage(self.filenameBackground, cv.GetMat(self.npBackground))

    
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
            height, width = cvimage.size #cv.GetSize(cvimage)
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

                except:
                    rospy.logwarn ('Exception in FindAngleEcc()')
                    pass
        
        return angle, ecc
        

    def ContourinfoFromMoments(self, moments):
        #rospy.logwarn('moments=%s' % repr(moments))
        m00 = moments['m00'] #cv.GetSpatialMoment(moments,0,0)
        m10 = moments['m10'] #cv.GetSpatialMoment(moments,1,0)
        m01 = moments['m01'] #cv.GetSpatialMoment(moments,0,1)
        
        if m00 != 0.0:
            x = m10/m00
            y = m01/m00
        
        else: # There was just one pixel in the contour.
            x = None
            y = None
            #rospy.logwarn ('zero moments')
          
        u11 = moments['mu11'] #cv.GetCentralMoment(moments,1,1)
        u20 = moments['mu20'] #cv.GetCentralMoment(moments,2,0)
        u02 = moments['mu02'] #cv.GetCentralMoment(moments,0,2)
        area = m00
        angle, ecc = self.FindAngleEcc(u20, u11, u02)
        #rospy.logwarn('u: %s, %s, %s, %s' % (m00, u20, u11, u02))

#        rospy.logwarn ('Moments: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, xy=(%s, %s)' % (m00, m10, m01, u20, u11, u02, x, y))

        return (x, y, area, angle, ecc)
    
    
    # AppendContourinfoListsFromContour()
    # Converts contour to a contourinfo, transforms it to the output frame, and appends the contourinfo members to their respective lists.
    #
    def AppendContourinfoListsFromContour(self, contour):
        moments = cv2.moments(contour)  # Sometimes the contour contains just one pixel, resulting in moments=(0,0,0,0,0,0)
        (x, y, area, angle, ecc) = self.ContourinfoFromMoments(moments)
        if (x is None) or (y is None):
            (x,y) = contour[0][0]
            area = 0.0001 # one pixel's worth.
            angle = 99.9
            ecc = 1.0
            
        # Save contourinfo_lists
        ptsContour = PointStamped()
        ptsContour.header.frame_id = "ImageRect"
        ptsContour.point.x = x
        ptsContour.point.y = y
            
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
                
        


    def ContourinfoListFromImage(self, npImage):
        self.x0_list = []
        self.y0_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        
        # Find contours
        sumImage = cv2.sumElems(npImage)
        if self.minSumImage < sumImage[0]:
            (contours,hierarchy) = cv2.findContours(npImage, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE) # Modifies npImage.
        else:
            (contours,hierarchy) = (None, None)
            

        # Put the top-level contours into the contourinfo_lists
        self.nContours = 0
        if contours is not None:
            (NEXT,PREV,CHILD,PARENT)=(0,1,2,3)
            iContour = 0
            while (True):    # While there's a next contour. 
                contour = contours[iContour]
                self.AppendContourinfoListsFromContour(contour) # self.nContours++ gets incremented inside function.
                iContour = hierarchy[0][iContour][NEXT]
                if iContour<0:
                    break
                    
        
        # Put lists into contourinfo_lists.
        contourinfo_lists = ContourInfo()
        contourinfo_lists.header.stamp = self.header.stamp
        contourinfo_lists.header.frame_id = self.frameidOutput # i.e. Camera
        
        if self.nContours > 0:
            contourinfo_lists.x = self.x0_list
            contourinfo_lists.y = self.y0_list
            contourinfo_lists.angle = self.angle_list
            contourinfo_lists.area = self.area_list
            contourinfo_lists.ecc = self.ecc_list
        
            
        # Remove duplicates and too-small contours.
        if self.nContours > 0:
            # Repackage the data.
            contourinfo_list = []
            for iContour in range(self.nContours):
                if (self.areaContourMin <= contourinfo_lists.area[iContour]):
                    contourinfo_list.append([contourinfo_lists.x[iContour], 
                                     contourinfo_lists.y[iContour], 
                                     contourinfo_lists.angle[iContour], 
                                     contourinfo_lists.area[iContour], 
                                     contourinfo_lists.ecc[iContour]])
            
            # Remove the dups.
            contourinfo_list = sorted(tuple(contourinfo_list))
            contourinfo_list = [x for i, x in enumerate(contourinfo_list) if (not i) or (N.linalg.norm(N.array(x[0:2])-N.array(contourinfo_list[i-1][0:2])) > self.distanceDuplicateContour)]
        
            # Repackage the de-duped data.
            self.nContours = len(contourinfo_list)
            contourinfo_lists.x = []
            contourinfo_lists.y = []
            contourinfo_lists.angle = []
            contourinfo_lists.area = []
            contourinfo_lists.ecc = []
            for iContour in range(self.nContours):
                contourinfo_lists.x.append(contourinfo_list[iContour][0])
                contourinfo_lists.y.append(contourinfo_list[iContour][1])
                contourinfo_lists.angle.append(contourinfo_list[iContour][2])
                contourinfo_lists.area.append(contourinfo_list[iContour][3])
                contourinfo_lists.ecc.append(contourinfo_list[iContour][4])
            
        return contourinfo_lists, contours    
        

    def Image_callback(self, image):
#        rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
#        self.timePrev = rospy.Time.now().to_sec()

        if not self.preinit:
            return
        self.header = image.header
        
        # Convert ROS image to OpenCV image.
        try:
            self.npCamera = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, "passthrough")))
        except CvBridgeError, e:
            rospy.logwarn ('Exception %s' % e)
        
        if not self.initialized:
            self.InitializeImages()

        if self.initialized:        
            # Check for new diff_threshold value
            self.diff_threshold = rospy.get_param("tracking/diff_threshold", 50)

            radiusMask = int(rospy.get_param("camera/mask/radius", 9999)) 
            if radiusMask != self.radiusMask:
                self.radiusMask = radiusMask 
                self.npMask = N.zeros([self.camerainfo.height, self.camerainfo.width], dtype=N.uint8) #cv.Zero(self.matMask)
                cv2.circle(self.npMask,
                          (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                          int(self.radiusMask), 
                          self.color_max, 
                          cv.CV_FILLED)
            
            # Mask the camera image.
            self.npCamera = cv2.bitwise_and(self.npCamera, self.npMask)
            
            # Normalize the histogram.
            #self.npBuffer = N.zeros([self.matCamera.height, self.matCamera.width], dtype=N.uint8)
            self.npCamera = cv2.equalizeHist(self.npCamera)
            
            
            if self.bUseBackgroundSubtraction:
                # Update the background image
                cv2.accumulateWeighted(N.float32(self.npCamera), self.npfBackground, self.alphaBackground)
                #self.matBackground = cv.fromarray(N.uint8(self.npfBackground))
                self.npBackground = N.uint8(self.npfBackground)

                # Subtract background
                #self.npBuffer = N.zeros([self.matCamera.height, self.matCamera.width], dtype=N.uint8)
                npBuffer = cv2.equalizeHist(self.npBackground)
                self.npBackground2 = npBuffer
                self.npForeground = cv2.absdiff(self.npCamera, self.npBackground2)
            else:
                self.npForeground = self.npCamera

            
            # Threshold
            (threshold,self.npThreshold) = cv2.threshold(self.npForeground, 
                                             self.diff_threshold, 
                                             self.max_8U, 
                                             cv2.THRESH_TOZERO)
            #rospy.logwarn ('self.npThreshold=%s' % repr(self.npThreshold))

            
            # Get the ContourInfo.
            (self.contourinfo_lists, self.contours) = self.ContourinfoListFromImage(self.npThreshold)
            self.pubContourInfo.publish(self.contourinfo_lists)
            
            # Convert to color for display image
            if self.pubImageProcessed.get_num_connections() > 0:
                self.npProcessed = cv2.cvtColor(self.npCamera, cv.CV_GRAY2RGB)
                
                # Draw contours on Processed image.
                if self.contours:
                    #cv.DrawContours(self.matProcessed, self.contours, cv.CV_RGB(0,0,self.color_max), cv.CV_RGB(0,self.color_max,0), 1, 1)
                    cv2.drawContours(self.npProcessed, self.contours, -1, cv.CV_RGB(0,0,self.color_max), thickness=1, maxLevel=1)
                
                
            
            # Publish processed image
            if self.pubImageProcessed.get_num_connections() > 0:
                try:
                    #cv.Copy(self.matProcessed, self.matProcessed2)
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npProcessed), "passthrough")
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
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npBackground), "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageBackground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

            
            # Publish thresholded image
            if self.pubImageThreshold.get_num_connections() > 0:
                try:
                    self.npCamera = cv2.add(self.npThreshold, self.npForeground)
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npCamera), "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageThreshold.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

              
            # Publish foreground image
            if self.pubImageForeground.get_num_connections() > 0:
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npForeground), "passthrough")
                    image2.header.stamp = image.header.stamp
                    self.pubImageForeground.publish(image2)
                    del image2
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

              
            # Publish a special image for use in rviz.
            if self.pubImageRviz.get_num_connections() > 0:
                self.npProcessedFlip, = cv2.flip(self.npProcessed, 0)
                image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.npProcessedFlip), "passthrough")
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
  
