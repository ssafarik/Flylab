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
import threading
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from arena_tf.srv import ArenaCameraConversion
from flycore.msg import TrackingCommand
from experiment_srvs.srv import Trigger, ExperimentParams
from tracking.msg import ContourinfoLists




###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourinfoLists message, and several intermediary images.
#
class ContourGenerator:

    def __init__(self):
        
        # Image Initialization
        self.initConstructor = False
        self.initBackground = False
        self.initImages = False
        self.initialized = False
        
        self.lock = threading.Lock()
        
        self.bUseBackgroundSubtraction  = rospy.get_param('tracking/usebackgroundsubtraction', True)    # Set False to turn off bg subtraction.        
        self.bUseTransforms             = rospy.get_param('tracking/usetransforms', True)               # Set False to stay in camera coordinates.
        self.bEqualizeHist = False
        self.header = None
        self.matCamera = None
        self.matBackground = None
        
        self.command = None # Valid values:  None, or 'establish_background'
        self.bEstablishBackground = False # Gets set to true when user requests to establish a new background automatically.
        
        # Messages
        self.camerainfo = None
        queue_size_images = rospy.get_param('tracking/queue_size_images', 1)
        
        #self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo, self.CameraInfo_callback)
        self.subImageRect           = rospy.Subscriber('camera/image_rect', Image, self.Image_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True)
        self.subImageBackgroundFile = rospy.Subscriber('camera/image_backgroundfile', Image, self.ImageBackgroundInit_callback, queue_size=1, buff_size=262144, tcp_nodelay=True) # The image from disk needs to come in on a different topic so it doesn't go directly into the bag file.
        self.subImageBackgroundInit = rospy.Subscriber('camera/image_backgroundinit', Image, self.ImageBackgroundInit_callback, queue_size=1, buff_size=262144, tcp_nodelay=True)
        self.subTrackingCommand     = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        
        self.pubTrackingCommand     = rospy.Publisher('tracking/command', TrackingCommand)
        self.pubImageProcessed      = rospy.Publisher('camera/image_processed', Image)
        self.pubImageBackground     = rospy.Publisher('camera/image_background', Image)
        self.pubImageBackgroundInit = rospy.Publisher('camera/image_backgroundinit', Image, latch=True) # We publish the current background image (at trial_start & trigger) primarily so that rosbag can record it.
        self.pubImageForeground     = rospy.Publisher('camera/image_foreground', Image)
        self.pubImageThreshold      = rospy.Publisher('camera/image_threshold', Image)
        self.pubImageRoi            = rospy.Publisher('camera/image_roi', Image)
        self.pubImageRviz           = rospy.Publisher('camera_rviz/image_rect', Image)
        self.pubCamerainfoRviz      = rospy.Publisher('camera_rviz/camera_info', CameraInfo)
        self.pubContourinfoLists    = rospy.Publisher('ContourinfoLists', ContourinfoLists)
        self.seq = 0
        self.selfPublishedBackground = False
        
        if self.bUseTransforms:
            self.tfrx = tf.TransformListener()
        
        self.alphaBackground          = rospy.get_param('tracking/alphaBackground', 0.01) # Alpha value for moving average background.
        self.alphaBackgroundEstablish = rospy.get_param('tracking/alphaBackgroundEstablish', 0.05) # Alpha value to use when establishing a new background automatically. 

        self.widthRoi  = rospy.get_param ('tracking/roi/width', 15)
        self.heightRoi = rospy.get_param ('tracking/roi/height', 15)
        
        # Contour Info
        self.contourinfolists = ContourinfoLists()
        self.x_list = []
        self.y_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        self.imgRoi_list = []
        self.nContours = 0
        self.nContoursMax = rospy.get_param('tracking/nContoursMax', 20)
        self.areaContourMin = rospy.get_param('tracking/areaContourMin', 0.0)
        self.minEccForDisplay = 1.75
        self.minSumImage = 100
        self.distanceDuplicateContour = rospy.get_param('tracking/distanceDuplicateContour', 0.1)        
        if (1 < self.nContoursMax):
            self.nContoursMax -= 1
        
        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.cvbridge = CvBridge()
        
        # Coordinate Systems
        self.frameidOutput = rospy.get_param('frameid_contours', 'ImageRect')
        
        self.ptsOriginImage = PointStamped()
        self.ptsOriginImage.header.frame_id = 'ImageRect'
        self.ptsOriginImage.point.x = 0
        self.ptsOriginImage.point.y = 0
        
        self.ptsOriginArena = PointStamped()
        self.ptsOriginArena.header.frame_id = 'Arena'
        self.ptsOriginArena.point.x = 0
        self.ptsOriginArena.point.y = 0
        

        # Mask Info
        self.radiusMask = int(rospy.get_param('camera/mask/radius', 9999))

        self.timePrev = rospy.Time.now().to_sec()
        
        rospy.Service('tracking/init',            ExperimentParams, self.Init_callback)
        rospy.Service('tracking/trial_start',     ExperimentParams, self.TrialStart_callback)
        rospy.Service('tracking/trial_end',       ExperimentParams, self.TrialEnd_callback)
        rospy.Service('tracking/trigger',         Trigger,          self.Trigger_callback)
        rospy.Service('tracking/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)

        self.initConstructor = True
        

    def InitializeImages(self):
        if (self.header is not None) and (self.matCamera is not None):

            # Initialize the images.
            self.matProcessed         = N.zeros([self.height, self.width, 3], dtype=N.uint8)
            self.matProcessedFlip     = N.zeros([self.height, self.width, 3], dtype=N.uint8)
            self.matMask              = N.zeros([self.height, self.width], dtype=N.uint8)
            self.matForeground        = N.zeros([self.height, self.width], dtype=N.uint8)
            self.matThreshold         = N.zeros([self.height, self.width], dtype=N.uint8)
            
            
            if self.bUseTransforms:
                b = False
                while not b:
                    try:
                        self.tfrx.waitForTransform('Arena', 
                                                   self.ptsOriginImage.header.frame_id, 
                                                   self.ptsOriginImage.header.stamp, 
                                                   rospy.Duration(1.0))
                    except tf.Exception, e:
                        rospy.logwarn('ExceptionA transforming mask frame %s->Arena:  %s' % (self.ptsOriginImage.header.frame_id, e))
                        
                    try:
                        self.ptsOriginMask = self.tfrx.transformPoint('Arena', self.ptsOriginImage)
                        self.ptsOriginMask.point.x = -self.ptsOriginMask.point.x
                        self.ptsOriginMask.point.y = -self.ptsOriginMask.point.y
                        b = True
                    except tf.Exception, e:
                        rospy.logwarn('ExceptionB transforming mask frame %s->Arena:  %s' % (self.ptsOriginImage.header.frame_id, e))
            else:
                self.ptsOriginMask = PointStamped(point=Point(x=0, y=0))
                        
            cv2.circle(self.matMask,
                      (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            
                
            self.initialized = True


    def Init_callback(self, experimentparams):
        return True

    # TrialStart_callback()
    # Publishes the current background image.
    #
    def TrialStart_callback(self, experimentparams):
        while (not self.initialized):
            rospy.sleep(0.5)

            
        # Publish the current background image.
        if (self.matBackground is not None):#(self.pubImageBackground.get_num_connections() > 0) and (self.matBackground is not None):
            try:
                imgBackground = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matBackground), 'passthrough')
                imgBackground.header.stamp = self.header.stamp
                rospy.logwarn ('TrialStart_callback publish()')
                self.pubImageBackgroundInit.publish(imgBackground)
                self.selfPublishedBackground = True
            except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                rospy.logwarn ('Exception %s' % e)
            
        return True
                
                
    # Trigger_callback() 
    # Publishes the current background image.
    #
    def Trigger_callback(self, reqTrigger):
        while (not self.initialized):
            rospy.sleep(0.5)

        # Publish the current background image.
        if (reqTrigger.triggered):
            if (self.matBackground is not None):#(self.pubImageBackground.get_num_connections() > 0) and (self.matBackground is not None):
                try:
                    imgBackground = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matBackground), 'passthrough')
                    imgBackground.header.stamp = self.header.stamp
                    rospy.logwarn ('Trigger_callback publish()')
                    self.pubImageBackgroundInit.publish(imgBackground)
                    self.selfPublishedBackground = True
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

        return True
    
    def TrialEnd_callback(self, experimentparams):
        return True

    def WaitUntilDone_callback(self, experimentparams):
        return True


    def ImageBackgroundInit_callback (self, image):
        while (not self.initConstructor):
            rospy.loginfo('Waiting for initConstructor.')
            rospy.sleep(0.1)
        
            
        if (not self.selfPublishedBackground):
            try:
                self.matBackground = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
            except CvBridgeError, e:
                rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
                self.matBackground = None
            else:
                self.matfBackground = N.float32(self.matBackground)
                self.initBackground = True
        else:
            self.selfPublishedBackground = False
        
        
    def CameraInfo_callback (self, msgCameraInfo):
        #with self.lock:
            if self.initConstructor:
                self.camerainfo = msgCameraInfo
                
                if not self.initialized:
                    self.InitializeImages()
            
                if (self.initImages and self.initBackground):
                    self.initialized = True
            
    # TrackingCommand_callback()
    # Receives commands to change the tracking behavior.
    # See TrackingCommand.msg for details.
    #
    def TrackingCommand_callback(self, trackingcommand):
        self.command = trackingcommand.command
        if (self.command=='establish_background'):
            self.bEstablishBackground = True
            self.nContoursEstablish = int(trackingcommand.param)
            rospy.logwarn('establish_background started...')
        
    
    def MmFromPixels (self, xIn):
        response = self.camera_from_arena(xIn, xIn)
        return (response.xDst, response.yDst)
        
        
    def PixelsFromMm (self, xIn):
        if self.bUseTransforms:
            ptsIn = PointStamped()
            ptsIn.header.frame_id = 'Arena'
            ptsIn.point.x = 0.0
            ptsIn.point.y = 0.0
            ptsOrigin = self.tfrx.transformPoint('ImageRect', ptsIn)        
            
            ptsIn.point.x = xIn
            ptsIn.point.y = xIn
            ptsOut = self.tfrx.transformPoint('ImageRect', ptsIn)
            xOut = (ptsOut.point.x - ptsOrigin.point.x)
            
        else:
            xOut = xIn 
                
        return xOut
        
        
    def DrawAngleLine(self, matImage, x0, y0, angle, ecc, length):
        if (not N.isnan(angle)) and (self.minEccForDisplay < ecc):
            height, width = matImage.size
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
            
            
            cv.Line(matImage, 
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
        m00 = moments['m00']
        m10 = moments['m10']
        m01 = moments['m01']
        
        if m00 != 0.0:
            x = m10/m00
            y = m01/m00
        
        else: # There was just one pixel in the contour.
            x = None
            y = None
            #rospy.logwarn ('zero moments')
          
        u11 = moments['mu11']
        u20 = moments['mu20']
        u02 = moments['mu02']
        area = m00
        angle, ecc = self.FindAngleEcc(u20, u11, u02)
        #rospy.logwarn('u: %s, %s, %s, %s' % (m00, u20, u11, u02))

#        rospy.logwarn ('Moments: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, xy=(%s, %s)' % (m00, m10, m01, u20, u11, u02, x, y))

        return (x, y, area, angle, ecc)
    
    
    # AppendContourinfoListsFromContour()
    # Converts contour to a contourinfo, transforms it to the output frame, and appends the contourinfo members to their respective lists.
    #
    def AppendContourinfoListsFromContour(self, contour, matForeground):
        moments = cv2.moments(contour)  # Sometimes the contour contains just one pixel, resulting in moments=(0,0,0,0,0,0)
        (x, y, area, angle, ecc) = self.ContourinfoFromMoments(moments)
        if (x is None) or (y is None):
            (x,y) = contour[0][0]
            area = 0.0001 # one pixel's worth.
            angle = 99.9
            ecc = 1.0
        
            
        # Save contourinfolists
        ptsContour = PointStamped()
        ptsContour.header.frame_id = 'ImageRect'
        ptsContour.point.x = x
        ptsContour.point.y = y
            
        if x is not None:
            imgRoi = self.cvbridge.cv_to_imgmsg(cv.fromarray(cv2.getRectSubPix(matForeground, (self.widthRoi, self.heightRoi), (x,y))), 'passthrough')
            #self.pubImageRoi.publish(imgRoi)
            try:
                if self.bUseTransforms:
                    self.ptsOutput = self.tfrx.transformPoint(self.frameidOutput, ptsContour)
                else:
                    self.ptsOutput = ptsContour
                    
                self.x_list.append(self.ptsOutput.point.x)
                self.y_list.append(self.ptsOutput.point.y)
                self.angle_list.append(angle)
                self.area_list.append(area)
                self.ecc_list.append(ecc)
                self.imgRoi_list.append(imgRoi)
                self.nContours += 1

            except tf.Exception, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))
                self.ptsOutput = PointStamped()
            except TypeError, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))
                
        


    def ContourinfoListsFromImage(self, matThreshold, matForeground):
        self.x_list = []
        self.y_list = []
        self.angle_list = []
        self.area_list = []
        self.ecc_list = []
        self.imgRoi_list = []
        
        # Find contours
        sumImage = cv2.sumElems(matThreshold)
        if self.minSumImage < sumImage[0]:
            (contours,hierarchy) = cv2.findContours(matThreshold, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE) # Modifies matThreshold.
        else:
            (contours,hierarchy) = (None, None)
            

        # Put the top-level contours into the contourinfolists
        self.nContours = 0
        if contours is not None:
            (NEXT,PREV,CHILD,PARENT)=(0,1,2,3)
            iContour = 0
            while (0<=iContour) and (iContour<length(contours)): 
                contour = contours[iContour]
                self.AppendContourinfoListsFromContour(contour, matForeground) # self.nContours++ gets incremented inside function.
                iContour = hierarchy[0][iContour][NEXT]
                    
        
        # Put lists into contourinfolists.
        contourinfolists = ContourinfoLists()
        contourinfolists.header.seq = self.seq
        contourinfolists.header.stamp = self.header.stamp
        contourinfolists.header.frame_id = self.frameidOutput # i.e. Camera
        self.seq += 1
        
        if self.nContours > 0:
            contourinfolists.x = self.x_list
            contourinfolists.y = self.y_list
            contourinfolists.angle = self.angle_list
            contourinfolists.area = self.area_list
            contourinfolists.ecc = self.ecc_list
            contourinfolists.imgRoi = self.imgRoi_list
        
            
        # Remove duplicates and too-small contours.
        if self.nContours > 0:
            # Repackage the data as a list of lists, i.e. [[x,y,a,a,e],[x,y,a,a,e],...], skipping too-small contours.
            contourinfolists_list = []
            for iContour in range(self.nContours):
                if (self.areaContourMin <= contourinfolists.area[iContour]):
                    contourinfolists_list.append([contourinfolists.x[iContour], 
                                                 contourinfolists.y[iContour], 
                                                 contourinfolists.angle[iContour], 
                                                 contourinfolists.area[iContour], 
                                                 contourinfolists.ecc[iContour], 
                                                 contourinfolists.imgRoi[iContour]])

            # Remove the dups.
            contourinfolists_list = sorted(tuple(contourinfolists_list))
            contourinfolists_list = [x for i, x in enumerate(contourinfolists_list) if (not i) or (N.linalg.norm(N.array(contourinfolists_list[i][0:2])-N.array(contourinfolists_list[i-1][0:2])) > self.distanceDuplicateContour)]
        
            # Repackage the de-duped data.
            self.nContours = len(contourinfolists_list)
            contourinfolists.x = []
            contourinfolists.y = []
            contourinfolists.angle = []
            contourinfolists.area = []
            contourinfolists.ecc = []
            contourinfolists.imgRoi = []
            for iContour in range(self.nContours):
                contourinfolists.x.append(contourinfolists_list[iContour][0])
                contourinfolists.y.append(contourinfolists_list[iContour][1])
                contourinfolists.angle.append(contourinfolists_list[iContour][2])
                contourinfolists.area.append(contourinfolists_list[iContour][3])
                contourinfolists.ecc.append(contourinfolists_list[iContour][4])
                contourinfolists.imgRoi.append(contourinfolists_list[iContour][5])

            
        return contourinfolists, contours    
        

    def Image_callback(self, image):
        #rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
        #self.timePrev = rospy.Time.now().to_sec()

        if not self.initConstructor:
            return

        self.header = image.header
        self.height = image.height
        self.width = image.width
        # Convert ROS image to OpenCV image.
        try:
            self.matCamera = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
        except CvBridgeError, e:
            rospy.logwarn ('Exception converting ROS image to opencv:  %s' % e)
        
        if not self.initImages:
            self.InitializeImages()
            
        if (self.initImages and self.initBackground):
            self.initialized = True
            

        if self.initialized:        
            # Check for new diff_threshold value
            self.diff_threshold = rospy.get_param('tracking/diff_threshold', 50)
            
            # Create the mask.
            radiusMask = int(rospy.get_param('camera/mask/radius', 9999)) 
            if radiusMask != self.radiusMask:
                self.radiusMask = radiusMask 
                self.matMask = N.zeros([self.height, self.width], dtype=N.uint8)
                cv2.circle(self.matMask,
                          (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                          int(self.radiusMask), 
                          self.color_max, 
                          cv.CV_FILLED)
            
            # Normalize the histogram.
            if self.bEqualizeHist:
                self.matCamera = cv2.equalizeHist(self.matCamera)
            
            
            # Update the background.
            #rospy.logwarn('types: %s' % [type(N.float32(self.matCamera)), type(self.matfBackground), type(self.alphaBackground)])
            if (self.bEstablishBackground):
                self.alphaBackground = self.alphaBackgroundEstablish
            else:
                self.alphaBackground = rospy.get_param('tracking/alphaBackground', 0.00001) # Alpha value for moving average background.

            cv2.accumulateWeighted(N.float32(self.matCamera), self.matfBackground, self.alphaBackground)
            self.matBackground = N.uint8(self.matfBackground)

            # Create the foreground.
            if self.bEqualizeHist:
                matBackgroundEq = cv2.equalizeHist(self.matBackground)


            if (self.bUseBackgroundSubtraction) and (self.matBackground is not None):
                self.matForeground = cv2.absdiff(self.matCamera, self.matBackground)
            else:
                self.matForeground = self.matCamera

            
            # Threshold
            (threshold,self.matThreshold) = cv2.threshold(self.matForeground, 
                                             self.diff_threshold, 
                                             self.max_8U, 
                                             cv2.THRESH_TOZERO)

            
            # Mask the threshold image.
            self.matThreshold = cv2.bitwise_and(self.matThreshold, self.matMask)
            
            # Get the ContourinfoLists.
            (self.contourinfolists, self.contours) = self.ContourinfoListsFromImage(self.matThreshold, self.matForeground)    # Modifies self.matThreshold
            self.pubContourinfoLists.publish(self.contourinfolists)
            
            # Convert to color for display image
            if self.pubImageProcessed.get_num_connections() > 0:
                # Mask the camera image, and convert it to color.
                self.matCamera = cv2.bitwise_and(self.matCamera, self.matMask)
                self.matProcessed = cv2.cvtColor(self.matCamera, cv.CV_GRAY2RGB)
                
                # Draw contours on Processed image.
                if self.contours:
                    cv2.drawContours(self.matProcessed, self.contours, -1, cv.CV_RGB(0,0,self.color_max), thickness=1, maxLevel=1)
                
            
            # Publish processed image
            if self.pubImageProcessed.get_num_connections() > 0:
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matProcessed), 'passthrough')
                    image2.header = image.header
                    image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
                    #rospy.logwarn(image2.encoding)
                    self.pubImageProcessed.publish(image2)
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)
            
            # Publish background image
            if (self.pubImageBackground.get_num_connections() > 0) and (self.matBackground is not None):
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matBackground), 'passthrough')
                    image2.header.stamp = image.header.stamp
                    self.pubImageBackground.publish(image2)
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

            
            # Publish thresholded image
            if self.pubImageThreshold.get_num_connections() > 0:
                try:
                    self.matCamera = cv2.add(self.matThreshold, self.matForeground)
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matCamera), 'passthrough')
                    image2.header.stamp = image.header.stamp
                    self.pubImageThreshold.publish(image2)
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

              
            # Publish foreground image
            if self.pubImageForeground.get_num_connections() > 0:
                try:
                    image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matForeground), 'passthrough')
                    image2.header.stamp = image.header.stamp
                    self.pubImageForeground.publish(image2)
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

              
            # Publish a special image for use in rviz.
            if self.pubImageRviz.get_num_connections() > 0:
                self.matProcessedFlip = cv2.flip(self.matProcessed, 0)
                image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matProcessedFlip), 'passthrough')
                image2.header = image.header
                image2.header.frame_id = 'Arena'
                image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
                
                camerainfo2 = CameraInfo()#copy.copy(self.camerainfo)
                camerainfo2.header = self.header
                camerainfo2.header.frame_id = 'Arena'
                camerainfo2.height = self.height
                camerainfo2.width = self.width
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
                

            # When automatically establishing a new background, determine if the background has stabilized.                
            if (self.bEstablishBackground) and (self.nContours==self.nContoursEstablish):
                self.nContoursEstablished += 1
            else:
                self.nContoursEstablished = 0
              
            # Stabilized when more than three time-constants worth of background.  
            if (self.nContoursEstablished > 3/self.alphaBackgroundEstablish):
                #self.pubTrackingCommand.publish(TrackingCommand(command='save_background')) # Let the user manually save it when they're happy with it.
                rospy.logwarn('establish_background Finished.  Click <Save Background Image> if it\'s acceptable.')
                self.bEstablishBackground = False
                self.nContoursEstablished = 0



    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('ContourGenerator') #, anonymous=True)
    try:
        cg = ContourGenerator()
        cg.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
  
