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
from scipy.cluster.vq import kmeans,vq
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
        self.matImageRect = None
        self.matBackground = None
        
        self.command = None # Valid values:  None, or 'establish_background'
        self.bEstablishBackground = False # Gets set to true when user requests to establish a new background automatically.
        
        # Messages
        self.camerainfo = None
        queue_size_images = rospy.get_param('tracking/queue_size_images', 1)
        
        #self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo, self.CameraInfo_callback)
        self.subImageRect           = rospy.Subscriber('camera/image_rect', Image, self.Image_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True)
        self.subImageBackgroundOriginate = rospy.Subscriber('camera/image_background_originate', Image, self.ImageBackgroundSet_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True) # The image from disk needs to come in on a different topic so it doesn't go directly into the bag file.
        self.subImageBackgroundSet  = rospy.Subscriber('camera/image_background_set', Image, self.ImageBackgroundSet_callback, queue_size=queue_size_images, buff_size=262144, tcp_nodelay=True)
        self.subTrackingCommand     = rospy.Subscriber('tracking/command', TrackingCommand, self.TrackingCommand_callback)
        
        self.pubImageProcessed      = rospy.Publisher('camera/image_processed', Image)
        self.pubImageBackground     = rospy.Publisher('camera/image_background', Image)
        self.pubImageBackgroundSet  = rospy.Publisher('camera/image_background_set', Image, latch=True) # We publish the current background image (at trial_start & trigger) primarily so that rosbag can record it.
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
        
        self.rcBackgroundEstablish = rospy.get_param('tracking/rcBackgroundEstablish', 1.0) # Time constant to use when establishing a new background automatically. 

        self.widthRoi  = rospy.get_param ('tracking/roi/width', 15)
        self.heightRoi = rospy.get_param ('tracking/roi/height', 15)
        
        # Contour Info
        self.contourinfolists = ContourinfoLists()
        self.contourinfolists_dict = {'x':[], 'y':[], 'angle':[], 'area':[], 'ecc':[], 'imgRoi':[], 'iContour':[]}
        self.nContours = 0
        self.nContoursPrev = 0
        
        self.nObjects = 0
        self.bMerged = False
        
        self.nContoursMax = rospy.get_param('tracking/nContoursMax', 20)
        self.areaContourMin = rospy.get_param('tracking/areaContourMin', 0.0)
        self.minEccForDisplay = 1.75
        self.minSumImage = 100
        self.distanceDuplicateContour = rospy.get_param('tracking/distanceDuplicateContour', 0.1)        
        if (1 < self.nContoursMax):
            self.nContoursMax -= 1
        
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)) #N.ones([3,3], dtype=N.uint8)

        
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
        self.ptsOriginArena.header.frame_id = '/Arena'
        self.ptsOriginArena.point.x = 0
        self.ptsOriginArena.point.y = 0
        

        # Mask Info
        self.radiusMask = int(rospy.get_param('camera/mask/radius', 9999))

        self.stampPrev = None#rospy.Time.now()
        
        self.services = {}
        self.services['tracking/init'] = rospy.Service('tracking/init',            ExperimentParams, self.Init_callback)
        self.services['tracking/trial_start'] = rospy.Service('tracking/trial_start',     ExperimentParams, self.TrialStart_callback)
        self.services['tracking/trial_end'] = rospy.Service('tracking/trial_end',       ExperimentParams, self.TrialEnd_callback)
        self.services['tracking/trigger'] = rospy.Service('tracking/trigger',         Trigger,          self.Trigger_callback)
        self.services['tracking/wait_until_done'] = rospy.Service('tracking/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)

        self.initConstructor = True
        

    def InitializeImages(self):
        if (self.header is not None) and (self.matImageRect is not None):

            # Initialize the images.
            self.matProcessed         = N.zeros([self.height, self.width, 3], dtype=N.uint8)
            self.matProcessedFlip     = N.zeros([self.height, self.width, 3], dtype=N.uint8)
            self.matMask              = N.zeros([self.height, self.width], dtype=N.uint8)
            self.matForeground        = N.zeros([self.height, self.width], dtype=N.uint8)
            self.matThreshold         = N.zeros([self.height, self.width], dtype=N.uint8)
            self.matMaskOthers        = N.zeros([self.height, self.width], dtype=N.uint8)
            
            
            if self.bUseTransforms:
                b = False
                while not b:
                    try:
                        self.tfrx.waitForTransform('/Arena', 
                                                   self.ptsOriginImage.header.frame_id, 
                                                   self.ptsOriginImage.header.stamp, 
                                                   rospy.Duration(1.0))
                    except tf.Exception, e:
                        rospy.logwarn('ExceptionA transforming mask frame %s->Arena:  %s' % (self.ptsOriginImage.header.frame_id, e))
                        
                    try:
                        self.ptsOriginMask = self.tfrx.transformPoint('/Arena', self.ptsOriginImage)
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
            
                
            self.initImages = True


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
                self.selfPublishedBackground = True
                self.pubImageBackgroundSet.publish(imgBackground)
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
                    self.selfPublishedBackground = True
                    self.pubImageBackgroundSet.publish(imgBackground)
                except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                    rospy.logwarn ('Exception %s' % e)

        return True
    
    def TrialEnd_callback(self, experimentparams):
        return True

    def WaitUntilDone_callback(self, experimentparams):
        return True


    def ImageBackgroundSet_callback (self, image):
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

        if (self.command=='initialize'):
            self.nRobots = trackingcommand.nRobots
            self.nFlies = trackingcommand.nFlies
            self.nObjects = self.nRobots + self.nFlies
        
        
    
    def MmFromPixels (self, xIn):
        response = self.camera_from_arena(xIn, xIn)
        return (response.xDst, response.yDst)
        
        
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
        

    def ContourinfoFromContour(self, contour):
        moments = cv2.moments(contour)  # Sometimes the contour contains just one pixel, resulting in moments=(0,0,0,0,0,0)

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
    # Converts contour to a contourinfo...
    # transforms it to the output frame...
    # computes the contour image minus the other contour images...
    # and appends the contourinfo members to their respective lists.
    #
    def AppendContourinfoListsFromContour(self, contour_list, iContour, matForeground):
        contour = contour_list[iContour]
        (x, y, area, angle, ecc) = self.ContourinfoFromContour(contour)
        if (x is None) or (y is None):
            (x,y) = contour[0][0]
            area = 0.0001 # one pixel's worth.
            angle = 99.9
            ecc = 1.0
        
            
        # Transform and Save contourinfo to the dict.
        if (x is not None):
            ptsContour = PointStamped()
            ptsContour.header.frame_id = 'ImageRect'
            ptsContour.point.x = x
            ptsContour.point.y = y
            
            try:
                if self.bUseTransforms:
                    self.ptsOutput = self.tfrx.transformPoint(self.frameidOutput, ptsContour)
                else:
                    self.ptsOutput = ptsContour
                    
            except tf.Exception, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))
                self.ptsOutput = PointStamped()
            except TypeError, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.frameidOutput, ptsContour.header.frame_id, e))

            else:
                self.contourinfolists_dict['x'].append(self.ptsOutput.point.x)
                self.contourinfolists_dict['y'].append(self.ptsOutput.point.y)
                self.contourinfolists_dict['angle'].append(angle)
                self.contourinfolists_dict['area'].append(area)
                self.contourinfolists_dict['ecc'].append(ecc)
                self.contourinfolists_dict['imgRoi'].append(None)
                self.contourinfolists_dict['iContour'].append(iContour)


            # Get the ROI pixels for each contour, minus the pixels of the other contours.
            if (matForeground is not None):
                nContours = len(self.contourinfolists_dict['x'])
                #for iContour in range(nContours):
                x = self.contourinfolists_dict['x'][iContour]
                y = self.contourinfolists_dict['y'][iContour]
    
                self.matMaskOthers.fill(0)
                
                # Go through all the other contours.
                normRoi = N.linalg.norm([self.widthRoi,self.heightRoi])
                for kContour in range(nContours):
                    if (kContour != iContour):
                        xk = self.contourinfolists_dict['x'][kContour]
                        yk = self.contourinfolists_dict['y'][kContour]
                        
                        # If the kth ROI can overlap with the ith ROI, then add that fly's pixels to the subtraction mask.  
                        if (N.linalg.norm([x-xk,y-yk]) < normRoi):
                            jContour = self.contourinfolists_dict['iContour'][kContour]
                            cv2.drawContours(self.matMaskOthers, contour_list, jContour, 255, cv.CV_FILLED, 4, self.hierarchy, 0)
                        
                self.matMaskOthers = cv2.dilate(self.matMaskOthers, self.kernel, iterations=2)
                matOthers = cv2.bitwise_and(self.matMaskOthers, matForeground)
                matRoi = cv2.getRectSubPix(matForeground - matOthers, 
                                           (self.widthRoi, self.heightRoi), 
                                           (x,y))
                imgRoi = self.cvbridge.cv_to_imgmsg(cv.fromarray(matRoi), 'passthrough')
                self.contourinfolists_dict['imgRoi'][iContour] = imgRoi
                

                
                

    def RemoveDupsAndTooSmallContours(self, contoursIn_list):
        contoursOut_list = []
        iContourLargest = None
        
        # Put the contours into the contourinfolists_dict.
        self.contourinfolists_dict = {'x':[], 'y':[], 'angle':[], 'area':[], 'ecc':[], 'imgRoi':[], 'iContour':[]}
        for iContour in range(len(contoursIn_list)):
            self.AppendContourinfoListsFromContour(contoursIn_list, iContour, None)

        # Put lists into contourinfolists.
        contourinfolists = ContourinfoLists()
        
            
        if (len(contoursIn_list) > 0):
            # Skip too-small contours, and repackage the data as a list of lists, i.e. [[x,y,a,a,e,i,i],[x,y,a,a,e,i,i],...]
            contourinfolists_list = []
            for iContour in range(len(contoursIn_list)):
                if (self.areaContourMin <= self.contourinfolists_dict['area'][iContour]):
                    contourinfolists_list.append([self.contourinfolists_dict['x'][iContour],        # 0 
                                                  self.contourinfolists_dict['y'][iContour],        # 1
                                                  self.contourinfolists_dict['angle'][iContour],    # 2
                                                  self.contourinfolists_dict['area'][iContour],     # 3
                                                  self.contourinfolists_dict['ecc'][iContour],      # 4
                                                  self.contourinfolists_dict['imgRoi'][iContour],   # 5
                                                  self.contourinfolists_dict['iContour'][iContour]])# 6

            # Remove the dups.
            contourinfolists_list = sorted(tuple(contourinfolists_list))
            contourinfolists_list = [x for i, x in enumerate(contourinfolists_list) if (not i) or (N.linalg.norm(N.array(contourinfolists_list[i][0:2])-N.array(contourinfolists_list[i-1][0:2])) > self.distanceDuplicateContour)]
        
        
            # Count the contours.
            self.nContours = len(contourinfolists_list)
            

            # Reconstruct the contour_list from the deduped data, also find the one with max area.
            area = -1
            for iContour in range(self.nContours):
                contoursOut_list.append(contoursIn_list[contourinfolists_list[iContour][6]]) # Append contours indexed from the original list.
                if (area < contourinfolists_list[iContour][3]):
                    iContourLargest = iContour
                    area = contourinfolists_list[iContour][3]

        
        return (contoursOut_list, iContourLargest)
    
    
    # Split a contour into two (or multiple possible subcontours).
    def SplitContour(self, contourIn, defects):
        contoursOut = []
        
        if (defects is not None):
            nDefects = len(defects)
            
            # Divide into two, based on kmeans clustering.
            if (nDefects<2):
                contourInB = contourIn.squeeze()
                centroids,_ = kmeans(contourInB, 2)    # Make two clusters.
                iAssignment,_ = vq(contourInB, centroids)          # Assign points to clusters.
                
                contoursOut.append(contourIn[iAssignment==0])
                contoursOut.append(contourIn[iAssignment==1])

            
            # Divide into two, at the line connecting the two greatest defects.
            if (nDefects==2):
                iBySize = N.argsort(defects[:,0,3]) # Sorted indices of defect size.
                
                # Get the two largest defects, in index order.
                j = min(defects[iBySize[-1],0,2], defects[iBySize[-2],0,2])
                k = max(defects[iBySize[-1],0,2], defects[iBySize[-2],0,2])
                
                # Keep the shared points.
                #contour1 = contourIn[j:k+1]
                #contour2 = N.concatenate((contourIn[0:j+1],contourIn[k:len(contourIn)+1]))
                
                # Don't keep the shared points.
                contoursOut.append(contourIn[j+1:k])
                contoursOut.append(N.concatenate((contourIn[0:j],contourIn[k+1:len(contourIn)+1])))
                #contour1 = contourIn[j+1:k]
                #contour2 = N.concatenate((contourIn[0:j],contourIn[k+1:len(contourIn)+1]))

                
            # Divide into three candidate contour pairs, and let the ContourIdentifier figure it out.
            if (nDefects>2):
                iBySize = N.argsort(defects[:,0,3]) # Indices sorted by defect size.
                
                # Indices of three largest defects, sorted by point location.
                iByLocation = N.array([defects[iBySize[-3],0,2], defects[iBySize[-2],0,2], defects[iBySize[-1],0,2]])
                iByLocation.sort()

                # Indices of three largest defects, in location order.
                j = iByLocation[0]
                k = iByLocation[1]
                m = iByLocation[2]

                # Three contours.
                #contoursOut.append(contourIn[j+1:k])
                #contoursOut.append(contourIn[k+1:m])
                #contoursOut.append(N.concatenate((contourIn[0:j],contourIn[m+1:len(contourIn)+1])))

                # Three pairs of contours.
                (p1, p2) = (j, k)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(N.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))

                (p1, p2) = (k, m)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(N.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))
                
                (p1, p2) = (j, m)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(N.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))
                
                
        else:
            contourInB = contourIn.squeeze()
            centroids,_ = kmeans(contourInB, 2)    # Make two clusters.
            iAssignment,_ = vq(contourInB, centroids)          # Assign points to clusters.
            
            contoursOut.append(contourIn[iAssignment==0])
            contoursOut.append(contourIn[iAssignment==1])

        return contoursOut #(contour1,contour2)
        
        
    
    def ContourinfoListsFromImage(self, matThreshold, matForeground):
        # Find contours
        sumImage = cv2.sumElems(matThreshold)
        if self.minSumImage < sumImage[0]:
            (contours,hierarchy) = cv2.findContours(matThreshold, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_NONE )#cv2.CHAIN_APPROX_SIMPLE) # Modifies matThreshold.
        else:
            (contours,hierarchy) = (None, None)
            

        # Make a list of the top-level contours.
        self.contour_list = []
        if (contours is not None):
            (NEXT,PREV,CHILD,PARENT)=(0,1,2,3)
            iContour = 0
            while (0 <= iContour < len(contours)): 
                self.contour_list.append(contours[iContour])
                iContour = hierarchy[0][iContour][NEXT]

            
        # Clean the contours.
        (self.contour_list, iContourLargest) = self.RemoveDupsAndTooSmallContours(self.contour_list)
        self.nContours = len(self.contour_list)

            

        if (self.nContours > 0):

            # Determine if two contours have merged, and need to be split.
            iContourMerged = iContourLargest
            
            if (self.nContours < self.nObjects) and (iContourMerged is not None):
                self.bMerged = True
            else:
                self.bMerged = False
                

            # Split the merged contour.
            if self.bMerged:
                contour = self.contour_list.pop(iContourMerged)
                hull = cv2.convexHull(contour, returnPoints=False)
                defects = cv2.convexityDefects(contour, hull) 

                self.contour_list.extend(self.SplitContour(contour, defects))
                self.nContours = len(self.contour_list)
                

        # Make the hierarchy for self.contour_list.
        self.hierarchy = [[]]
        for iContour in range(len(self.contour_list)):
            self.hierarchy[0].append([(iContour+1 if iContour+1<len(self.contour_list) else -1),   # NEXT 
                                      (iContour-1 if iContour-1>=0 else -1),                       # PREV
                                      -1,                                                          # CHILD
                                      -1])                                                         # PARENT
        self.hierarchy = N.array(self.hierarchy)
                                         

        # Put the contours into the contourinfolists_dict.
        self.contourinfolists_dict = {'x':[], 'y':[], 'angle':[], 'area':[], 'ecc':[], 'imgRoi':[], 'iContour':[]}
        for iContour in range(self.nContours):
            self.AppendContourinfoListsFromContour(self.contour_list, iContour, matForeground)
                
                    
        
        # Put lists into a ContourinfoLists message.
        contourinfolists = ContourinfoLists()
        contourinfolists.header.seq = self.seq
        contourinfolists.header.stamp = self.header.stamp
        contourinfolists.header.frame_id = self.frameidOutput # i.e. Camera
        self.seq += 1
        if self.nContours > 0:
            contourinfolists.x = self.contourinfolists_dict['x']
            contourinfolists.y = self.contourinfolists_dict['y']
            contourinfolists.angle = self.contourinfolists_dict['angle']
            contourinfolists.area = self.contourinfolists_dict['area']
            contourinfolists.ecc = self.contourinfolists_dict['ecc']
            contourinfolists.imgRoi = self.contourinfolists_dict['imgRoi']
        
            
        self.nContoursPrev = self.nContours
    
        return contourinfolists  
        

    def Image_callback(self, image):
        try:
            #rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
            #self.timePrev = rospy.Time.now().to_sec()
            
            if not self.initConstructor:
                return
    
            if (self.stampPrev is not None):
                self.dt = image.header.stamp - self.stampPrev
            else:
                self.dt = rospy.Time(0)
                
            self.stampPrev = image.header.stamp
            
    
            self.header = image.header
            self.height = image.height
            self.width = image.width
            # Convert ROS image to OpenCV image.
            try:
                self.matImageRect = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
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
                    self.matMask.fill(0)
                    cv2.circle(self.matMask,
                              (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                              int(self.radiusMask), 
                              self.color_max, 
                              cv.CV_FILLED)
                
                # Normalize the histogram.
                if self.bEqualizeHist:
                    self.matImageRect = cv2.equalizeHist(self.matImageRect)
                
                
                # Update the background.
                #rospy.logwarn('types: %s' % [type(N.float32(self.matImageRect)), type(self.matfBackground), type(self.rcBackground)])
                if (self.bEstablishBackground):
                    rcBackground = self.rcBackgroundEstablish
                else:
                    rcBackground = rospy.get_param('tracking/rcBackground', 2500) # Time constant for moving average background.
                
                alphaBackground = 1 - N.exp(-self.dt.to_sec() / rcBackground)
                
                cv2.accumulateWeighted(N.float32(self.matImageRect), self.matfBackground, alphaBackground)
                self.matBackground = N.uint8(self.matfBackground)
    
    
                # Create the foreground.
                if self.bEqualizeHist:
                    matBackgroundEq = cv2.equalizeHist(self.matBackground)
    
    
                if (self.bUseBackgroundSubtraction) and (self.matBackground is not None):
                    self.matForeground = cv2.absdiff(self.matImageRect, self.matBackground)
                else:
                    self.matForeground = self.matImageRect
    
                
                # Threshold
                (threshold,self.matThreshold) = cv2.threshold(self.matForeground, 
                                                 self.diff_threshold, 
                                                 self.max_8U, 
                                                 cv2.THRESH_TOZERO)
    
                
                # Apply the arena mask to the threshold image.
                self.matThreshold = cv2.bitwise_and(self.matThreshold, self.matMask)
                
                # Get the ContourinfoLists.
                self.contourinfolists = self.ContourinfoListsFromImage(self.matThreshold, self.matForeground)    # Modifies self.matThreshold
                self.pubContourinfoLists.publish(self.contourinfolists)
                
                # Convert to color for display image
                if self.pubImageProcessed.get_num_connections() > 0:
                    # Apply the arena mask to the camera image, and convert it to color.
                    self.matImageRect = cv2.bitwise_and(self.matImageRect, self.matMask)
                    self.matProcessed = cv2.cvtColor(self.matImageRect, cv.CV_GRAY2RGB)
                    
                    # Draw contours on Processed image.
                    colors = [cv.CV_RGB(0,              0,              self.color_max), # blue
                              cv.CV_RGB(0,              self.color_max, 0             ), # green
                              cv.CV_RGB(self.color_max, 0,              0             ), # red
                              cv.CV_RGB(self.color_max, self.color_max, 0             ), # yellow
                              cv.CV_RGB(self.color_max, 0,              self.color_max), # magenta
                              cv.CV_RGB(0,              self.color_max, self.color_max)] # cyan
                    if self.contour_list:
                        for k in range(len(self.contour_list)):
                            #color = cv.CV_RGB(0,0,self.color_max)
                            color = colors[k % len(colors)]
                            cv2.drawContours(self.matProcessed, self.contour_list, k, color, thickness=1, maxLevel=1)
                    
                
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
                        self.matImageRect = cv2.add(self.matThreshold, self.matForeground)
                        image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matImageRect), 'passthrough')
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
                    image2.header.frame_id = '/Arena'
                    image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
                    
                    camerainfo2 = CameraInfo()#copy.copy(self.camerainfo)
                    camerainfo2.header = self.header
                    camerainfo2.header.frame_id = '/Arena'
                    camerainfo2.height = self.height
                    camerainfo2.width = self.width
                    k11 = rospy.get_param('/k11', 1.0)
                    k13 = rospy.get_param('/k13', 0.0)
                    k22 = rospy.get_param('/k22', 1.0)
                    k23 = rospy.get_param('/k23', 0.0)
                    k33 = rospy.get_param('/k33', 1.0)
                    p11 = rospy.get_param('/p11', -0.023)
                    p13 = rospy.get_param('/p13', 0.0)
                    p22 = rospy.get_param('/p22', +0.023)
                    p23 = rospy.get_param('/p23', 0.0)
                    p33 = rospy.get_param('/p33', 1.0)
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
                    self.nImagesContoursEstablished += 1
                else:
                    self.nImagesContoursEstablished = 0
                  
                # Stabilized when more than two time-constants worth of background.  
                if (self.bEstablishBackground) and (self.nImagesContoursEstablished > 2*(rcBackground/self.dt.to_sec())):
                    rospy.logwarn('establish_background Finished.  Click <Save Background Image> if it\'s acceptable.')
                    self.bEstablishBackground = False
                    self.nImagesContoursEstablished = 0
        except rospy.exceptions.ROSException, e:
            rospy.loginfo('CG ROSException: %s' % e)


    def Main(self):
        rospy.spin()

#         # Shutdown all the services we offered.
#         for key in self.services:
#             self.services[key].shutdown()
        
      

def main(args):
    rospy.init_node('ContourGenerator') #, anonymous=True)
    try:
        cg = ContourGenerator()
        cg.Main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
  
