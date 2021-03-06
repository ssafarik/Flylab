#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import cv2
import copy
import tf
import numpy as np
from scipy.cluster.vq import kmeans,vq
import threading
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from arena_tf.srv import ArenaCameraConversion
from flycore.msg import TrackingCommand
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsChoices
from tracking.msg import ContourinfoLists
from pythonmodules import SetDict
import cProfile



###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourinfoLists message, and several intermediary images.
#
class ContourGenerator:

    def __init__(self):
        
        # Image Initialization
        self.bInitConstructor = False
        self.bInitBackground = False
        self.bInitImages = False
        self.bInitialized = False
        
        self.lock = threading.Lock()
        self.lockBuffer = threading.Lock()

        nQueue              = 2
        self.bufferImages   = [None]*nQueue # Circular buffer for incoming images.
        self.iImgLoading    = 0  # Index of the next slot to load.
        self.iImgWorking    = 0  # Index of the slot to process, i.e. the oldest image in the buffer.

        self.bEqualizeHist = False
        self.header = None
        self.matImageRect = None
        self.matBackground = None
        
        self.command = None # Valid values:  None, or 'establish_background'
        self.bEstablishBackground = False # Gets set to true when user requests to establish a new background automatically.
        

        # Read all the params.
        self.params = rospy.get_param('/', {})
        defaults = {'camera':{'mask':{'x':0, 
                                      'y':0,
                                      'radius':9999}},
                    'tracking':{'diff_threshold':20,
                                'usebackgroundsubtraction': True,
                                'usetransforms': True,
                                'queue_size_images':1,
                                'rcBackgroundEstablish':1.0,
                                'rcBackground':2500.0,
                                'roi':{'width':15,
                                       'height':15},
                                'nContoursMax':20,
                                'distanceDuplicateContour':0.1,
                                'distanceMergedContours':40.0,
                                'frameid_contours':'ImageRect'}
                    }
        SetDict.SetWithPreserve(self.params, defaults)
        self.tParamsPrev = rospy.Time.now()
        

        # Messages
        self.camerainfo = None
        
        self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo, self.CameraInfo_callback)
        self.subImageRect           = rospy.Subscriber('camera/image_rect', Image, self.Image_callback, queue_size=self.params['tracking']['queue_size_images'], buff_size=262144, tcp_nodelay=True)
        
        # The background image from disk needs to come in on a different topic than the background image from a replayed bag file, i.e. "camera/image_background_originate" so it
        # doesn't go directly into the bag file that's being recorded.
        self.subImageBackgroundOriginate = rospy.Subscriber('camera/image_background_originate', Image, self.ImageBackgroundSet_callback, queue_size=self.params['tracking']['queue_size_images'], buff_size=262144, tcp_nodelay=True) 
        self.subImageBackgroundSet       = rospy.Subscriber('camera/image_background_set',       Image, self.ImageBackgroundSet_callback, queue_size=self.params['tracking']['queue_size_images'], buff_size=262144, tcp_nodelay=True)
        self.subTrackingCommand          = rospy.Subscriber('tracking/command',                  TrackingCommand, self.TrackingCommand_callback)
        
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
        
        if (self.params['tracking']['usetransforms']):
            self.tfrx = tf.TransformListener()
            rospy.sleep(1)
        

        
        # Contour Info
        self.contourinfolists = ContourinfoLists()
        self.nContours = 0
        self.nContoursPrev = 0
        
        self.nObjects = 0
        
        self.minEccForDisplay = 1.75
        self.minSumImage = 100
        if (1 < self.params['tracking']['nContoursMax']):
            self.params['tracking']['nContoursMax'] -= 1
        
        self.rClosestPrev = 55555
        self.rClosest = 55555
        self.bMerged = False
        
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)) #np.ones([3,3], dtype=np.uint8)

        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.cvbridge = CvBridge()
        
        # Coordinate Systems
        
        self.ptsOriginImage = PointStamped()
        self.ptsOriginImage.header.frame_id = 'ImageRect'
        self.ptsOriginImage.point.x = 0
        self.ptsOriginImage.point.y = 0
        
        self.ptsOriginArena = PointStamped()
        self.ptsOriginArena.header.frame_id = 'Arena'
        self.ptsOriginArena.point.x = 0
        self.ptsOriginArena.point.y = 0
        

        self.stampPrev = None#rospy.Time.now()
        
        self.services = {}
        self.services['tracking/init']              = rospy.Service('tracking/init',            ExperimentParamsChoices, self.Init_callback)
        self.services['tracking/trial_start']       = rospy.Service('tracking/trial_start',     ExperimentParams,        self.TrialStart_callback)
        self.services['tracking/trial_end']         = rospy.Service('tracking/trial_end',       ExperimentParams,        self.TrialEnd_callback)
        self.services['tracking/trigger']           = rospy.Service('tracking/trigger',         Trigger,                 self.Trigger_callback)
        self.services['tracking/wait_until_done']   = rospy.Service('tracking/wait_until_done', ExperimentParamsChoices, self.WaitUntilDone_callback)

        self.bInitConstructor = True
        

    def InitializeImages(self):
        if (self.header is not None) and (self.matImageRect is not None):

            # Initialize the images.
            self.matProcessed         = np.zeros([self.height, self.width, 3], dtype=np.uint8)
            self.matProcessedFlip     = np.zeros([self.height, self.width, 3], dtype=np.uint8)
            self.matMask              = np.zeros([self.height, self.width], dtype=np.uint8)
            self.matForeground        = np.zeros([self.height, self.width], dtype=np.uint8)
            self.matThreshold         = np.zeros([self.height, self.width], dtype=np.uint8)
            self.matMaskOthers        = np.zeros([self.height, self.width], dtype=np.uint8)
            
            if (self.params['tracking']['usetransforms']):
                b = False
                while not b:
                    try:
                        self.tfrx.waitForTransform('Mask', 
                                                   self.ptsOriginImage.header.frame_id, 
                                                   self.ptsOriginImage.header.stamp, 
                                                   rospy.Duration(1.0))
                    except tf.Exception, e:
                        rospy.logwarn('ExceptionA transforming mask frame %s->Mask:  %s' % (self.ptsOriginImage.header.frame_id, e))
                        
                    try:
                        self.ptsOriginMask = self.tfrx.transformPoint('Mask', self.ptsOriginImage)
                        self.ptsOriginMask.point.x = -self.ptsOriginMask.point.x
                        self.ptsOriginMask.point.y = -self.ptsOriginMask.point.y
                        b = True
                    except tf.Exception, e:
                        rospy.logwarn('ExceptionB transforming mask frame %s->Mask:  %s' % (self.ptsOriginImage.header.frame_id, e))
            else:
                self.ptsOriginMask = PointStamped(point=Point(x=0, y=0))
                        
            cv2.circle(self.matMask,
                      (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                      int(self.params['camera']['mask']['radius']), 
                      self.color_max, 
                      cv.CV_FILLED)
            
                
            self.bInitImages = True


    def Init_callback(self, experimentparamsChoices):
        return True

    # TrialStart_callback()
    # Publishes the current background image.
    #
    def TrialStart_callback(self, experimentparams):
        while (not self.bInitialized):
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
        while (not self.bInitialized):
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

    def WaitUntilDone_callback(self, experimentparamsChoices):
        return True


    def ImageBackgroundSet_callback (self, image):
        while (not self.bInitConstructor):
            rospy.loginfo('Waiting for initConstructor.')
            rospy.sleep(0.5)
        
            
        if (not self.selfPublishedBackground):
            try:
                self.matBackground = np.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
            except CvBridgeError, e:
                rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
                self.matBackground = None
            else:
                self.matfBackground = np.float32(self.matBackground)
                self.bInitBackground = True
        else:
            self.selfPublishedBackground = False
        
        
    def CameraInfo_callback (self, msgCameraInfo):
        if (self.bInitConstructor):
            self.camerainfo = msgCameraInfo
            
            if (not self.bInitialized):
                self.InitializeImages()
        
            if (self.bInitImages and self.bInitBackground):
                self.bInitialized = True
            
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
            self.bUseVisualServoing = trackingcommand.bUseVisualServoing
            if (self.bUseVisualServoing):
                self.nRobots            = trackingcommand.nRobots
            else:
                self.nRobots            = max(trackingcommand.nRobots-1, 0)
                
            self.nFlies             = trackingcommand.nFlies
            self.nObjects           = self.nRobots + self.nFlies
        
        
    
    # Given the various image moments, compute the angle and eccentricity.
    # Angle is set to NaN when the image is circular. 
    # Eigenvalues/vectors of
    # [ u20, -u11,
    #  -u11,  u02]
    #
    def FindAngleEcc(self, u20, u11, u02):
        angle = float('NaN')
        ecc = 1.0
        
        if (u11 != 0): # Div by zero.
            inside = 4*u11*u11 + u20*u20 - 2*u20*u02 + u02*u02
            if (inside >= 0): # Complex answer.
                inside = np.sqrt(inside)
                
                # Eigenvalues & eigenvectors.
                L1 = (u20+u02-inside)/2 
                L2 = (u20+u02+inside)/2
                V1 = (u02-u20+inside)/(2*u11)
                V2 = (u02-u20-inside)/(2*u11)
                
                rise = 1
                try:
                    if (L2 < L1):
                        run = -V1
                        ecc = L1/L2 # np.sqrt(1-(L1/L2)*(L1/L2))
                      
                    else:
                        run = -V2
                        ecc = L2/L1
                      
                    angle = -np.arctan2(rise, run)

                except:
                    rospy.logwarn ('Exception in FindAngleEcc()')
                    pass
        
        return angle, ecc
        

    # ContourinfoFromContour()
    # Convert a contour (a hierarchical list) into 
    # a contourinfo, a tuple of (x, y, area, angle, ecc).
    #
    def ContourinfoFromContour(self, contour):
        area = cv2.contourArea(contour)

        if (5 <= len(contour)):
            ((x,y), axes, degMinor) = cv2.fitEllipse(contour)
            
            degMajor = 90.0 - degMinor
            angle = (((degMajor % 180.0)-180.0) * np.pi / 180.0) # Put on range -180 to 0, and convert to radians.

            lenMajor = max(axes)
            lenMinor = min(axes)
            ecc = np.sqrt(1.0-(lenMinor/lenMajor)**2.0)
        else:
            moments = cv2.moments(contour)
      
            #rospy.logwarn('moments=%s' % repr(moments))
            m00 = moments['m00']
            m10 = moments['m10']
            m01 = moments['m01']
              
            if (m00 != 0.0):
                x = m10/m00
                y = m01/m00
            else: # There was just one pixel in the contour.
                (x,y) = contour[0][0]

            angle = float('NaN') # Must use NaN instead of None, so that it can go through the ROS message.
            ecc = 0.0
        
#         if (ecc>0.6):
#             rospy.logwarn('areas: %3.2f, %3.2f' % (area, area2))
#             rospy.logwarn('angles: %+3.2f, %+3.2f' % (angle*180/np.pi, angle2*180/np.pi))
#             rospy.logwarn('ecc: %+3.2f, %+3.2f' % (ecc, ecc2))
#             rospy.logwarn('-----')

        return (x, y, area, angle, ecc)
    

    # GetContourinfolistsDictFromContourList()
    # Converts contour to a contourinfo...
    # transforms it to the output frame...
    # computes the contour image minus the other contour images...
    # and appends the contourinfo members to their respective lists.
    #
    def GetContourinfolistsDictFromContourList(self, contour_list, matForeground):
        contourinfolists_dict = {'x':[], 'y':[], 'angle':[], 'area':[], 'ecc':[], 'imgRoi':[], 'iContour':[]}#, 'contour':[]}
        
        for iContour in range(len(contour_list)):
            contour = contour_list[iContour]
            (x, y, area, angle, ecc) = self.ContourinfoFromContour(contour)
            
                
            # Transform and Save contourinfo to the dict.
            ptsContour = PointStamped()
            ptsContour.header.frame_id = 'ImageRect'
            ptsContour.point.x = x
            ptsContour.point.y = y
            
            try:
                if (self.params['tracking']['usetransforms']):
                    self.ptsOutput = self.tfrx.transformPoint(self.params['tracking']['frameid_contours'], ptsContour)
                else:
                    self.ptsOutput = ptsContour
                    
            except tf.Exception, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.params['tracking']['frameid_contours'], ptsContour.header.frame_id, e))
                self.ptsOutput = PointStamped()
            except TypeError, e:
                rospy.logwarn ('Exception transforming point to frame=%s from frame=%s: %s' % (self.params['tracking']['frameid_contours'], ptsContour.header.frame_id, e))
    
            else:
                contourinfolists_dict['x'].append(self.ptsOutput.point.x)
                contourinfolists_dict['y'].append(self.ptsOutput.point.y)
                contourinfolists_dict['angle'].append(angle)
                contourinfolists_dict['area'].append(area)
                contourinfolists_dict['ecc'].append(ecc)
                contourinfolists_dict['imgRoi'].append(None)
                contourinfolists_dict['iContour'].append(iContour)
                #contourinfolists_dict['contour'].append(contour_list[iContour])
    
    
            # Get the ROI pixels for each contour, minus the pixels of the other contours.
            if (matForeground is not None):
                nContours = len(contourinfolists_dict['x'])
                #for iContour in range(nContours):
                x = contourinfolists_dict['x'][iContour]
                y = contourinfolists_dict['y'][iContour]
    
                self.matMaskOthers.fill(0)
                
                # Go through all the other contours.
                normRoi = np.linalg.norm([self.params['tracking']['roi']['width'],self.params['tracking']['roi']['height']])
                for kContour in range(nContours):
                    if (kContour != iContour):
                        xk = contourinfolists_dict['x'][kContour]
                        yk = contourinfolists_dict['y'][kContour]
                        
                        # If the kth ROI can overlap with the ith ROI, then add that fly's pixels to the subtraction mask.  
                        if (np.linalg.norm([x-xk,y-yk]) < normRoi):
                            jContour = contourinfolists_dict['iContour'][kContour]
                            cv2.drawContours(self.matMaskOthers, contour_list, jContour, 255, cv.CV_FILLED, 4, self.hierarchy, 0)
                        
                self.matMaskOthers = cv2.dilate(self.matMaskOthers, self.kernel, iterations=2)
                matOthers = cv2.bitwise_and(self.matMaskOthers, matForeground)
                matRoi = cv2.getRectSubPix(matForeground - matOthers, 
                                           (self.params['tracking']['roi']['width'], self.params['tracking']['roi']['height']), 
                                           (x,y))
                imgRoi = self.cvbridge.cv_to_imgmsg(cv.fromarray(matRoi), 'passthrough')
                contourinfolists_dict['imgRoi'][iContour] = imgRoi
                
    
        return contourinfolists_dict
            

    def RemoveDupsAndTooSmallContours(self, contoursIn_list):
        contoursOut_list = []
        iContourLargest = None
        
        # Put the contours into the contourinfolists_dict.
        contourinfolists_dict = self.GetContourinfolistsDictFromContourList(contoursIn_list, None)

        # Put lists into contourinfolists.
        contourinfolists = ContourinfoLists()
        
            
        if (0 < len(contoursIn_list)):
            # Skip too-small contours, and repackage the data as a list of lists, i.e. [[x,y,a,a,e,i,i],[x,y,a,a,e,i,i],...]
            contourinfolists_list = []
            for iContour in range(len(contoursIn_list)):
                if (self.params['tracking']['areaContourMin'] <= contourinfolists_dict['area'][iContour]):
                    contourinfolists_list.append([contourinfolists_dict['x'][iContour],        # 0 
                                                  contourinfolists_dict['y'][iContour],        # 1
                                                  contourinfolists_dict['angle'][iContour],    # 2
                                                  contourinfolists_dict['area'][iContour],     # 3
                                                  contourinfolists_dict['ecc'][iContour],      # 4
                                                  contourinfolists_dict['imgRoi'][iContour],   # 5
                                                  contourinfolists_dict['iContour'][iContour]])# 6

            # Remove the dups.
            contourinfolists_list = sorted(tuple(contourinfolists_list))
            contourinfolists_list = [x for i, x in enumerate(contourinfolists_list) if (not i) or (np.linalg.norm(np.array(contourinfolists_list[i][0:2])-np.array(contourinfolists_list[i-1][0:2])) > self.params['tracking']['distanceDuplicateContour'])]
        
        
            # Count the contours.
            self.nContours = len(contourinfolists_list)
            

            # Reconstruct the contour_list from the deduped data.
            for iContour in range(self.nContours):
                contoursOut_list.append(contoursIn_list[contourinfolists_list[iContour][6]]) # Append contours indexed from the original list.

        
        return contoursOut_list
    
    
    # GetMergedContour()
    # Determine if two contours have merged, and if so which is the merged contour.
    #
    def GetMergedContour(self, contour_list):
        
        # Only update the prev distance if we're not merged.
        if (not self.bMerged):
            self.rClosestPrev = self.rClosest
         
             
        # Make an Nx2 array of contour positions.
        nContours = len(contour_list)
        contourinfolists_dict = self.GetContourinfolistsDictFromContourList(contour_list, None)
        xyContours = np.array([contourinfolists_dict['x'], contourinfolists_dict['y']]).T
         
         
        # Compute the distances from each contour to each other contour, then find the min.
        d0 = np.subtract.outer(xyContours[:,0], xyContours[:,0]) # nContours-by-nContours matrix of x-distances between contours.
        d1 = np.subtract.outer(xyContours[:,1], xyContours[:,1]) # nContours-by-nContours matrix of y-distances between contours.
        d = np.hypot(d0, d1) + np.eye(len(contour_list))*55555                # nContours-by-nContours matrix of 2-norm distances between contours (but can't be close to itself).
        self.rClosest = d.min() # Distance between the two closest contours (in pixels).
         
                 
        # Determine if there's a merged contour.
        if (self.nContours < self.nObjects) and (self.rClosestPrev < self.params['tracking']['distanceMergedContours']):
            self.bMerged = True
        else:
            self.bMerged = False


        # Find the contour with the largest area.
        if (self.bMerged):
            iContourMerged = contourinfolists_dict['iContour'][np.argmax(contourinfolists_dict['area'])]
        else:
            iContourMerged = None
        
            
        return iContourMerged
                

        
    # Split a contour into two (or multiple possible subcontours).
    def SplitContour(self, contourIn, defects):
        contoursOut = []
        
        if (defects is not None):
            nDefects = len(defects)
            
            # Divide into two, based on kmeans clustering.
            if (nDefects < 2):
                contourInB = contourIn.squeeze()
                centroids,_ = kmeans(contourInB, 2)    # Make two clusters.
                iAssignment,_ = vq(contourInB, centroids)          # Assign points to clusters.
                
                contoursOut.append(contourIn[iAssignment==0])
                contoursOut.append(contourIn[iAssignment==1])

            
            # Divide into two, at the line connecting the two greatest defects.
            if (nDefects==2):
                iBySize = np.argsort(defects[:,0,3]) # Sorted indices of defect size.
                
                # Get the two largest defects, in index order.
                j = min(defects[iBySize[-1],0,2], defects[iBySize[-2],0,2])
                k = max(defects[iBySize[-1],0,2], defects[iBySize[-2],0,2])
                
                # Don't keep the shared points.
                contoursOut.append(contourIn[j+1:k])
                contoursOut.append(np.concatenate((contourIn[0:j],contourIn[k+1:len(contourIn)+1])))

                
            # Divide into three candidate contour pairs, and let the ContourIdentifier figure out which are the best match.
            if (2 < nDefects):
                iBySize = np.argsort(defects[:,0,3]) # Indices sorted by defect size.
                
                # Indices of three largest defects, sorted by point location.
                iByLocation = np.array([defects[iBySize[-3],0,2], defects[iBySize[-2],0,2], defects[iBySize[-1],0,2]])
                iByLocation.sort()

                # Indices of three largest defects, in location order.
                j = iByLocation[0]
                k = iByLocation[1]
                m = iByLocation[2]

                # Three contours.
                #contoursOut.append(contourIn[j+1:k])
                #contoursOut.append(contourIn[k+1:m])
                #contoursOut.append(np.concatenate((contourIn[0:j],contourIn[m+1:len(contourIn)+1])))

                # Three pairs of contours.
                (p1, p2) = (j, k)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(np.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))

                (p1, p2) = (k, m)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(np.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))
                
                (p1, p2) = (j, m)
                contoursOut.append(contourIn[p1+1:p2])
                contoursOut.append(np.concatenate((contourIn[0:p1],contourIn[p2+1:len(contourIn)+1])))
                
                
        else:
            contourInB = contourIn.squeeze()
            centroids,_ = kmeans(contourInB, 2)         # Make two clusters.
            iAssignment,_ = vq(contourInB, centroids)   # Assign points to clusters.
            
            contoursOut.append(contourIn[iAssignment==0])
            contoursOut.append(contourIn[iAssignment==1])


        return contoursOut #(contour1,contour2)
        
        
        
    def ContourinfoListsFromImage(self, matThreshold, matForeground):
        # Find contours
        sumImage = cv2.sumElems(matThreshold)
        if (self.minSumImage < sumImage[0]):
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
        self.contour_list = self.RemoveDupsAndTooSmallContours(self.contour_list)
        self.nContours = len(self.contour_list)

            

        if (0 < self.nContours):

            # Determine if two contours have merged, and need to be split.
            #rospy.logwarn(self.contour_list)
            iContourMerged = self.GetMergedContour(self.contour_list)

            # Split the merged contour.
            if (iContourMerged is not None):
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
        self.hierarchy = np.array(self.hierarchy)
                                         

        # Put the contours into the contourinfolists_dict.
        contourinfolists_dict = self.GetContourinfolistsDictFromContourList(self.contour_list, matForeground)
                
                    
        
        # Put lists into a ContourinfoLists message.
        contourinfolists = ContourinfoLists()
        contourinfolists.header.seq = self.seq
        contourinfolists.header.stamp = self.header.stamp
        contourinfolists.header.frame_id = self.params['tracking']['frameid_contours'] # i.e. Camera
        self.seq += 1
        if (0 < self.nContours):
            contourinfolists.x = contourinfolists_dict['x']
            contourinfolists.y = contourinfolists_dict['y']
            contourinfolists.angle = contourinfolists_dict['angle']
            contourinfolists.area = contourinfolists_dict['area']
            contourinfolists.ecc = contourinfolists_dict['ecc']
            contourinfolists.imgRoi = contourinfolists_dict['imgRoi']
        
            
        self.nContoursPrev = self.nContours
    
        return contourinfolists  
        

    def Image_callback(self, rosimg):
        # Receive the image:
        with self.lockBuffer:
            if (self.bufferImages[self.iImgLoading] is None):   # There's an empty slot in the buffer.
                iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                iImgWorkingNext = self.iImgWorking
                self.iDroppedFrame = 0
            else:                                               # The buffer is full; we'll overwrite the oldest entry.
                iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                iImgWorkingNext = (self.iImgWorking+1) % len(self.bufferImages)
                self.iDroppedFrame += 1

            self.bufferImages[self.iImgLoading] = rosimg
            self.iImgLoading = iImgLoadingNext
            self.iImgWorking = iImgWorkingNext
    
    
    def ProcessImage(self):
        rosimg = None
        
        with self.lockBuffer:
            if (self.bufferImages[self.iImgWorking] is not None):
                rosimg = self.bufferImages[self.iImgWorking]
                
                # Mark this buffer entry as available for loading.
                self.bufferImages[self.iImgWorking] = None
    
                # Go to the next image.
                self.iImgWorking = (self.iImgWorking+1) % len(self.bufferImages)

        
        if (rosimg is not None):
            try:
                #rospy.logwarn('Image_callback(now-prev=%s)' % (rospy.Time.now().to_sec()-self.timePrev))
                #self.timePrev = rospy.Time.now().to_sec()
                if (not self.bInitConstructor):
                    return
        
                if (self.stampPrev is not None):
                    self.dt = rosimg.header.stamp - self.stampPrev
                else:
                    self.dt = rospy.Time(0)
                self.stampPrev = rosimg.header.stamp
                
        
                self.header = rosimg.header
                self.height = rosimg.height
                self.width = rosimg.width
                # Convert ROS image to OpenCV image.
                try:
                    self.matImageRect = np.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(rosimg, 'passthrough')))
                except CvBridgeError, e:
                    rospy.logwarn ('Exception converting ROS image to opencv:  %s' % e)
                if (not self.bInitImages):
                    self.InitializeImages()
                if (self.bInitImages and self.bInitBackground):
                    self.bInitialized = True
                    
                #rospy.logwarn('%s', (self.bInitConstructor, self.bInitImages, self.bInitBackground, self.bInitialized))
                if (self.bInitialized):        
                    # Check for new params.
                    tParams = rospy.Time.now()
                    if (1 < (tParams-self.tParamsPrev).to_sec()):
                        self.tParamsPrev = tParams
                        self.paramsPrev = copy.deepcopy(self.params) 
                        SetDict.SetWithOverwrite(self.params, rospy.get_param('/',{}))
                    
                    # Create the mask.
                    if (self.paramsPrev['camera']['mask']['radius'] != self.params['camera']['mask']['radius']):
                        self.matMask.fill(0)
                        cv2.circle(self.matMask,
                                  (int(self.ptsOriginMask.point.x),int(self.ptsOriginMask.point.y)),
                                  int(self.params['camera']['mask']['radius']), 
                                  self.color_max, 
                                  cv.CV_FILLED)
                    
                    # Normalize the histogram.
                    if (self.bEqualizeHist):
                        self.matImageRect = cv2.equalizeHist(self.matImageRect)
                    
                    
                    # Update the background.
                    #rospy.logwarn('types: %s' % [type(np.float32(self.matImageRect)), type(self.matfBackground), type(self.rcBackground)])
                    if (self.bEstablishBackground):
                        rcBackground = self.params['tracking']['rcBackgroundEstablish']
                    else:
                        rcBackground = self.params['tracking']['rcBackground'] # Time constant for moving average background.
                    
                    alphaBackground = 1 - np.exp(-self.dt.to_sec() / rcBackground)
                    
                    try:
                        cv2.accumulateWeighted(np.float32(self.matImageRect), self.matfBackground, alphaBackground)
                    except cv2.error:
                        rospy.logerr('Most likely your background image file did not come from this camera.')
                        rospy.logerr('Please delete the file (%s) and try again.' % self.params['tracking']['filenameBackground'])
                        
                    self.matBackground = np.uint8(self.matfBackground)
        
        
                    # Create the foreground.
                    if (self.bEqualizeHist):
                        matBackgroundEq = cv2.equalizeHist(self.matBackground)
        
                    if (self.params['tracking']['usebackgroundsubtraction']) and (self.matBackground is not None):
                        self.matForeground = cv2.absdiff(self.matImageRect, self.matBackground)
                    else:
                        self.matForeground = self.matImageRect
        
                    
                    # Threshold
                    (threshold,self.matThreshold) = cv2.threshold(self.matForeground, 
                                                     self.params['tracking']['diff_threshold'], 
                                                     self.max_8U, 
                                                     cv2.THRESH_TOZERO)
        
                    
                    # Apply the arena mask to the threshold image.
                    self.matThreshold = cv2.bitwise_and(self.matThreshold, self.matMask)
        
                    # Get the ContourinfoLists.
                    self.contourinfolists = self.ContourinfoListsFromImage(self.matThreshold, self.matForeground)    # Modifies self.matThreshold
                    self.pubContourinfoLists.publish(self.contourinfolists)
                    
                    # Convert to color for display image
                    if (0 < self.pubImageProcessed.get_num_connections()):
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
                        if (self.contour_list):
                            for k in range(len(self.contour_list)):
                                #color = cv.CV_RGB(0,0,self.color_max)
                                color = colors[k % len(colors)]
                                cv2.drawContours(self.matProcessed, self.contour_list, k, color, thickness=1, maxLevel=1)
                        
                    
                    # Publish processed image
                    if (0 < self.pubImageProcessed.get_num_connections()):
                        try:
                            image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matProcessed), 'passthrough')
                            image2.header = rosimg.header
                            image2.encoding = 'bgr8' # Fix a bug introduced in ROS fuerte.
                            #rospy.logwarn(image2.encoding)
                            self.pubImageProcessed.publish(image2)
                        except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                            rospy.logwarn ('Exception %s' % e)
                    
                    # Publish background image
                    if (self.pubImageBackground.get_num_connections() > 0) and (self.matBackground is not None):
                        try:
                            image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matBackground), 'passthrough')
                            image2.header.stamp = rosimg.header.stamp
                            image2.encoding = 'mono8'
                            self.pubImageBackground.publish(image2)
                        except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                            rospy.logwarn ('Exception %s' % e)
        
                    
                    # Publish thresholded image
                    if (0 < self.pubImageThreshold.get_num_connections()):
                        try:
                            self.matImageRect = cv2.add(self.matThreshold, self.matForeground)
                            image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matImageRect), 'passthrough')
                            image2.header.stamp = rosimg.header.stamp
                            image2.encoding = 'mono8'
                            self.pubImageThreshold.publish(image2)
                        except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                            rospy.logwarn ('Exception %s' % e)
        
                      
                    # Publish foreground image
                    if (0 < self.pubImageForeground.get_num_connections()):
                        try:
                            image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matForeground), 'passthrough')
                            image2.header.stamp = rosimg.header.stamp
                            image2.encoding = 'mono8'
                            self.pubImageForeground.publish(image2)
                        except (MemoryError, CvBridgeError, rospy.exceptions.ROSException), e:
                            rospy.logwarn ('Exception %s' % e)
        
                      
                    # Publish a special image for use in rviz.
                    if (0 < self.pubImageRviz.get_num_connections()):
                        self.matProcessedFlip = cv2.flip(self.matProcessed, 0)
                        image2 = self.cvbridge.cv_to_imgmsg(cv.fromarray(self.matImageRect), 'passthrough')
                        image2.header = rosimg.header
                        image2.encoding = 'mono8'
                        
                        params = rospy.get_param('/', {})
                        defaults = {'k11':1000, 'k13':320, 'k22':1000, 'k23':240, 'k33':1.0, 'p11':1000, 'p13':320, 'p22':1000, 'p23':240, 'p33':1.0}
                        SetDict.SetWithPreserve(params, defaults)
                          
                        k11 = params['k11']
                        k13 = params['k13']
                        k22 = params['k22']
                        k23 = params['k23']
                        k33 = params['k33']
                        p11 = params['p11']
                        p13 = params['p13']
                        p22 = params['p22']
                        p23 = params['p23']
                        p33 = params['p33']
        
        #                     camerainfo2 = CameraInfo()#copy.copy(self.camerainfo)
        #                     camerainfo2.header = rosimg.header
        #                     camerainfo2.height = self.height
        #                     camerainfo2.width = self.width
                        z = rospy.get_param('camera/arena_tvec_2')
                        mag = z/self.camerainfo.K[0]
                        camerainfo2 = copy.copy(self.camerainfo)
                        camerainfo2.D = (0.0, 0.0, 0.0, 0.0, 0.0)
#                         camerainfo2.K = (mag*self.camerainfo.K[0], mag*self.camerainfo.K[1], mag*self.camerainfo.K[2], \
#                                          mag*self.camerainfo.K[3], mag*self.camerainfo.K[4], mag*self.camerainfo.K[5], \
#                                              self.camerainfo.K[6],     self.camerainfo.K[7], self.camerainfo.K[8])
                        camerainfo2.K = (k11, 0,   k13, \
                                         0,   k22, k23, \
                                         0,   0,   k33)
                                         
                        camerainfo2.R = (1.0, 0.0, 0.0, \
                                         0.0, 1.0, 0.0, \
                                         0.0, 0.0, 1.0)
                        mag = z/self.camerainfo.P[0]
#                         camerainfo2.P = (mag*self.camerainfo.P[0], mag*self.camerainfo.P[1], mag*self.camerainfo.P[2],  self.camerainfo.P[3], \
#                                          mag*self.camerainfo.P[4], mag*self.camerainfo.P[5], mag*self.camerainfo.P[6],  self.camerainfo.P[7], \
#                                              self.camerainfo.P[8],     self.camerainfo.P[9],     self.camerainfo.P[10], self.camerainfo.P[11])
                        camerainfo2.P = (p11, 0,   p13, 0, \
                                         0,   p22, p23, 0, \
                                         0,   0,   p33, 0)
        
        
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
        while (not rospy.is_shutdown()):
            self.ProcessImage()

#         # Shutdown all the services we offered.
#         for key in self.services:
#             self.services[key].shutdown()
        
      

if (__name__ == '__main__'):
    rospy.init_node('ContourGenerator') #, anonymous=True)
        
    cg = ContourGenerator()
    cg.Main()
    #cProfile.run('cg.Main()', '/home/rancher/profile.pstats')    

  
