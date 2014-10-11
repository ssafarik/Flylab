#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('calibration')
import rospy
import copy
import cv
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo

from arena_tf.srv import ArenaCameraConversion
from tracking.msg import ContourinfoLists
from patterngen.msg import MsgPattern
from flycore.msg import MsgFrameState


class CalibrateStageArena():
    def __init__(self):
        self.initialized = False
        self.initialized_arrays = False
        rospy.init_node('CalibrateStageArena')
        
        self.cvbridge = CvBridge()

        self.tfrx = tf.TransformListener()
        self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo,       self.CameraInfo_callback, queue_size=2)
        self.subImage               = rospy.Subscriber('camera/image_rect',  Image,            self.Image_callback, queue_size=2)
        self.subContourinfoLists    = rospy.Subscriber('ContourinfoLists',   ContourinfoLists, self.ContourinfoLists_callback, queue_size=2)
        self.pubSetPattern          = rospy.Publisher('SetPattern',          MsgPattern, latch=True)

        self.camerainfo = None
        self.imgOutput = None
        
        self.cacheImage = [None, None]
        self.cachePose = [None, None]
        self.iWorking = 0
                
        cv2.namedWindow('Stage/Arena Calibration', 1)
        
        
        self.colorMax = 255
        self.colorFont = cv.CV_RGB(self.colorMax,0,0)
        self.textError = ''
        
        self.nPointsCriteria = 20 #60
        self.distPointsCriteria = 10

        self.eccVisual = 0
        self.areaVisual = 0
        self.eccVisual_array = np.array([])
        self.areaVisual_array = np.array([])
        self.eccVisualMin = 1000000000
        self.eccVisualMax = 0
        self.areaVisualMin = 1000000000
        self.areaVisualMax = 0
        
        self.xMin = 99999
        self.xMax = -99999
        self.yMin = 99999
        self.yMax = -99999

        # Checkerboard info
        self.sizeCheckerboard = (3,2)
        self.nCols = self.sizeCheckerboard[0]
        self.nRows = self.sizeCheckerboard[1]
        self.checker_size = rospy.get_param('calibration/checker_size', 15)
        self.bRotateGrid = False
        
        self.rvec      = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec      = np.zeros([1, 3], dtype=np.float32).squeeze()

        self.rvec[0] = rospy.get_param('/camera/arena_rvec_0')
        self.rvec[1] = rospy.get_param('/camera/arena_rvec_1')
        self.rvec[2] = rospy.get_param('/camera/arena_rvec_2')
        self.tvec[0] = rospy.get_param('/camera/arena_tvec_0')
        self.tvec[1] = rospy.get_param('/camera/arena_tvec_1')
        self.tvec[2] = rospy.get_param('/camera/arena_tvec_2')
        
        #self.tvec[0] = 0.0
        #self.tvec[1] = 0.0
        #self.tvec[2] = 0.0
        #rospy.logwarn(self.tvec)
      

#         rospy.loginfo ('Waiting for ImageRect transform...')
#         while True:
#             try:
#                 stamp = self.tfrx.getLatestCommonTime('ImageRect', 'Camera')
#                 (self.transCameraRect, rot) = self.tfrx.lookupTransform('ImageRect', 'Camera', stamp)
#                 break
#             except tf.Exception, e:
#                 rospy.logwarn('Exception1 in StageArena: %s' % e)
#         rospy.loginfo ('Found ImageRect transform.')
        

        rospy.loginfo ('Waiting for service: arena_from_image...')
        rospy.wait_for_service('arena_from_image')
        try:
            self.ArenaFromCamera = rospy.ServiceProxy('arena_from_image', ArenaCameraConversion)
        except rospy.ServiceException, e:
            rospy.logwarn('Exception2 in StageArena: %s' % e)
        rospy.loginfo ('Found service: arena_from_image.')

        rospy.loginfo ('Waiting for service: image_from_arena...')
        rospy.wait_for_service('image_from_arena')
        try:
            self.CameraFromArena = rospy.ServiceProxy('image_from_arena', ArenaCameraConversion)
        except rospy.ServiceException, e:
            rospy.logwarn('Exception3 in StageArena: %s' % e)
        rospy.loginfo ('Found service: image_from_arena.')

            
        self.nRobots = 1
            
        rospy.on_shutdown(self.OnShutdown_callback)

        
        
    def OnShutdown_callback(self):
        if self.initialized:
            msgPattern = MsgPattern()
            msgPattern.shape = 'constant'
            msgPattern.points = []
            msgPattern.frameidPosition = 'Stage'
            msgPattern.frameidAngle = 'Stage'
            msgPattern.hzPattern = 1.0
            msgPattern.hzPoint = 100
            msgPattern.count = 1
            msgPattern.size = Point(x=0,y=0)
            msgPattern.restart = True
            msgPattern.param = 0
            self.pubSetPattern.publish (msgPattern)
        
    
    def CameraInfo_callback (self, msgCameraInfo):
        self.camerainfo = msgCameraInfo
        self.K_rect = np.reshape(self.camerainfo.P,[3,4])[0:3,0:3] # camerainfo.K and camerainfo.D apply to image_raw; P w/ D=0 applies to image_rect.
        self.D_rect = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

      

    def Image_callback(self, image):
        # Point to the cache non-working entry.
        iLoading = (self.iWorking+1) % 2
        self.cacheImage[iLoading] = image 
    
    
    def ProcessImage(self):
        if (self.cacheImage[self.iWorking] is not None) and (self.cachePose[self.iWorking] is not None):
            image = self.cacheImage[self.iWorking]
            poseVisual = self.cachePose[self.iWorking]

            t0 = rospy.Time.now().to_sec()
            if (self.initialized):
                try:
                    imgCamera = np.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
                except CvBridgeError, e:
                    rospy.logwarn('Exception CvBridgeError in image callback: %s' % e)
                
                if (self.imgOutput is None):
                    self.imgOutput   = np.zeros([imgCamera.shape[0], imgCamera.shape[1], 3], dtype=np.uint8)
                
                self.imgOutput = cv2.cvtColor(imgCamera, cv.CV_GRAY2RGB)
                xText = 25
                yText = 25
                dyText = 20
                
                
                if (self.nRobots>0):
                    try:
                        stamp = self.tfrx.getLatestCommonTime('Stage', 'EndEffector')
                        (posKinematic, rotKinematic_quat) = self.tfrx.lookupTransform('Stage', 'EndEffector', stamp)
                    except tf.Exception, e:
                        self.initialized_endeffector = False
                        rospy.logwarn ('SA Exception getting Kinematic coordinates: %s' % e)
                    else:
                        xKinematic = posKinematic[0]
                        yKinematic = posKinematic[1]
                        zKinematic = posKinematic[2]
                        self.initialized_endeffector = True
                else:
                    # No robots means we'll just end up with the identity transform.
                    xKinematic = poseVisual.pose.position.x
                    yKinematic = poseVisual.pose.position.y
                    zKinematic = 0.0
                    self.initialized_endeffector = True
                    
                    
                if (self.initialized_endeffector):
                    pointVisualNew    = np.array([[poseVisual.pose.position.x], [poseVisual.pose.position.y],  [0]])             # In Arena coordinates, from tracking.
                    pointKinematicNew = np.array([[xKinematic],                 [yKinematic],                  [zKinematic]])    # In Stage coordinates, from kinematics.
                    
                    if (self.initialized_arrays):
                        pointKinematicPrev = self.pointKinematic_array[:,-1].reshape((3,1))
                        if self.distPointsCriteria < tf.transformations.vector_norm((pointKinematicNew - pointKinematicPrev)):
                            self.pointVisual_array = np.append(self.pointVisual_array, pointVisualNew, axis=1)
                            self.pointKinematic_array = np.append(self.pointKinematic_array, pointKinematicNew, axis=1)
                            self.eccVisual_array = np.append(self.eccVisual_array,self.eccVisual)
                            self.areaVisual_array = np.append(self.areaVisual_array,self.areaVisual)
                    else:
                        self.pointVisual_array = pointVisualNew
                        self.pointKinematic_array = pointKinematicNew
                        self.initialized_arrays = True
                    
    
                    n = self.pointVisual_array.shape[1]
                    for iPoint in range(max(0,n-10),n):
                        # Draw blue dots on prior visual locations.
                        pointImage = self.PointImageFromArena(Point(x=self.pointVisual_array[0,iPoint], y=self.pointVisual_array[1,iPoint]))                        
                        cv2.circle(self.imgOutput, 
                                   (int(pointImage.x), int(pointImage.y)), 
                                   5, cv.CV_RGB(0,0,self.colorMax/2), cv.CV_FILLED)
                        
                        # Draw green dots on prior kinematic locations.
                        pointImage = self.PointImageFromArena(Point(x=self.pointKinematic_array[0,iPoint], y=self.pointKinematic_array[1,iPoint]))                        
                        cv2.circle(self.imgOutput, 
                                   (int(pointImage.x), int(pointImage.y)), 
                                   5, cv.CV_RGB(0,self.colorMax/2,0), cv.CV_FILLED)
                    
                    # Draw bright blue dots on visual location.
                    pointImage = self.PointImageFromArena(poseVisual.pose.position)                        
                    cv2.circle(self.imgOutput, 
                               (int(pointImage.x), int(pointImage.y)), 
                               3, cv.CV_RGB(0,0,self.colorMax), cv.CV_FILLED)
                    
                    # Draw bright green dots on kinematic location.
                    pointImage = self.PointImageFromArena(Point(x=xKinematic, y=yKinematic))                        
                    cv2.circle(self.imgOutput, 
                               (int(pointImage.x), int(pointImage.y)), 
                               3, cv.CV_RGB(0,self.colorMax,0), cv.CV_FILLED)
    
    
                    nPoints = self.pointKinematic_array.shape[1]
                    if (self.nPointsCriteria < nPoints):
                        
                        # The transform between kinematic millimeters and visual pixels.
                        self.T_m2p = tf.transformations.superimposition_matrix(self.pointKinematic_array, self.pointVisual_array, scaling=True)
                        self.T_p2m = tf.transformations.inverse_matrix(self.T_m2p)
                        T = self.T_p2m
                        
                        position = tf.transformations.translation_from_matrix(T)
                        
                        # Eccentricity.
                        eccMean = np.mean(self.eccVisual_array)
                        eccStd  = np.std(self.eccVisual_array)
                        self.eccVisualMin = eccMean - 3*eccStd
                        if self.eccVisualMin < 0:
                            self.eccVisualMin = 0
                        self.eccVisualMax = eccMean + 3*eccStd
                        # rospy.logwarn('eccMean = %s, eccStd = %s\n' % (eccMean, eccStd))
                        
                        # Area.
                        areaMean = np.mean(self.areaVisual_array)
                        areaStd  = np.std(self.areaVisual_array)
                        self.areaVisualMin = areaMean - 3*areaStd
                        if self.areaVisualMin < 0:
                            self.areaVisualMin = 0
                        self.areaVisualMax = areaMean + 3*areaStd
                        # rospy.logwarn('areaMean = %s, areaStd = %s\n' % (areaMean, areaStd))
                        
                        (factor, origin, direction) = tf.transformations.scale_from_matrix(T)
                        quaternion = tf.transformations.quaternion_from_matrix(T)
                        # euler = tf.transformations.euler_from_matrix(self.T_arena_stage,'rxyz')
    
    
    
                        display_text = 'Translation = [%0.3f, %0.3f, %0.3f]' % (position[0], position[1], position[2])
                        cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                        yText += dyText
                        display_text = 'Quaternion = [%0.3f, %0.3f, %0.3f, %0.3f]' % (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                        cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                        yText += dyText
                        display_text = 'eccVisualMin = %0.3f, eccVisualMax = %0.3f' % (self.eccVisualMin, self.eccVisualMax)
                        cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                        yText += dyText
                        display_text = 'areaVisualMin = %0.0f, areaVisualMax = %0.0f' % (self.areaVisualMin, self.areaVisualMax)
                        cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                        yText += dyText
                        display_text = 'Factor = %s' % factor
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'Origin = %s' % origin
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'Direction = %s' % direction
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'T = [%+0.4f, %+0.4f, %+0.4f, %+0.4f]' % (T[0][0], T[0][1], T[0][2], T[0][3])
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'T = [%+0.4f, %+0.4f, %+0.4f, %+0.4f]' % (T[1][0], T[1][1], T[1][2], T[1][3])
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'T = [%+0.4f, %+0.4f, %+0.4f, %+0.4f]' % (T[2][0], T[2][1], T[2][2], T[2][3])
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        display_text = 'T = [%+0.4f, %+0.4f, %+0.4f, %+0.4f]' % (T[3][0], T[3][1], T[3][2], T[3][3])
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText
                        
                        display_text = 'nPoints = %d' % nPoints
                        cv2.putText(self.imgOutput, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                        yText += dyText

                        self.bRotateGrid = True
                    else:
                        yText += 12*dyText
                        
                            
                    yText += dyText
                    display_text = 'positionVisual = [%0.3f, %0.3f]' % (poseVisual.pose.position.x, poseVisual.pose.position.y)
                    cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    display_text = 'positionKinematic = [%0.3f, %0.3f]' % (xKinematic, yKinematic)
                    cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    xRadius = (self.xMax - self.xMin)/2
                    yRadius = (self.yMax - self.yMin)/2
                    radius = (xRadius + yRadius)/2
                    display_text = 'Max Radius = %0.3f' % radius
                    cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    
                # End if self.initialized_endeffector
                
                
                cv2.putText(self.imgOutput, self.textError, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                x=rospy.get_param('/x',0)
                y=rospy.get_param('/y',0)
                self.imgOutput[y,x] = 0
                
                cv2.imshow('Stage/Arena Calibration', self.imgOutput)
                cv2.waitKey(1)
            t1 = rospy.Time.now().to_sec()
            #rospy.logwarn('time=%f' % (t1-t0))

            # Mark this entry as done.
            self.cacheImage[self.iWorking] = None
            self.cachePose[self.iWorking] = None
            
        # Go to the other image.
        self.iWorking = (self.iWorking+1) % 2

   
    def ContourinfoLists_callback(self, contourinfolists):
        if self.initialized:
            nContours  = len(contourinfolists.x)
            
            if (nContours==1):
                self.textError = ''

                poseContour = PoseStamped(header=contourinfolists.header,
                                          pose=Pose(position=Point(x=contourinfolists.x[0],
                                                                   y=contourinfolists.y[0])) 
                                          )

                poseVisual = PoseStamped()
                poseVisual.header = contourinfolists.header
                poseVisual.header.frame_id = 'Arena'
                poseVisual.pose = self.PoseArenaFromImage(poseContour) # Gives the position of the contour in millimeters.
                
                # Point to the cache non-working entry.
                iLoading = (self.iWorking+1) % 2
                self.cachePose[iLoading] = poseVisual 

                self.areaVisual = contourinfolists.area[0]
                self.eccVisual  = contourinfolists.ecc[0]
                
                self.xMin = min(self.xMin, poseVisual.pose.position.x)
                self.xMax = max(self.xMax, poseVisual.pose.position.x)
                self.yMin = min(self.yMin, poseVisual.pose.position.y)
                self.yMax = max(self.yMax, poseVisual.pose.position.y)
                
            elif (nContours==0):
                #rospy.logwarn('ERROR:  No objects detected.')
                self.textError = 'ERROR:  No objects detected.'
            elif (nContours>1):
                #rospy.logwarn('ERROR:  More than one object detected.')
                self.textError = 'ERROR:  More than one object detected.'
            

    # PoseArenaFromImage()
    # Takes a pose in image coordinates (i.e. pixels), and transforms it to arena coordinates (i.e. millimeters).
    #
    def PoseArenaFromImage(self, poseImage):
        response = self.ArenaFromCamera([poseImage.pose.position.x],[poseImage.pose.position.y])
        poseArena = Pose()
        poseArena.position.x = response.xDst[0]
        poseArena.position.y = response.yDst[0]
        
        return poseArena
    
    
    # PointImageFromArena()
    # Takes a point in arena coordinates (i.e. millimeters), and transforms it to image coordinates (i.e. pixels).
    #
    def PointImageFromArena(self, pointArena):
        response = self.CameraFromArena([pointArena.x],[pointArena.y])
        pointImage = Point(x=response.xDst[0], y=response.yDst[0])
        
        return pointImage
    
    
    def Main(self):
        rospy.sleep(2)
        msgPattern = MsgPattern()

        # Publish a goto(0,0) pattern message, and wait for the end effector to initialize.
        msgPattern.frameidPosition = 'Stage'
        msgPattern.frameidAngle = 'Stage'
        msgPattern.shape = 'constant'
        msgPattern.hzPattern = 1.0
        msgPattern.hzPoint = rospy.get_param('actuator/hzPoint', 100.0)
        msgPattern.count = 1
        msgPattern.size = Point(x=0,y=0)
        msgPattern.points = []
        msgPattern.restart = True
        msgPattern.param = 0
        self.pubSetPattern.publish (msgPattern)
        rospy.sleep(2)
        self.initialized = True

        # Setup the calibration pattern.
        msgPattern.frameidPosition = 'Stage'
        msgPattern.frameidAngle = 'Stage'
        msgPattern.shape = rospy.get_param('calibration/shape', 'spiral')
        if msgPattern.shape=='spiral':
            msgPattern.hzPattern = 0.008
        else:
            msgPattern.hzPattern = 0.1
        msgPattern.count = -1
        msgPattern.points = []
        msgPattern.size = Point(x=0.9*rospy.get_param('arena/radius_inner', 25.4), y=0)
        msgPattern.restart = True
        msgPattern.param = 0.0
        msgPattern.direction = 1

        try:
            while (not rospy.is_shutdown()):
                t0 = rospy.Time.now().to_sec()
                msgPattern.direction *= -1
                self.pubSetPattern.publish (msgPattern)
                while (rospy.Time.now().to_sec()-t0 < 60):
                    self.ProcessImage()
        except Exception:
            pass
        
        # Publish a goto(0,0) pattern message.
        msgPattern.frameidPosition = 'Stage'
        msgPattern.frameidAngle = 'Stage'
        msgPattern.shape = 'constant'
        msgPattern.hzPattern = 1.0
        msgPattern.count = 1
        msgPattern.size = Point(x=0,y=0)
        msgPattern.points = []
        msgPattern.restart = True
        msgPattern.param = 0
        self.pubSetPattern.publish (msgPattern)
    
    

if __name__ == '__main__':
    cal = CalibrateStageArena()
    cal.Main()
    cv2.destroyAllWindows()

