#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('calibration')
import rospy
import copy
import cv
import cv2
import numpy as N
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
        self.initialized_images = False
        self.initialized_pose = False
        self.initialized_arrays = False
        #self.initialized_endeffector = False
        rospy.init_node('CalibrateStageArena')
        
        self.cvbridge = CvBridge()

        self.tfrx = tf.TransformListener()
        self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo,       self.CameraInfo_callback)
        self.subImage               = rospy.Subscriber('camera/image_rect',  Image,            self.Image_callback)
        self.subContourinfoLists    = rospy.Subscriber('ContourinfoLists',   ContourinfoLists, self.ContourinfoLists_callback)
        self.pubSetPattern          = rospy.Publisher('SetPattern',          MsgPattern, latch=True)

        self.camerainfo = None
        
        cv2.namedWindow('Stage/Arena Calibration', 1)
        
        self.poseRobot = PoseStamped()
        
        self.colorMax = 255
        self.colorFont = cv.CV_RGB(self.colorMax,0,0)
        self.textError = ''
        
        self.nPointsCriteria = 20 #60
        self.distPointsCriteria = 10

        self.eccRobot = 0
        self.areaRobot = 0
        self.eccRobot_array = N.array([])
        self.areaRobot_array = N.array([])
        self.eccRobotMin = 1000000000
        self.eccRobotMax = 0
        self.areaRobotMin = 1000000000
        self.areaRobotMax = 0
        
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
        
        self.rvec      = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.tvec      = N.zeros([1, 3], dtype=N.float32).squeeze()

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
      

        rospy.loginfo ('Waiting for ImageRect transform...')
        while True:
            try:
                stamp = self.tfrx.getLatestCommonTime('ImageRect', 'Camera')
                (self.transCameraRect, rot) = self.tfrx.lookupTransform('ImageRect', 'Camera', stamp)
                break
            except tf.Exception, e:
                rospy.logwarn('Exception1 in StageArena: %s' % e)
        rospy.loginfo ('Found ImageRect transform.')
        

        rospy.loginfo ('Waiting for service: arena_from_camera...')
        rospy.wait_for_service('arena_from_camera')
        try:
            self.ArenaFromCamera = rospy.ServiceProxy('arena_from_camera', ArenaCameraConversion)
        except rospy.ServiceException, e:
            rospy.logwarn('Exception2 in StageArena: %s' % e)
        rospy.loginfo ('Found service: arena_from_camera.')

        rospy.loginfo ('Waiting for service: camera_from_arena...')
        rospy.wait_for_service('camera_from_arena')
        try:
            self.CameraFromArena = rospy.ServiceProxy('camera_from_arena', ArenaCameraConversion)
        except rospy.ServiceException, e:
            rospy.logwarn('Exception3 in StageArena: %s' % e)
        rospy.loginfo ('Found service: camera_from_arena.')

            
        self.nRobots = 1
            
        rospy.on_shutdown(self.OnShutdown_callback)

        
        
    def InitializeImages(self, (height,width)):
        self.imgOutput   = N.zeros([height, width, 3], dtype=N.uint8)
        self.initialized_images = True
    
    
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
        self.K = N.reshape(self.camerainfo.P,[3,4])[0:3,0:3] # camerainfo.K and camerainfo.D apply to image_raw; P w/ D=0 applies to image_rect.
        self.D = N.array([0.0, 0.0, 0.0, 0.0, 0.0])

      

    # Append 1's to the points.        
    def to_homo(self, array):
        array = N.append(array, N.ones((1, array.shape[1])), axis=0)
        return array
    
    # Remove the trailing 1 from each point.
    def from_homo(self, array):
        return array[:-1,:]
    
    
    # Draw the x/y/z axes projected onto the given image, using camera intrinsic & extrinsic parameters.
    def DrawOriginAxes(self, img, rvec, tvec):
        if self.camerainfo is not None:
            pointsAxes          = N.zeros([4, 3], dtype=N.float32)
            pointsAxesProjected = N.zeros([4, 2], dtype=N.float32)
        
            pointsAxes[0][:] = 0.0               # (0,0,0)T origin point,    pt[0] 
            pointsAxes[1][0] = self.checker_size # (1,0,0)T point on x-axis, pt[1]
            pointsAxes[2][1] = self.checker_size # (0,1,0)T point on y-axis, pt[2]
            pointsAxes[3][2] = self.checker_size # (0,0,1)T point on z-axis, pt[3]
            widthAxisLine = 3

            (pointsAxesProjected,jacobian) = cv2.projectPoints(pointsAxes,
                                                               rvec,
                                                               tvec,
                                                               self.K,
                                                               self.D
                                                               )
            
            # origin point
            pt1 = tuple(pointsAxesProjected[0][0])

            # draw x-axis
            pt2 = tuple(pointsAxesProjected[1][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(self.colorMax,0,0), widthAxisLine)

            # draw y-axis
            pt2 = tuple(pointsAxesProjected[2][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(0,self.colorMax,0), widthAxisLine)
            
            # draw z-axis
            pt2 = tuple(pointsAxesProjected[3][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(0,0,self.colorMax), widthAxisLine)
            
            

    def DrawGrid(self, img, rvec, tvec):
        if self.camerainfo is not None:
            hLinspace = (N.array(range(self.nRows)) - (self.nCols-1)/2) * self.checker_size
            hZeros = N.zeros(hLinspace.shape)
            hOnes = N.ones(hLinspace.shape) * self.checker_size * (self.nCols - 1)/2

            vLinspace = (N.array(range(self.nCols)) - (self.nRows-1)/2) * self.checker_size
            vZeros = N.zeros(vLinspace.shape)
            vOnes = N.ones(vLinspace.shape) * self.checker_size * (self.nRows - 1)/2

            hStart = N.array([-hOnes, hLinspace, hZeros]).astype('float32')
            hEnd   = N.array([ hOnes, hLinspace, hZeros]).astype('float32')
            vStart = N.array([vLinspace, -vOnes, vZeros]).astype('float32')
            vEnd   = N.array([vLinspace,  vOnes, vZeros]).astype('float32')
            
            if self.bRotateGrid:
                hStart = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(hStart)))
                hEnd   = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(hEnd)))
                vStart = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(vStart)))
                vEnd   = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(vEnd)))
                self.bRotateGrid = False
            
            widthGridline = 1
            
            (hStartProjected,jacobian) = cv2.projectPoints(hStart.transpose(),
                                              rvec,
                                              tvec,
                                              self.K,
                                              self.D)
            (hEndProjected,jacobian) = cv2.projectPoints(hEnd.transpose(),
                                              rvec,
                                              tvec,
                                              self.K,
                                              self.D)
            (vStartProjected,jacobian) = cv2.projectPoints(vStart.transpose(),
                                              rvec,
                                              tvec,
                                              self.K,
                                              self.D)
            (vEndProjected,jacobian) = cv2.projectPoints(vEnd.transpose(),
                                              rvec,
                                              tvec,
                                              self.K,
                                              self.D)

            for iLine in range(self.nRows):
                cv2.line(img, tuple(hStartProjected[iLine,:][0].astype('int32')), tuple(hEndProjected[iLine,:][0].astype('int32')), cv.CV_RGB(self.colorMax,0,0), widthGridline)
            for iLine in range(self.nCols):
                cv2.line(img, tuple(vStartProjected[iLine,:][0].astype('int32')), tuple(vEndProjected[iLine,:][0].astype('int32')), cv.CV_RGB(self.colorMax,0,0), widthGridline)
            

    def Image_callback(self, image):
        t0 = rospy.Time.now().to_sec()
        if self.initialized:
            try:
                imgCamera = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
            except CvBridgeError, e:
                rospy.logwarn('Exception CvBridgeError in image callback: %s' % e)
            
            if (not self.initialized_images):
                self.InitializeImages(imgCamera.shape)
            
            self.imgOutput = cv2.cvtColor(imgCamera, cv.CV_GRAY2RGB)
            xText = 25
            yText = 25
            dyText = 20
            
            
            if self.initialized_pose:
                if (self.nRobots>0):
                    try:
                        stamp = self.tfrx.getLatestCommonTime('Stage', 'EndEffector')
                        (posEndEffector, rotEndEffector_quat) = self.tfrx.lookupTransform('Stage', 'EndEffector', stamp)
                    except tf.Exception, e:
                        self.initialized_endeffector = False
                        rospy.logwarn ('SA Exception getting EndEffector coordinates: %s' % e)
                    else:
                        xEndEffector = posEndEffector[0]
                        yEndEffector = posEndEffector[1]
                        zEndEffector = posEndEffector[2]
                        self.initialized_endeffector = True
                else:
                    # No robots means we'll just end up with the identity transform.
                    xEndEffector = self.poseRobot.pose.position.x
                    yEndEffector = self.poseRobot.pose.position.y
                    zEndEffector = 0.0
                    self.initialized_endeffector = True
                    
                if (self.initialized_endeffector):
                    pointRobotNew       = N.array([[self.poseRobot.pose.position.x], [self.poseRobot.pose.position.y],  [0]])               # In Arena coordinates, from tracking.
                    pointEndEffectorNew = N.array([[xEndEffector],                   [yEndEffector],                    [zEndEffector]])    # In Stage coordinates, from kinematics.
                    
                    if (self.initialized_arrays):
                        pointEndEffectorPrev = self.pointEndEffector_array[:,-1].reshape((3,1))
                        if self.distPointsCriteria < tf.transformations.vector_norm((pointEndEffectorNew - pointEndEffectorPrev)):
                            self.pointRobot_array = N.append(self.pointRobot_array, pointRobotNew, axis=1)
                            self.pointEndEffector_array = N.append(self.pointEndEffector_array, pointEndEffectorNew, axis=1)
                            self.eccRobot_array = N.append(self.eccRobot_array,self.eccRobot)
                            self.areaRobot_array = N.append(self.areaRobot_array,self.areaRobot)
                    else:
                        self.pointRobot_array = pointRobotNew
                        self.pointEndEffector_array = pointEndEffectorNew
                        self.initialized_arrays = True
                    
    
                    n = self.pointRobot_array.shape[1]
                    for iPoint in range(max(0,n-10),n):
                        # Draw blue dots on prior robot locations.
                        pointImage = self.PointImageFromArena(Point(x=self.pointRobot_array[0,iPoint], y=self.pointRobot_array[1,iPoint]))                        
                        cv2.circle(self.imgOutput, 
                                   (int(pointImage.x), int(pointImage.y)), 
                                   5, cv.CV_RGB(0,0,self.colorMax/2), cv.CV_FILLED)
                        
                        # Draw green dots on prior end-effector locations.
                        pointImage = self.PointImageFromArena(Point(x=self.pointEndEffector_array[0,iPoint], y=self.pointEndEffector_array[1,iPoint]))                        
                        cv2.circle(self.imgOutput, 
                                   (int(pointImage.x), int(pointImage.y)), 
                                   5, cv.CV_RGB(0,self.colorMax/2,0), cv.CV_FILLED)
                    
                    # Draw bright blue dots on robot location.
                    pointImage = self.PointImageFromArena(self.poseRobot.pose.position)                        
                    cv2.circle(self.imgOutput, 
                               (int(pointImage.x), int(pointImage.y)), 
                               3, cv.CV_RGB(0,0,self.colorMax), cv.CV_FILLED)
                    
                    # Draw bright green dots on end-effector location.
                    pointImage = self.PointImageFromArena(Point(x=xEndEffector, y=yEndEffector))                        
                    cv2.circle(self.imgOutput, 
                               (int(pointImage.x), int(pointImage.y)), 
                               3, cv.CV_RGB(0,self.colorMax,0), cv.CV_FILLED)
    
    
                    nPoints = self.pointEndEffector_array.shape[1]
                    if (self.nPointsCriteria < nPoints):
                        
                        # The transform between Stage and Arena.
                        self.T_arena_stage = tf.transformations.superimposition_matrix(self.pointEndEffector_array, self.pointRobot_array, scaling=True)
                        self.T_stage_arena = tf.transformations.inverse_matrix(self.T_arena_stage)
                        T = self.T_stage_arena
                        
                        position = tf.transformations.translation_from_matrix(T)
                        
                        # Eccentricity.
                        eccMean = N.mean(self.eccRobot_array)
                        eccStd  = N.std(self.eccRobot_array)
                        self.eccRobotMin = eccMean - 3*eccStd
                        if self.eccRobotMin < 0:
                            self.eccRobotMin = 0
                        self.eccRobotMax = eccMean + 3*eccStd
                        # rospy.logwarn('eccMean = %s, eccStd = %s\n' % (eccMean, eccStd))
                        
                        # Area.
                        areaMean = N.mean(self.areaRobot_array)
                        areaStd  = N.std(self.areaRobot_array)
                        self.areaRobotMin = areaMean - 3*areaStd
                        if self.areaRobotMin < 0:
                            self.areaRobotMin = 0
                        self.areaRobotMax = areaMean + 3*areaStd
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
                        display_text = 'eccRobotMin = %0.3f, eccRobotMax = %0.3f' % (self.eccRobotMin, self.eccRobotMax)
                        cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                        yText += dyText
                        display_text = 'areaRobotMin = %0.0f, areaRobotMax = %0.0f' % (self.areaRobotMin, self.areaRobotMax)
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
                        #self.DrawOriginAxes(self.imgOutput, self.rvec, self.tvec)
                        #self.DrawGrid(self.imgOutput, self.rvec, self.tvec)
                    else:
                        yText += 12*dyText
                        
                            
                    yText += dyText
                    display_text = 'positionRobot = [%0.3f, %0.3f]' % (self.poseRobot.pose.position.x, self.poseRobot.pose.position.y)
                    cv2.putText(self.imgOutput,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    display_text = 'positionEndEffector = [%0.3f, %0.3f]' % (xEndEffector, yEndEffector)
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
            
            cv2.imshow('Stage/Arena Calibration', self.imgOutput)
            cv2.waitKey(3)
        t1 = rospy.Time.now().to_sec()
        #rospy.logwarn('time=%f' % (t1-t0))

   
    def ContourinfoLists_callback(self, contourinfolists):
        if self.initialized:
            header     = contourinfolists.header
            x_list     = contourinfolists.x
            y_list     = contourinfolists.y
            angle_list = contourinfolists.angle
            area_list  = contourinfolists.area
            ecc_list   = contourinfolists.ecc
            nContours  = len(x_list)
            
            if (nContours==1):
                self.textError = ''
                if not self.initialized_pose:
                    self.initialized_pose = True

                poseContour = PoseStamped(header=contourinfolists.header,
                                        pose=Pose(position=Point(x=x_list[0],
                                                                 y=y_list[0])) 
                                        )

                self.poseRobot.header = contourinfolists.header
                self.poseRobot.header.frame_id = 'Arena'
                self.poseRobot.pose = self.PoseArenaFromImage(poseContour) # Gives the position of the robot in arena coordinates.
                self.areaRobot = area_list[0]
                self.eccRobot  = ecc_list[0]
                
                self.xMin = min(self.xMin, self.poseRobot.pose.position.x)
                self.xMax = max(self.xMax, self.poseRobot.pose.position.x)
                self.yMin = min(self.yMin, self.poseRobot.pose.position.y)
                self.yMax = max(self.yMax, self.poseRobot.pose.position.y)
                
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

        while (not rospy.is_shutdown()):
            msgPattern.direction *= -1
            self.pubSetPattern.publish (msgPattern)
            rospy.sleep(60)
        
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

