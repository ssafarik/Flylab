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
from pythonmodules import cvNumpy,CameraParameters
from geometry_msgs.msg import Point, PoseStamped
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
        self.initialized_endeffector = False
        rospy.init_node('CalibrateStageArena')
        
        self.cvbridge = CvBridge()

        self.tfrx = tf.TransformListener()
        self.subCameraInfo          = rospy.Subscriber('camera/camera_info', CameraInfo,       self.CameraInfo_callback)
        self.subImage               = rospy.Subscriber('camera/image_rect',  Image,            self.Image_callback)
        self.subContourinfoLists    = rospy.Subscriber('ContourinfoLists',   ContourinfoLists, self.ContourinfoLists_callback)
        self.subEndEffector         = rospy.Subscriber('EndEffector',        MsgFrameState,    self.EndEffector_callback)
        self.pubPatternGen          = rospy.Publisher('SetSignalGen',        MsgPattern, latch=True)

        self.camerainfo = None
        
        cv2.namedWindow('Stage/Arena Calibration', 1)
        
        self.poseRobot_camera = PoseStamped()
        self.poseRobot_rect = PoseStamped()
        self.poseRobot_arena = PoseStamped()
        
        self.colorMax = 255
        self.colorFont = cv.CV_RGB(self.colorMax,0,0)
        self.textError = ''
        
        self.point_count_min = 20 #60
        self.dist_min = 10

        self.eccRobot = 0
        self.areaRobot = 0
        self.eccRobot_array = N.array([])
        self.areaRobot_array = N.array([])
        self.eccRobotMin = 1000000000
        self.eccRobotMax = 0
        self.areaRobotMin = 1000000000
        self.areaRobotMax = 0
        
                # Checkerboard info
        self.sizeCheckerboard = (8,6)
        self.nCols = self.sizeCheckerboard[1]
        self.nRows = self.sizeCheckerboard[0]
        self.nCorners = self.nCols * self.nRows
        self.checker_size = rospy.get_param('calibration/checker_size', 15)
        self.nGridlines = 9
        self.rotate_grid = False
        
        self.pointsAxes                     = N.zeros([4, 3], dtype=N.float32)
        self.pointsAxesProjected            = N.zeros([4, 2], dtype=N.float32)
        #self.pointsGridlineStartProjected   = N.zeros([nGridlines, 2], dtype=N.float32)
        #self.pointsGridlineEndProjected     = N.zeros([nGridlines, 2], dtype=N.float32)
        
        self.rvec      = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.tvec      = N.zeros([1, 3], dtype=N.float32).squeeze()

        rospy.logwarn ('Waiting for ImageRect transform...')
        while True:
            try:
                (self.camera_rect_trans, rot) = self.tfrx.lookupTransform('/ImageRect', '/Camera', rospy.Time(0))
                break
            except tf.LookupException:
                pass
        rospy.logwarn ('Found ImageRect transform.')
        

        rospy.logwarn ('Waiting for service: arena_from_camera...')
        rospy.wait_for_service('arena_from_camera')
        try:
            self.arena_from_camera = rospy.ServiceProxy('arena_from_camera', ArenaCameraConversion)
        except rospy.ServiceException, e:
            rospy.logwarn('Exception in StageArena: %s' % e)
        rospy.logwarn ('Found service: arena_from_camera.')
            
        self.nRobots = rospy.get_param('nRobots', 0)
        if (self.nRobots==0):
            self.initialized_endeffector = True
            
        rospy.on_shutdown(self.OnShutdown_callback)

        
        
    def InitializeImages(self, (height,width)):
        self.imgDisplay   = N.zeros([height, width, 3], dtype=N.uint8)
        self.initialized_images = True
    
    
    def OnShutdown_callback(self):
        if self.initialized:
            msgPattern = MsgPattern()
            msgPattern.mode = 'byshape'
            msgPattern.shape = 'constant'
            msgPattern.points = []
            msgPattern.frame_id = 'Stage'
            msgPattern.hzPattern = 1.0
            msgPattern.hzPoint = 100
            msgPattern.count = 1
            msgPattern.size = Point(x=0,y=0)
            msgPattern.preempt = True
            msgPattern.param = 0
            self.pubPatternGen.publish (msgPattern)
        
    
    def EndEffector_callback (self, state):
        self.initialized_endeffector = True
        
    
    def CameraInfo_callback (self, msgCameraInfo):
        if self.camerainfo is None:
            self.camerainfo = msgCameraInfo

            self.rvec[0] = rospy.get_param('/camera_arena_rvec_0')
            self.rvec[1] = rospy.get_param('/camera_arena_rvec_1')
            self.rvec[2] = rospy.get_param('/camera_arena_rvec_2')
            self.tvec[0] = rospy.get_param('/camera_arena_tvec_0')
            self.tvec[1] = rospy.get_param('/camera_arena_tvec_1')
            self.tvec[2] = rospy.get_param('/camera_arena_tvec_2')
            
            #self.tvec[0] = 0.0
            #self.tvec[1] = 0.0
            #self.tvec[2] = 0.0
            #rospy.logwarn(self.tvec)
      
      
        
    def to_homo(self, array):
        array = N.append(array, N.ones((1, array.shape[1])), axis=0)
        return array
    
    
    def from_homo(self, array):
        return array[:-1,:]
    
    
    # Draw the x/y/z axes projected onto the given image, using camera intrinsic & extrinsic parameters.
    def DrawOriginAxes(self, img, rvec, tvec):
        if self.camerainfo is not None:
            self.pointsAxes       = N.zeros([4, 3], dtype=N.float32) #cv.CreateMat(4, 3, cv.CV_32FC1)
            self.pointsAxes[0][:] = 0.0               # (0,0,0)T origin point,    pt[0] 
            self.pointsAxes[1][0] = self.checker_size # (1,0,0)T point on x-axis, pt[1]
            self.pointsAxes[2][1] = self.checker_size # (0,1,0)T point on y-axis, pt[2]
            self.pointsAxes[3][2] = self.checker_size # (0,0,1)T point on z-axis, pt[3]
            widthAxisLine = 3

            (self.pointsAxesProjected,jacobian) = cv2.projectPoints(self.pointsAxes,
                                                                    rvec,
                                                                    tvec,
                                                                    N.reshape(self.camerainfo.K, [3,3]),
                                                                    N.reshape(self.camerainfo.D, [5,1])
                                                                    )
            
            # origin point
            pt1 = tuple(self.pointsAxesProjected[0][0])

            # draw x-axis
            pt2 = tuple(self.pointsAxesProjected[1][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(self.colorMax,0,0), widthAxisLine)

            # draw y-axis
            pt2 = tuple(self.pointsAxesProjected[2][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(0,self.colorMax,0), widthAxisLine)
            
            # draw z-axis
            pt2 = tuple(self.pointsAxesProjected[3][0])
            cv2.line(img, pt1, pt2, cv.CV_RGB(0,0,self.colorMax), widthAxisLine)
            
            

    def DrawGrid(self, img):
        if self.camerainfo is not None:
            #points_range = (N.array(range(nGridlines)) - (nGridlines-1)/2) * checker_size
            #points_zeros = N.zeros(points_range.shape)
            #points_ones = N.ones(points_range.shape) * checker_size * (nGridlines - 1)/2
            #x_start_points_array = N.array([-points_ones, points_range, points_zeros]).astype('float32')
            #x_end_points_array = N.array([ points_ones, points_range, points_zeros]).astype('float32')
            #y_start_points_array = N.array([points_range, -points_ones, points_zeros]).astype('float32')
            #y_end_points_array = N.array([points_range, points_ones, points_zeros]).astype('float32')
            #if self.rotate_grid:
            #    x_start_points_array = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(x_start_points_array)))
            #    x_end_points_array = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(x_end_points_array)))
            #    y_start_points_array = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(y_start_points_array)))
            #    y_end_points_array = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(y_end_points_array)))
            #    self.rotate_grid = False
            #x_start_points = cvNumpy.array_to_mat(x_start_points_array.transpose())
            #x_end_points = cvNumpy.array_to_mat(x_end_points_array.transpose())
            #y_start_points = cvNumpy.array_to_mat(y_start_points_array.transpose())
            #y_end_points = cvNumpy.array_to_mat(y_end_points_array.transpose())

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
            
            if self.rotate_grid:
                hStart = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(hStart)))
                hEnd   = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(hEnd)))
                vStart = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(vStart)))
                vEnd   = self.from_homo(N.dot(self.T_stage_arena, self.to_homo(vEnd)))
                self.rotate_grid = False
            
            widthGridline = 1
            
            (hStartProjected,jacobian) = cv2.projectPoints(hStart.transpose(),
                                              self.rvec,
                                              self.tvec,
                                              N.reshape(self.camerainfo.K,[3,3]),
                                              N.reshape(self.camerainfo.D,[5,1]))
            (hEndProjected,jacobian) = cv2.projectPoints(hEnd.transpose(),
                                              self.rvec,
                                              self.tvec,
                                              N.reshape(self.camerainfo.K,[3,3]),
                                              N.reshape(self.camerainfo.D,[5,1]))
            (vStartProjected,jacobian) = cv2.projectPoints(vStart.transpose(),
                                              self.rvec,
                                              self.tvec,
                                              N.reshape(self.camerainfo.K,[3,3]),
                                              N.reshape(self.camerainfo.D,[5,1]))
            (vEndProjected,jacobian) = cv2.projectPoints(vEnd.transpose(),
                                              self.rvec,
                                              self.tvec,
                                              N.reshape(self.camerainfo.K,[3,3]),
                                              N.reshape(self.camerainfo.D,[5,1]))
            
            for iLine in range(self.nRows):
                cv2.line(img, tuple(hStartProjected[iLine,:][0].astype('int32')), tuple(hEndProjected[iLine,:][0].astype('int32')), cv.CV_RGB(self.colorMax,0,0), widthGridline)
            for iLine in range(self.nCols):
                cv2.line(img, tuple(vStartProjected[iLine,:][0].astype('int32')), tuple(vEndProjected[iLine,:][0].astype('int32')), cv.CV_RGB(self.colorMax,0,0), widthGridline)
            

    def Image_callback(self, image):
        if self.initialized:
            try:
                imgInput = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, "passthrough")))
            except CvBridgeError, e:
                rospy.logwarn('Exception CvBridgeError in image callback: %s' % e)
            
            if not self.initialized_images:
                self.InitializeImages(imgInput.shape)
            
            self.imgDisplay = cv2.cvtColor(imgInput, cv.CV_GRAY2RGB)
            xText = 25
            yText = 25
            dyText = 20
            
            
            if self.initialized_pose:
                if self.initialized_arrays:
                    image_point_num = self.image_point_array.shape[1]
                    for image_point_n in range(image_point_num):
                        cv2.circle(self.imgDisplay, 
                                   (int(self.image_point_array[0,image_point_n]),int(self.image_point_array[1,image_point_n])), 
                                   5, cv.CV_RGB(self.colorMax,0,self.colorMax), cv.CV_FILLED)
                
                cv2.circle(self.imgDisplay, 
                           (int(self.poseRobot_rect.pose.position.x),int(self.poseRobot_rect.pose.position.y)), 
                           3, cv.CV_RGB(0,0,self.colorMax), cv.CV_FILLED)
                # display_text = 'poseRobot_camera.x = ' + str(round(self.poseRobot_camera.pose.position.x,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                # display_text = 'poseRobot_camera.y = ' + str(round(self.poseRobot_camera.pose.position.y,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                
                display_text = 'RobotImage Pose with Respect to Arena = [%0.3f, %0.3f]' % (self.poseRobot_arena.pose.position.x,self.poseRobot_arena.pose.position.y)
                cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                # display_text = 'poseRobot_arena.x = ' + str(round(self.poseRobot_arena.pose.position.x,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                # display_text = 'poseRobot_arena.y = ' + str(round(self.poseRobot_arena.pose.position.y,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                
                # display_text = 'stage_state.x = ' + str(round(self.stage_state.x,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                # display_text = 'stage_state.y = ' + str(round(self.stage_state.y,3))
                # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                #yText += dyText
                
                if (self.nRobots>0):
                    try:
                        (trans,rot_quat) = self.tfrx.lookupTransform('Stage', 'EndEffector', rospy.Time(0))
                    except tf.LookupException, e:
                        rospy.logwarn ('Exception in StageArena: %s' % e)
                    else:
                        xEndEffector = trans[0]
                        yEndEffector = trans[1]
                        zEndEffector = trans[2]
                else:
                    xEndEffector = self.poseRobot_arena.pose.position.x
                    yEndEffector = self.poseRobot_arena.pose.position.y
                    zEndEffector = 0.0
                    
                image_point_new = N.array([[self.poseRobot_rect.pose.position.x], [self.poseRobot_rect.pose.position.y]])
                arena_point_new = N.array([[self.poseRobot_arena.pose.position.x], [self.poseRobot_arena.pose.position.y],[0]])
                stage_point_new = N.array([[xEndEffector], [yEndEffector], [zEndEffector]])
                
                display_text = 'EndEffector Pose with Respect to Stage = [%0.3f, %0.3f]' % (xEndEffector, yEndEffector)
                cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                if self.initialized_arrays:
                    arena_point_prev = self.arena_point_array[:,-1].reshape((3,1))
                    # rospy.logwarn('arena_point_prev = \n%s', str(arena_point_prev))
                    if self.dist_min < tf.transformations.vector_norm((arena_point_new - arena_point_prev)):
                        self.image_point_array = N.append(self.image_point_array,image_point_new,axis=1)
                        self.arena_point_array = N.append(self.arena_point_array,arena_point_new,axis=1)
                        # rospy.logwarn('arena_point_array = \n%s', str(self.arena_point_array))
                        self.stage_point_array = N.append(self.stage_point_array,stage_point_new,axis=1)
                        # rospy.logwarn('stage_point_array = \n%s', str(self.stage_point_array))
                        self.eccRobot_array = N.append(self.eccRobot_array,self.eccRobot)
                        self.areaRobot_array = N.append(self.areaRobot_array,self.areaRobot)
                else:
                    self.image_point_array = image_point_new
                    self.arena_point_array = arena_point_new
                    self.stage_point_array = stage_point_new
                    self.initialized_arrays = True
                
                if self.point_count_min < self.arena_point_array.shape[1]:
                    self.T_arena_stage = tf.transformations.superimposition_matrix(self.arena_point_array, self.stage_point_array, scaling=True)
                    self.T_stage_arena = tf.transformations.inverse_matrix(self.T_arena_stage)
                    # self.T_stage_arena = tf.transformations.superimposition_matrix(self.arena_point_array, self.stage_point_array)
                    # self.T_arena_stage = tf.transformations.inverse_matrix(self.T_stage_arena)
                    # tvector = tf.transformations.translation_from_matrix(self.T_arena_stage)
                    tvector = tf.transformations.translation_from_matrix(self.T_stage_arena)
                    display_text = 'Translation Vector = [%0.3f, %0.3f, %0.3f]' % (tvector[0],tvector[1],tvector[2])
                    cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    # rospy.logwarn('tvec = \n%s',str(tvec))
                    # quaternion = tf.transformations.quaternion_from_matrix(self.T_arena_stage)
                    quaternion = tf.transformations.quaternion_from_matrix(self.T_stage_arena)
                    display_text = 'Quaternion = [%0.3f, %0.3f, %0.3f, %0.3f]' % (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                    cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    # rospy.logwarn('quaternion = \n%s',str(quaternion))
                    # euler = tf.transformations.euler_from_matrix(self.T_arena_stage,'rxyz')
                    # display_text = 'Translation Vector = [%0.2f, %0.2f, %0.2f]' % (self.tvec[0],
                    # cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    #yText += dyText
                    # rospy.logwarn('euler = \n%s',str(euler))
                    
                    eccMean = N.mean(self.eccRobot_array)
                    eccStd = N.std(self.eccRobot_array)
                    # rospy.logwarn('eccMean = %s, eccStd = %s\n' % (eccMean, eccStd))
                    self.eccRobotMin = eccMean - eccStd*3
                    if self.eccRobotMin < 0:
                        self.eccRobotMin = 0
                    self.eccRobotMax = eccMean + eccStd*3
                    areaMean = N.mean(self.areaRobot_array)
                    areaStd = N.std(self.areaRobot_array)
                    # rospy.logwarn('areaMean = %s, areaStd = %s\n' % (areaMean, areaStd))
                    self.areaRobotMin = areaMean - areaStd*3
                    if self.areaRobotMin < 0:
                        self.areaRobotMin = 0
                    self.areaRobotMax = areaMean + areaStd*3
                    display_text = 'eccRobotMin = %0.3f, eccRobotMax = %0.3f' % (self.eccRobotMin, self.eccRobotMax)
                    cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    display_text = 'areaRobotMin = %0.0f, areaRobotMax = %0.0f' % (self.areaRobotMin, self.areaRobotMax)
                    cv2.putText(self.imgDisplay,display_text,(xText,yText),cv.CV_FONT_HERSHEY_TRIPLEX, 0.5,self.colorFont)
                    yText += dyText
                    
                    factor, origin, direction = tf.transformations.scale_from_matrix(self.T_stage_arena)
                    display_text = 'Factor = %s' % factor
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'Origin = %s' % origin
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'Direction = %s' % direction
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'T_stage_arena = %s' % self.T_stage_arena[0]
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'T_stage_arena = %s' % self.T_stage_arena[1]
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'T_stage_arena = %s' % self.T_stage_arena[2]
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    display_text = 'T_stage_arena = %s' % self.T_stage_arena[3]
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    
                    self.rotate_grid = True
                    self.DrawGrid(self.imgDisplay)
                
                    self.DrawOriginAxes(self.imgDisplay, self.rvec, self.tvec)
                    self.DrawGrid(self.imgDisplay)
                    
        
            
            cv2.putText(self.imgDisplay, self.textError, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
            yText += dyText
            
            cv2.imshow('Stage Arena Calibration', self.imgDisplay)
            cv2.waitKey(3)

   
    def ContourinfoLists_callback(self,contourinfo_lists):
        if self.initialized:
            header = contourinfo_lists.header
            x_list = contourinfo_lists.x
            y_list = contourinfo_lists.y
            angle_list = contourinfo_lists.angle
            area_list = contourinfo_lists.area
            ecc_list = contourinfo_lists.ecc
            contour_count = min(len(x_list),len(y_list),len(angle_list),len(area_list),len(ecc_list))
            
            if contour_count==1:
                self.textError = ''
                if not self.initialized_pose:
                    self.initialized_pose = True
                  
                self.poseRobot_camera.header = contourinfo_lists.header
                self.poseRobot_camera.pose.position.x = x_list[0]
                self.poseRobot_camera.pose.position.y = y_list[0]
                self.poseRobot_rect = self.PoseRectFromCamera(self.poseRobot_camera)
                self.poseRobot_arena = self.PoseArenaFromCamera(self.poseRobot_camera)
                self.areaRobot = area_list[0]
                self.eccRobot = ecc_list[0]
            elif contour_count==0:
                #rospy.logwarn('ERROR:  No objects detected.')
                self.textError = 'ERROR:  No objects detected.'
            elif contour_count>1:
                #rospy.logwarn('ERROR:  More than one object detected.')
                self.textError = 'ERROR:  More than one object detected.'
            

    def PoseRectFromCamera(self,poseCamera):
        poseRect = PoseStamped()
        poseRect.pose.position.x = self.camera_rect_trans[0] + poseCamera.pose.position.x
        poseRect.pose.position.y = self.camera_rect_trans[1] + poseCamera.pose.position.y

        return poseRect
    
    
    def PoseArenaFromCamera(self, poseCamera):
        response = self.arena_from_camera([poseCamera.pose.position.x],[poseCamera.pose.position.y])
        poseArena = PoseStamped()
        poseArena.pose.position.x = response.Xdst[0]
        poseArena.pose.position.y = response.Ydst[0]
        
        return poseArena
    
    
    def Main(self):
        rospy.sleep(2)
        msgPattern = MsgPattern()

        # Publish a goto(0,0) pattern message, and wait for the end effector to initialize.
        msgPattern.mode = 'byshape'
        msgPattern.shape = 'constant'
        msgPattern.points = []
        msgPattern.frame_id = 'Stage'
        msgPattern.hzPattern = 1.0
        msgPattern.hzPoint = rospy.get_param('actuator/hzPoint', 100.0)
        msgPattern.count = 1
        msgPattern.size = Point(x=0,y=0)
        msgPattern.preempt = True
        msgPattern.param = 0
        self.pubPatternGen.publish (msgPattern)
        while not self.initialized_endeffector:
            rospy.sleep(0.5)
        rospy.sleep(2)
        self.initialized = True

        # Publish the spiral pattern message.
        msgPattern.mode = 'byshape'
        msgPattern.shape = rospy.get_param('calibration/shape', 'spiral')
        msgPattern.points = []
        msgPattern.frame_id = 'Stage'
        msgPattern.count = -1
        msgPattern.size = Point(x=0.7 * rospy.get_param('arena/radius_inner', 25.4),y=0)
        #msgPattern.radius = 0.8 * rospy.get_param('arena/radius_inner', 25.4)
        msgPattern.preempt = True
        msgPattern.param = 0
        if msgPattern.shape=='spiral':
            msgPattern.hzPattern = 0.01
        else:
            msgPattern.hzPattern = 0.1
        self.pubPatternGen.publish (msgPattern)

        rospy.spin()
        
        # Publish a goto(0,0) pattern message.
        msgPattern.mode = 'byshape'
        msgPattern.shape = 'constant'
        msgPattern.points = []
        msgPattern.frame_id = 'Stage'
        msgPattern.hzPattern = 1.0
        msgPattern.count = 1
        msgPattern.size = Point(x=0,y=0)
        msgPattern.preempt = True
        msgPattern.param = 0
        self.pubPatternGen.publish (msgPattern)
    
    

if __name__ == '__main__':
    cal = CalibrateStageArena()
    cal.Main()
    cv2.destroyAllWindows()

