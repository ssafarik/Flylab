#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('calibration')
import sys
import rospy
import cv
import tf
import math
import numpy as N
from pythonmodules import cvNumpy,CameraParameters
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
#from joystick_commands.msg import JoystickCommands
from cv_bridge import CvBridge, CvBridgeError

FRAME_CHECKERBOARD="Checkerboard"
FRAME_IMAGERECT="ImageRect"
FRAME_PLATE="Plate"
FRAME_CAMERA="Camera"

class CalibrateCameraPlate:

    def __init__(self):
        self.initialized = False
        rospy.init_node('CalibrateCameraPlate')

        self.initialized_images = False
        cv.NamedWindow("Camera Plate Calibration", 1)
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        self.bridge = CvBridge()

        
        self.camerainfo = None
        
        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.font_color = cv.CV_RGB(self.color_max,0,0)
        
        self.originPlate = PointStamped()
        self.originPlate.header.frame_id = FRAME_CAMERA
        
        self.originCamera = PointStamped()
        self.originCamera.header.frame_id = FRAME_CAMERA
        self.originCamera.point.x = 0
        self.originCamera.point.y = 0
        # self.image_plate_origin_found = False
        # self.plate_origin = PointStamped()
        # self.plate_origin.header.frame_id = FRAME_PLATE
        # self.plate_origin.point.x = 0
        # self.plate_origin.point.y = 0
        
        #(self.intrinsic_matrix,self.distortion_coeffs) = CameraParameters.intrinsic("rect")
        #self.KK_cx = self.intrinsic_matrix[0,2]
        #self.KK_cy = self.intrinsic_matrix[1,2]
        
        self.subCameraInfo = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        self.subImage = rospy.Subscriber("camera/image_rect", Image, self.image_callback)
        #self.subJoystick = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joy_callback)
        self.subPoint = rospy.Subscriber("Joystick/Commands", Point, self.point_callback)
        
        
        # Pattern info
        self.pattern_size = (8,6)
        self.col_corner_number = self.pattern_size[0]
        self.row_corner_number = self.pattern_size[1]
        self.board_corner_number = self.col_corner_number * self.row_corner_number
        self.checker_size = rospy.get_param('calibration/checker_size', 7.5)
        self.win = (4,4)
        self.zero_zone = (2,2)
        self.criteria = (cv.CV_TERMCRIT_ITER+cv.CV_TERMCRIT_EPS,100,.01)
        
        self.image_points = cv.CreateMat(self.board_corner_number, 2, cv.CV_32FC1)
        self.plate_points = cv.CreateMat(self.board_corner_number, 3, cv.CV_32FC1)
        self.point_counts = cv.CreateMat(1, 1, cv.CV_32SC1)
        self.rvec = cv.CreateMat(1, 3, cv.CV_32FC1)
        self.tvec = cv.CreateMat(1, 3, cv.CV_32FC1)
        self.rvec_sum = [0,0,0]
        self.tvec_sum = [0,0,0]
        self.n = 0
        
        self.origin_points = cv.CreateMat(4, 3, cv.CV_32FC1)
        self.origin_points_projected = cv.CreateMat(4, 2, cv.CV_32FC1)
        self.initialized = True
    
    
    def CameraInfo_callback (self, msgCameraInfo):
        self.camerainfo = msgCameraInfo
      
        
    def initialize_images(self, cv_image):
        if self.camerainfo is not None:
            self.im_size = cv.GetSize(cv_image)
            (self.im_width,self.im_height) = cv.GetSize(cv_image)
            self.im = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
            self.im_mask = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
            self.im_display = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)
            self.originPlate.point.x = self.im_width/2 - self.camerainfo.P[2] #self.KK_cx
            self.originPlate.point.y = self.im_height/2 - self.camerainfo.P[6] #self.KK_cy
            self.radiusMask = int(self.im_height * .48)
            self.initialized_images = True

    def add_board_to_data(self,corners):
        self.capture_count = 0
        step = self.capture_count * self.board_corner_number
        for corner_count in range(self.board_corner_number):
            (x,y) = corners[corner_count]
            cv.SetReal2D(self.image_points, step, 0, x)
            cv.SetReal2D(self.image_points, step, 1, y)
            cv.SetReal2D(self.plate_points, step, 0, (corner_count // self.col_corner_number)*self.checker_size)
            cv.SetReal2D(self.plate_points, step, 1, (corner_count % self.col_corner_number)*self.checker_size)
            cv.SetReal2D(self.plate_points, step, 2, 0.0)
            step += 1
        cv.SetReal2D(self.point_counts, self.capture_count, 0, self.board_corner_number)

    def draw_origin(self,im_color):
        if self.camerainfo is not None:
            cv.SetZero(self.origin_points)
            cv.SetReal2D(self.origin_points,1,0,self.checker_size) # x-direction
            cv.SetReal2D(self.origin_points,2,1,self.checker_size) # y-direction
            cv.SetReal2D(self.origin_points,3,2,self.checker_size) # z-direction
            axis_line_width = 3
            rect_line_width = 2
            cvmatK = cv.fromarray(N.reshape(self.camerainfo.K,[3,3]))
            cvmatD = cv.fromarray(N.reshape(self.camerainfo.D,[5,1]))
            cv.ProjectPoints2(self.origin_points,
                              self.rvec,
                              self.tvec,
                              cvmatK, #self.intrinsic_matrix,
                              cvmatD, #self.distortion_coeffs,
                              self.origin_points_projected)
            
            try:
                a = cvNumpy.mat_to_array(self.origin_points_projected)
                
                # origin point
                pt1 = tuple(a[0])
                
                # draw x-axis
                pt2 = tuple(a[1])
                cv.Line(im_color,pt1,pt2,cv.CV_RGB(self.color_max,0,0),axis_line_width)
                
                # draw y-axis
                pt2 = tuple(a[2])
                cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,self.color_max,0),axis_line_width)
                
                # draw z-axis
                pt2 = tuple(a[3])
                cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,0,self.color_max),axis_line_width)
                
                # if self.image_plate_origin_found:
                #   self.image_plate_origin = pt1
                #   self.image_plate_origin_found = False
                
                # display_text = "image_plate_origin = " + str(self.image_plate_origin)
                # cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)
                # display_text = "image_plate_origin = (%0.0f, %0.0f)" % (self.image_plate_origin[0],self.image_plate_origin[1])
                # cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)
            except:
                pass
            

    def find_extrinsics(self):
        if self.camerainfo is not None:

            # Update image mask
            cv.Circle(self.im_mask,
                      (int(self.undistorted_plate.point.x), int(self.undistorted_plate.point.y)), 
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            cv.And(self.im, self.im_mask, self.im)
            (statusCorners, corners) = cv.FindChessboardCorners(self.im, self.pattern_size)
            if statusCorners and (len(corners) == self.board_corner_number):
                sub_corners = cv.FindCornerSubPix(self.im, corners, self.win, self.zero_zone, self.criteria)
                cv.DrawChessboardCorners(self.im_display, self.pattern_size, sub_corners, statusCorners)
                self.add_board_to_data(corners)
                cvmatK = cv.fromarray(N.reshape(self.camerainfo.K,[3,3]))
                cvmatD = cv.fromarray(N.reshape(self.camerainfo.D,[5,1]))
                cv.FindExtrinsicCameraParams2(self.plate_points,
                                              self.image_points,
                                              cvmatK, #self.intrinsic_matrix,
                                              cvmatD, #self.distortion_coeffs,
                                              self.rvec,
                                              self.tvec)

                self.rvec_array = N.array(self.rvec).squeeze() #N.reshape(self.rvec,[1,3]) #cvNumpy.mat_to_array(self.rvec).squeeze()
                self.tvec_array = N.array(self.tvec).squeeze() #N.reshape(self.tvec,[1,3]) #cvNumpy.mat_to_array(self.tvec).squeeze()
                
                angleRvec = N.linalg.norm(self.rvec_array)
                R = tf.transformations.rotation_matrix(angleRvec, self.rvec_array)
                T = tf.transformations.translation_matrix(self.tvec_array)
                
                Wsub = N.zeros((3,3))
                Wsub[:,:-1] = R[:-1,:-2]
                Wsub[:,-1] = T[:-1,-1]
                
                M = N.reshape(self.camerainfo.K,[3,3])
                M[:-1,-1] = 0
                
                Hinv = N.dot(M, Wsub)
                Hinv = Hinv / Hinv[-1,-1]
                H = N.linalg.inv(Hinv)
                self.Hinv = Hinv
                self.H = H
                
                now = rospy.Time.now()
                
                self.tfbx.sendTransform(self.tvec_array,
                                          tf.transformations.quaternion_about_axis(angleRvec, self.rvec_array),
                                          now,
                                          FRAME_CHECKERBOARD,
                                          FRAME_IMAGERECT)
                
                # rospy.logwarn("H = \n%s", str(H))
                
                # rvec_array = cvNumpy.mat_to_array(self.rvec)
                # display_text = "rvec = " + str(rvec_array[0])
                # cv.PutText(self.im_display,display_text,(25,420),self.font,self.font_color)
                # tvec_array = cvNumpy.mat_to_array(self.tvec)
                # display_text = "tvec = " + str(tvec_array[0])
                # cv.PutText(self.im_display,display_text,(25,440),self.font,self.font_color)
                # self.draw_origin(self.im_display)
                # rvec_array = N.array([-3.06,-0.0014,0.0042])
                # tvec_array = N.array([3.9,46.7,335.9])
                # rvec_array = rvec_array.astype('float32')
                # tvec_array = tvec_array.astype('float32')
                # self.rvec = cvNumpy.array_to_mat(rvec_array)
                # self.tvec = cvNumpy.array_to_mat(tvec_array)
                self.draw_origin(self.im_display)
                self.remap_extrinsics(now)


    def remap_extrinsics(self, now):
        X = [self.originPlate.point.x, self.originPlate.point.x + self.checker_size]
        Y = [self.originPlate.point.y, self.originPlate.point.y]
        Z = [1,                         1]
        
        camera_points = N.array([X,Y,Z])
        plate_points = N.dot(self.H, camera_points)
        x0 = plate_points[0,0]
        x1 = plate_points[0,1]
        y0 = plate_points[1,0]
        y1 = plate_points[1,1]
        angleRot = N.arctan2((y1-y0),(x1-x0))
        
        # rospy.logwarn("plate_points = %s", str(plate_points))
        checkerboard_plate_vector = plate_points[:,0]
        checkerboard_plate_vector[2] = 0
        
        self.tfbx.sendTransform(checkerboard_plate_vector,
                                  tf.transformations.quaternion_from_euler(0, 0, angleRot),
                                  now,
                                  FRAME_PLATE,
                                  FRAME_CHECKERBOARD)
        
        try:
            # self.image_plate_origin = self.tfrx.transformPoint("Image",self.plate_origin)
            self.tfrx.waitForTransform(FRAME_IMAGERECT, FRAME_PLATE, now, rospy.Duration(1.0))
            (trans, quat) = self.tfrx.lookupTransform(FRAME_IMAGERECT, FRAME_PLATE, now)
            # rospy.logwarn("trans = %s", str(trans))
            # rospy.logwarn("rot = %s", str(rot))

            rot_array = tf.transformations.quaternion_matrix(quat)
            rot_array = rot_array[0:3,0:3]
            rot_array = rot_array.astype('float32')
            rot_mat = cvNumpy.array_to_mat(rot_array)
            cv.Rodrigues2(rot_mat, self.rvec)
            self.rvec_array = cvNumpy.mat_to_array(self.rvec).squeeze()
    
            self.tvec_array = N.array(trans)
            self.rvec = cvNumpy.array_to_mat(self.rvec_array)
            self.tvec = cvNumpy.array_to_mat(self.tvec_array)

            self.rvec_sum[0] += self.rvec_array[0]
            self.rvec_sum[1] += self.rvec_array[1]
            self.rvec_sum[2] += self.rvec_array[2]
            self.tvec_sum[0] += self.tvec_array[0]
            self.tvec_sum[1] += self.tvec_array[1]
            self.tvec_sum[2] += self.tvec_array[2]
            self.n += 1
            
            display_text = "checker_size=%0.3f" % self.checker_size
            cv.PutText(self.im_display,
                       display_text,
                       (25,400),
                       self.font,
                       self.font_color)

            display_text = "rvec=[%-0.3f, %-0.3f, %-0.3f] avg=[%-0.3f, %-0.3f, %-0.3f]" % (self.rvec_array[0],      self.rvec_array[1],      self.rvec_array[2],
                                                                                     self.rvec_sum[0]/self.n, self.rvec_sum[1]/self.n, self.rvec_sum[2]/self.n)
            cv.PutText(self.im_display,
                       display_text,
                       (25,420),
                       self.font,
                       self.font_color)
            #display_text = "tvec = [%0.3f, %0.3f, %0.3f]" % (self.tvec_array[0], self.tvec_array[1], self.tvec_array[2])
            display_text = "tvec=[%-0.3f, %-0.3f, %-0.3f] avg=[%-0.3f, %-0.3f, %-0.3f]" % (self.tvec_array[0],      self.tvec_array[1],      self.tvec_array[2],
                                                                                     self.tvec_sum[0]/self.n, self.tvec_sum[1]/self.n, self.tvec_sum[2]/self.n)
            cv.PutText(self.im_display,
                       display_text,
                       (25,440),
                       self.font,
                       self.font_color)
            # self.image_plate_origin_found = True
            self.draw_origin(self.im_display)
        except (tf.Exception):
            pass


    def image_callback(self,data):
        if self.initialized:
            try:
                cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
            except CvBridgeError, e:
                print e
            
            if not self.initialized_images:
                self.initialize_images(cv_image)
            
            if self.initialized_images:
                self.im = cv.CloneImage(cv_image)
                cv.CvtColor(cv_image, self.im_display, cv.CV_GRAY2RGB)
                
                # display_text = "originPlate.point.x = " + str(int(self.originPlate.point.x))
                display_text = "originPlate = [%0.0f, %0.0f]" % (self.originPlate.point.x, self.originPlate.point.y)
                cv.PutText(self.im_display, display_text,(25,25),self.font,self.font_color)
                # display_text = "originPlate.point.y = " + str(int(self.originPlate.point.y))
                # cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)
                
                try:
                    self.undistorted_camera = self.tfrx.transformPoint(FRAME_IMAGERECT, self.originCamera)
                    cv.Circle(self.im_display, (int(self.undistorted_camera.point.x),int(self.undistorted_camera.point.y)), 3, cv.CV_RGB(self.color_max,0,self.color_max), cv.CV_FILLED)
                    self.undistorted_plate = self.tfrx.transformPoint(FRAME_IMAGERECT,self.originPlate)
                    cv.Circle(self.im_display, (int(self.undistorted_plate.point.x),int(self.undistorted_plate.point.y)), 3, cv.CV_RGB(0,self.color_max,0), cv.CV_FILLED)
                    cv.Circle(self.im_display, (int(self.undistorted_plate.point.x),int(self.undistorted_plate.point.y)), int(self.radiusMask), cv.CV_RGB(0,self.color_max,0))
                    display_text = "radiusMask = " + str(int(self.radiusMask))
                    cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)
                    
                    self.find_extrinsics()
                except (tf.LookupException, tf.ConnectivityException):
                    pass
                
                cv.ShowImage("Camera Plate Calibration", self.im_display)
                cv.WaitKey(3)

    
    def joy_callback(self, data):
        if self.initialized and self.initialized_images:
            self.originPlate.point.x += data.x_velocity
            self.originPlate.point.y += -data.y_velocity
            self.radiusMask += data.radius_velocity
    
    def point_callback(self, point):
        if self.initialized and self.initialized_images:
            self.originPlate.point.x += point.x #x_velocity
            self.originPlate.point.y += -point.y #y_velocity
            self.radiusMask += point.z #radius_velocity
    
    

def main(args):
    cal = CalibrateCameraPlate()
    try:
        rospy.spin()
    except:
        print "Shutting down"
    cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
