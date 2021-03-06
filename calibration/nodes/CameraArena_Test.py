#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('calibration')
import sys
import rospy
import cv
import tf
import math
import numpy
from pythonmodules import cvNumpy,CameraParameters
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from joystick_commands.msg import JoystickCommands
from cv_bridge import CvBridge, CvBridgeError

class Calibration:

  def __init__(self):
    self.initialized = False
    self.initialized_images = False
    print "Initialized00"
    cv.NamedWindow("Camera Arena Calibration", 1)
    print "Initialized01"
    self.tfrx = tf.TransformListener()
    # print "Initialized02"
    # self.tfbx = tf.TransformBroadcaster()
    # print "Initialized03"
    # self.bridge = CvBridge()
    # print "Initialized04"
    # self.image_sub = rospy.Subscriber("UndistortedImage", Image, self.image_callback)
    # print "Initialized05"
    # self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joy_callback)
    # print "Initialized06"
    # self.color_max = 255
    # print "Initialized07"
    # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    # print "Initialized08"
    # self.font_color = cv.CV_RGB(self.color_max,0,0)
    # print "Initialized09"
    # self.camera_arena = PointStamped()
    # print "Initialized10"
    # self.camera_arena.header.frame_id = "Camera"
    # print "Initialized11"
    # self.camera_origin = PointStamped()
    # print "Initialized12"
    # self.camera_origin.header.frame_id = "Camera"
    # print "Initialized13"
    # self.camera_origin.point.x = 0
    # print "Initialized14"
    # self.camera_origin.point.y = 0
    # print "Initialized15"

    # (self.intrinsic_matrix,self.distortion_coeffs) = CameraParameters.intrinsic("undistorted")
    # self.KK_cx = self.intrinsic_matrix[0,2]
    # self.KK_cy = self.intrinsic_matrix[1,2]

    # self.M = cvNumpy.mat_to_array(self.intrinsic_matrix)
    # # Makes with respect to Camera coordinate system instead of Undistorted
    # self.M[:-1,-1] = 0

    # # Pattern info
    # self.pattern_size = (8,6)
    # self.col_corner_number = self.pattern_size[0]
    # self.row_corner_number = self.pattern_size[1]
    # self.board_corner_number = self.col_corner_number * self.row_corner_number
    # self.checker_size = 15
    # self.win = (4,4)
    # self.zero_zone = (2,2)
    # self.criteria = (cv.CV_TERMCRIT_ITER+cv.CV_TERMCRIT_EPS,100,.01)

    # self.image_points = cv.CreateMat(self.board_corner_number,2,cv.CV_32FC1)
    # self.arena_points = cv.CreateMat(self.board_corner_number,3,cv.CV_32FC1)
    # self.point_counts = cv.CreateMat(1,1,cv.CV_32SC1)
    # self.rvec = cv.CreateMat(1,3,cv.CV_32FC1)
    # self.tvec = cv.CreateMat(1,3,cv.CV_32FC1)

    # self.origin_points = cv.CreateMat(4,3,cv.CV_32FC1)
    # self.origin_points_projected = cv.CreateMat(4,2,cv.CV_32FC1)
    # self.initialized = True

  # def initialize_images(self,cv_image):
  #   self.im_size = cv.GetSize(cv_image)
  #   (self.im_width,self.im_height) = cv.GetSize(cv_image)
  #   self.im = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
  #   self.im_mask = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
  #   self.im_display = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)
  #   self.camera_arena.point.x = self.im_width//2 - self.KK_cx
  #   self.camera_arena.point.y = self.im_height//2 - self.KK_cy
  #   self.mask_radius = int(self.im_height * .48)
  #   self.initialized_images = True

  # def add_board_to_data(self,corners):
  #   self.capture_count = 0
  #   step = self.capture_count * self.board_corner_number
  #   for corner_count in range(self.board_corner_number):
  #     (x,y) = corners[corner_count]
  #     cv.SetReal2D(self.image_points, step, 0, x)
  #     cv.SetReal2D(self.image_points, step, 1, y)
  #     cv.SetReal2D(self.arena_points, step, 0, (corner_count // self.col_corner_number)*self.checker_size)
  #     cv.SetReal2D(self.arena_points, step, 1, (corner_count % self.col_corner_number)*self.checker_size)
  #     cv.SetReal2D(self.arena_points, step, 2, 0.0)
  #     step += 1
  #   cv.SetReal2D(self.point_counts, self.capture_count, 0, self.board_corner_number)

  # def draw_origin(self,im_color):
  #   cv.SetZero(self.origin_points)
  #   cv.SetReal2D(self.origin_points,1,0,self.checker_size) # x-direction
  #   cv.SetReal2D(self.origin_points,2,1,self.checker_size) # y-direction
  #   cv.SetReal2D(self.origin_points,3,2,self.checker_size) # z-direction
  #   axis_line_width = 3
  #   rect_line_width = 2
  #   cv.ProjectPoints2(self.origin_points,
  #                     self.rvec,
  #                     self.tvec,
  #                     self.intrinsic_matrix,
  #                     self.distortion_coeffs,
  #                     self.origin_points_projected)

  #   try:
  #     a = cvNumpy.mat_to_array(self.origin_points_projected)

  #     # origin point
  #     pt1 = tuple(a[0])

  #     # draw x-axis
  #     pt2 = tuple(a[1])
  #     cv.Line(im_color,pt1,pt2,cv.CV_RGB(self.color_max,0,0),axis_line_width)

  #     # draw y-axis
  #     pt2 = tuple(a[2])
  #     cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,self.color_max,0),axis_line_width)

  #     # draw z-axis
  #     pt2 = tuple(a[3])
  #     cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,0,self.color_max),axis_line_width)

  #   except:
  #     pass


  # def find_extrinsics(self):
  #   # Update image mask
  #   cv.Circle(self.im_mask,(int(self.undistorted_arena.point.x),int(self.undistorted_arena.point.y)), int(self.mask_radius), self.color_max, cv.CV_FILLED)
  #   cv.And(self.im,self.im_mask,self.im)
  #   (corners_status, corners) = cv.FindChessboardCorners(self.im, self.pattern_size)
  #   if corners_status and (len(corners) == self.board_corner_number):
  #     sub_corners = cv.FindCornerSubPix(self.im, corners, self.win, self.zero_zone, self.criteria)
  #     cv.DrawChessboardCorners(self.im_display, self.pattern_size, sub_corners, corners_status)
  #     self.add_board_to_data(corners)
  #     cv.FindExtrinsicCameraParams2(self.arena_points,
  #                                   self.image_points,
  #                                   self.intrinsic_matrix,
  #                                   self.distortion_coeffs,
  #                                   self.rvec,
  #                                   self.tvec)

  #     self.rvec_array = cvNumpy.mat_to_array(self.rvec).squeeze()
  #     self.tvec_array = cvNumpy.mat_to_array(self.tvec).squeeze()

  #     rvec_angle = numpy.linalg.norm(self.rvec_array)
  #     R = tf.transformations.rotation_matrix(rvec_angle,self.rvec_array)
  #     T = tf.transformations.translation_matrix(self.tvec_array)

  #     Wsub = numpy.zeros((3,3))
  #     Wsub[:,:-1] = R[:-1,:-2]
  #     Wsub[:,-1] = T[:-1,-1]

  #     Hinv = numpy.dot(self.M,Wsub)
  #     Hinv = Hinv/Hinv[-1,-1]
  #     H = numpy.linalg.inv(Hinv)
  #     self.Hinv = Hinv
  #     self.H = H

  #     self.tfbx.sendTransform(self.tvec_array,
  #                                       tf.transformations.quaternion_about_axis(rvec_angle, self.rvec_array),
  #                                       rospy.Time.now(),
  #                                       "Checkerboard",
  #                                       "Image")

  #     self.draw_origin(self.im_display)
  #     self.remap_extrinsics()

  # def image_callback(self,data):
  #   if self.initialized:
  #     try:
  #       cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
  #     except CvBridgeError, e:
  #       print e

  #     if not self.initialized_images:
  #       self.initialize_images(cv_image)

  #     self.im = cv.CloneImage(cv_image)
  #     cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

  #     display_text = "camera_arena_origin = [%0.0f, %0.0f]" % (self.camera_arena.point.x, self.camera_arena.point.y)
  #     cv.PutText(self.im_display,display_text,(25,25),self.font,self.font_color)

  #     try:
  #       self.undistorted_camera = self.tfrx.transformPoint("UndistortedImage",self.camera_origin)
  #       cv.Circle(self.im_display, (int(self.undistorted_camera.point.x),int(self.undistorted_camera.point.y)), 3, cv.CV_RGB(self.color_max,0,self.color_max), cv.CV_FILLED)
  #       self.undistorted_arena = self.tfrx.transformPoint("UndistortedImage",self.camera_arena)
  #       cv.Circle(self.im_display, (int(self.undistorted_arena.point.x),int(self.undistorted_arena.point.y)), 3, cv.CV_RGB(0,self.color_max,0), cv.CV_FILLED)
  #       cv.Circle(self.im_display, (int(self.undistorted_arena.point.x),int(self.undistorted_arena.point.y)), int(self.mask_radius), cv.CV_RGB(0,self.color_max,0))
  #       display_text = "mask radius = " + str(int(self.mask_radius))
  #       cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)

  #       self.find_extrinsics()
  #     except (tf.LookupException, tf.ConnectivityException):
  #       pass

  #     cv.ShowImage("Camera Arena Calibration", self.im_display)
  #     cv.WaitKey(3)

  # def joy_callback(self,data):
  #   if self.initialized and self.initialized_images:
  #     self.camera_arena.point.x += data.x_velocity
  #     self.camera_arena.point.y += -data.y_velocity
  #     self.mask_radius += data.radius_velocity

  # def remap_extrinsics(self):
  #   X = [self.camera_arena.point.x, self.camera_arena.point.x + self.checker_size]
  #   Y = [self.camera_arena.point.y, self.camera_arena.point.y]
  #   Z = [1,1]
  #   camera_points = numpy.array([X,Y,Z])
  #   arena_points = numpy.dot(self.H,camera_points)
  #   x0 = arena_points[0,0]
  #   x1 = arena_points[0,1]
  #   y0 = arena_points[1,0]
  #   y1 = arena_points[1,1]
  #   rot_angle = numpy.arctan2((y1-y0),(x1-x0))

  #   checkerboard_arena_vector = arena_points[:,0]
  #   checkerboard_arena_vector[2] = 0

  #   self.tfbx.sendTransform(checkerboard_arena_vector,
  #                                     tf.transformations.quaternion_from_euler(0, 0, rot_angle),
  #                                     rospy.Time.now(),
  #                                     "Arena",
  #                                     "Checkerboard")

  #   try:
  #     (trans,rot_quat) = self.tfrx.lookupTransform('Image', 'Arena', rospy.Time(0))

  #     rot_array = tf.transformations.quaternion_matrix(rot_quat)
  #     rot_array = rot_array[0:3,0:3]
  #     rot_array = rot_array.astype('float32')
  #     rot_mat = cvNumpy.array_to_mat(rot_array)
  #     cv.Rodrigues2(rot_mat,self.rvec)
  #     rvec_array_new = cvNumpy.mat_to_array(self.rvec).squeeze()
  #     tvec_array_new = numpy.array(trans)
  #     self.rvec_array = rvec_array_new
  #     self.tvec_array = tvec_array_new
  #     self.rvec = cvNumpy.array_to_mat(self.rvec_array)
  #     self.tvec = cvNumpy.array_to_mat(self.tvec_array)
  #     display_text = "rvec = [%0.3f, %0.3f, %0.3f]" % (self.rvec_array[0],self.rvec_array[1],self.rvec_array[2])
  #     cv.PutText(self.im_display,display_text,(25,420),self.font,self.font_color)
  #     display_text = "tvec = [%0.3f, %0.3f, %0.3f]" % (self.tvec_array[0],self.tvec_array[1],self.tvec_array[2])
  #     cv.PutText(self.im_display,display_text,(25,440),self.font,self.font_color)
  #     self.draw_origin(self.im_display)

  #   except (tf.LookupException, tf.ConnectivityException):
  #     pass


def main(args):
  rospy.init_node('camera_arena_calibration_test')
  cal = Calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
