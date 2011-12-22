#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_calibration')
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
    cv.NamedWindow("Camera Plate Calibration", 1)
    self.tfrx = tf.TransformListener()
    self.tfbx = tf.TransformBroadcaster()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("UndistortedImage", Image, self.image_callback)
    self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joy_callback)
    self.color_max = 255
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    self.font_color = cv.CV_RGB(self.color_max,0,0)
    self.camera_plate = PointStamped()
    self.camera_plate.header.frame_id = "Camera"
    self.camera_origin = PointStamped()
    self.camera_origin.header.frame_id = "Camera"
    self.camera_origin.point.x = 0
    self.camera_origin.point.y = 0
    self.initialized_images = False
    # self.image_plate_origin_found = False
    # self.plate_origin = PointStamped()
    # self.plate_origin.header.frame_id = "Plate"
    # self.plate_origin.point.x = 0
    # self.plate_origin.point.y = 0

    (self.intrinsic_matrix,self.distortion_coeffs) = CameraParameters.intrinsic("undistorted")
    self.KK_cx = self.intrinsic_matrix[0,2]
    self.KK_cy = self.intrinsic_matrix[1,2]

    self.M = cvNumpy.mat_to_array(self.intrinsic_matrix)
    # Makes with respect to Camera coordinate system instead of Undistorted
    self.M[:-1,-1] = 0

    # Pattern info
    self.pattern_size = (8,6)
    self.col_corner_number = self.pattern_size[0]
    self.row_corner_number = self.pattern_size[1]
    self.board_corner_number = self.col_corner_number * self.row_corner_number
    self.checker_size = 15
    self.win = (4,4)
    self.zero_zone = (2,2)
    self.criteria = (cv.CV_TERMCRIT_ITER+cv.CV_TERMCRIT_EPS,100,.01)

    self.image_points = cv.CreateMat(self.board_corner_number,2,cv.CV_32FC1)
    self.plate_points = cv.CreateMat(self.board_corner_number,3,cv.CV_32FC1)
    self.point_counts = cv.CreateMat(1,1,cv.CV_32SC1)
    self.rvec = cv.CreateMat(1,3,cv.CV_32FC1)
    self.tvec = cv.CreateMat(1,3,cv.CV_32FC1)

    self.origin_points = cv.CreateMat(4,3,cv.CV_32FC1)
    self.origin_points_projected = cv.CreateMat(4,2,cv.CV_32FC1)

  def initialize_images(self,cv_image):
    self.im_size = cv.GetSize(cv_image)
    (self.im_width,self.im_height) = cv.GetSize(cv_image)
    self.im = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
    self.im_mask = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
    self.im_display = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)
    self.camera_plate.point.x = self.im_width//2 - self.KK_cx
    self.camera_plate.point.y = self.im_height//2 - self.KK_cy
    self.mask_radius = int(self.im_height * .48)
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
    cv.SetZero(self.origin_points)
    cv.SetReal2D(self.origin_points,1,0,self.checker_size) # x-direction
    cv.SetReal2D(self.origin_points,2,1,self.checker_size) # y-direction
    cv.SetReal2D(self.origin_points,3,2,self.checker_size) # z-direction
    axis_line_width = 3
    rect_line_width = 2
    cv.ProjectPoints2(self.origin_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
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
    # Update image mask
    cv.Circle(self.im_mask,(int(self.undistorted_plate.point.x),int(self.undistorted_plate.point.y)), int(self.mask_radius), self.color_max, cv.CV_FILLED)
    cv.And(self.im,self.im_mask,self.im)
    (corners_status, corners) = cv.FindChessboardCorners(self.im, self.pattern_size)
    if corners_status and (len(corners) == self.board_corner_number):
      sub_corners = cv.FindCornerSubPix(self.im, corners, self.win, self.zero_zone, self.criteria)
      cv.DrawChessboardCorners(self.im_display, self.pattern_size, sub_corners, corners_status)
      self.add_board_to_data(corners)
      cv.FindExtrinsicCameraParams2(self.plate_points,
                                    self.image_points,
                                    self.intrinsic_matrix,
                                    self.distortion_coeffs,
                                    self.rvec,
                                    self.tvec)

      self.rvec_array = cvNumpy.mat_to_array(self.rvec).squeeze()
      self.tvec_array = cvNumpy.mat_to_array(self.tvec).squeeze()

      rvec_angle = numpy.linalg.norm(self.rvec_array)
      R = tf.transformations.rotation_matrix(rvec_angle,self.rvec_array)
      T = tf.transformations.translation_matrix(self.tvec_array)

      Wsub = numpy.zeros((3,3))
      Wsub[:,:-1] = R[:-1,:-2]
      Wsub[:,-1] = T[:-1,-1]

      Hinv = numpy.dot(self.M,Wsub)
      Hinv = Hinv/Hinv[-1,-1]
      H = numpy.linalg.inv(Hinv)
      self.Hinv = Hinv
      self.H = H
      self.tfbx.sendTransform(self.tvec_array,
                                        tf.transformations.quaternion_about_axis(rvec_angle, self.rvec_array),
                                        rospy.Time.now(),
                                        "Checkerboard",
                                        "Image")

      # rospy.logwarn("H = \n%s", str(H))

      # rvec_array = cvNumpy.mat_to_array(self.rvec)
      # display_text = "rvec = " + str(rvec_array[0])
      # cv.PutText(self.im_display,display_text,(25,420),self.font,self.font_color)
      # tvec_array = cvNumpy.mat_to_array(self.tvec)
      # display_text = "tvec = " + str(tvec_array[0])
      # cv.PutText(self.im_display,display_text,(25,440),self.font,self.font_color)
      # self.draw_origin(self.im_display)
      # rvec_array = numpy.array([-3.06,-0.0014,0.0042])
      # tvec_array = numpy.array([3.9,46.7,335.9])
      # rvec_array = rvec_array.astype('float32')
      # tvec_array = tvec_array.astype('float32')
      # self.rvec = cvNumpy.array_to_mat(rvec_array)
      # self.tvec = cvNumpy.array_to_mat(tvec_array)
      self.draw_origin(self.im_display)
      self.remap_extrinsics()

  def image_callback(self,data):
    try:
      cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
    except CvBridgeError, e:
      print e

    if not self.initialized_images:
      self.initialize_images(cv_image)

    self.im = cv.CloneImage(cv_image)
    cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

    # display_text = "camera_plate.point.x = " + str(int(self.camera_plate.point.x))
    display_text = "camera_plate_origin = [%0.0f, %0.0f]" % (self.camera_plate.point.x, self.camera_plate.point.y)
    cv.PutText(self.im_display,display_text,(25,25),self.font,self.font_color)
    # display_text = "camera_plate.point.y = " + str(int(self.camera_plate.point.y))
    # cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)

    try:
      self.undistorted_camera = self.tfrx.transformPoint("UndistortedImage",self.camera_origin)
      cv.Circle(self.im_display, (int(self.undistorted_camera.point.x),int(self.undistorted_camera.point.y)), 3, cv.CV_RGB(self.color_max,0,self.color_max), cv.CV_FILLED)
      self.undistorted_plate = self.tfrx.transformPoint("UndistortedImage",self.camera_plate)
      cv.Circle(self.im_display, (int(self.undistorted_plate.point.x),int(self.undistorted_plate.point.y)), 3, cv.CV_RGB(0,self.color_max,0), cv.CV_FILLED)
      cv.Circle(self.im_display, (int(self.undistorted_plate.point.x),int(self.undistorted_plate.point.y)), int(self.mask_radius), cv.CV_RGB(0,self.color_max,0))
      display_text = "mask radius = " + str(int(self.mask_radius))
      cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)

      self.find_extrinsics()
    except (tf.LookupException, tf.ConnectivityException):
      pass

    cv.ShowImage("Camera Plate Calibration", self.im_display)
    cv.WaitKey(3)

  def joy_callback(self,data):
    if self.initialized_images:
      self.camera_plate.point.x += data.x_velocity
      self.camera_plate.point.y += -data.y_velocity
      self.mask_radius += data.radius_velocity

  def remap_extrinsics(self):
    # self.find_H_image_to_plate()
    # display_text = "undistorted_plate.point = " + str(int(self.undistorted_plate.point.x)) + "," + str(int(self.undistorted_plate.point.y))
    # cv.PutText(self.im_display,display_text,(25,300),self.font,self.font_color)
    # Ximage = [self.undistorted_plate.point.x.tolist()]
    # Yimage = [self.undistorted_plate.point.y.tolist()]
    # Xplate,Yplate = self.image_to_plate(Ximage,Yimage)
    # display_text = "new origin point = " + str(int(Xplate)) + "," + str(int(Yplate))
    # cv.PutText(self.im_display,display_text,(25,320),self.font,self.font_color)
    # cv.SetZero(self.distortion_coeffs)
    # rotation_matrix = cv.CreateMat(3,3,cv.CV_32FC1)
    # cv.Rodrigues2(self.rvec, rotation_matrix)
    # R = cvNumpy.mat_to_array(rotation_matrix)
    # T = cvNumpy.mat_to_array(self.tvec).transpose()
    # W = numpy.concatenate((R,numpy.zeros((1,3))),0)
    # T = numpy.append(T,1)
    # T = numpy.array([T]).transpose()
    # rospy.logwarn("R = %s", str(R))
    # rospy.logwarn("T = %s", str(T))
    # W = numpy.concatenate((W,T),1)
    # rospy.logwarn("W = %s", str(W))

    # T2 = numpy.array([36.1,55.3,0,1])
    # T2 = numpy.array([10,0,0,1])
    # T2 = numpy.array([T2]).transpose()
    # ang = 1.086
    # ang = -1.086
    # R2 = numpy.array([[math.cos(ang), math.sin(ang), 0],
    #                   [-math.sin(ang), math.cos(ang), 0],
    #                   [0,0,1]])
    # rospy.logwarn("R2 = %s", str(R2))
    # rospy.logwarn("T2 = %s", str(T2))
    # W2 = numpy.concatenate((R2,numpy.zeros((1,3))),0)
    # W2 = numpy.concatenate((W2,T2),1)
    # rospy.logwarn("W2 = %s", str(W2))

    # W3 = numpy.dot(W,W2)
    # W3 = numpy.dot(W2,W)
    # rospy.logwarn("W3 = %s", str(W3))
    # tvec_new = W3[:-1,-1]
    # R_new = W3[:-1,:-1]
    # R_new = R_new.astype('float32')
    # rvec_new = cv.CreateMat(1,3,cv.CV_32FC1)
    # R_new = cvNumpy.array_to_mat(R_new)
    # cv.Rodrigues2(R_new,rvec_new)

    X = [self.camera_plate.point.x, self.camera_plate.point.x + self.checker_size]
    Y = [self.camera_plate.point.y, self.camera_plate.point.y]
    Z = [1,1]
    camera_points = numpy.array([X,Y,Z])
    plate_points = numpy.dot(self.H,camera_points)
    x0 = plate_points[0,0]
    x1 = plate_points[0,1]
    y0 = plate_points[1,0]
    y1 = plate_points[1,1]
    rot_angle = numpy.arctan2((y1-y0),(x1-x0))

    # rospy.logwarn("plate_points = %s", str(plate_points))
    checkerboard_plate_vector = plate_points[:,0]
    checkerboard_plate_vector[2] = 0
    self.tfbx.sendTransform(checkerboard_plate_vector,
                                      tf.transformations.quaternion_from_euler(0, 0, rot_angle),
                                      rospy.Time.now(),
                                      "Plate",
                                      "Checkerboard")
    try:
      # self.image_plate_origin = self.tfrx.transformPoint("Image",self.plate_origin)
      (trans,rot_quat) = self.tfrx.lookupTransform('Image', 'Plate', rospy.Time(0))
      # rospy.logwarn("trans = %s", str(trans))
      # rospy.logwarn("rot = %s", str(rot))

      rot_array = tf.transformations.quaternion_matrix(rot_quat)
      rot_array = rot_array[0:3,0:3]
      rot_array = rot_array.astype('float32')
      rot_mat = cvNumpy.array_to_mat(rot_array)
      cv.Rodrigues2(rot_mat,self.rvec)
      rvec_array_new = cvNumpy.mat_to_array(self.rvec).squeeze()
      # tvec_array_new = numpy.array([self.image_plate_origin.point.x,
      #                               self.image_plate_origin.point.y,
      #                               self.image_plate_origin.point.z])
      tvec_array_new = numpy.array(trans)
      self.rvec_array = rvec_array_new
      self.tvec_array = tvec_array_new
      self.rvec = cvNumpy.array_to_mat(self.rvec_array)
      self.tvec = cvNumpy.array_to_mat(self.tvec_array)
      display_text = "rvec = [%0.3f, %0.3f, %0.3f]" % (self.rvec_array[0],self.rvec_array[1],self.rvec_array[2])
      cv.PutText(self.im_display,display_text,(25,420),self.font,self.font_color)
      display_text = "tvec = [%0.3f, %0.3f, %0.3f]" % (self.tvec_array[0],self.tvec_array[1],self.tvec_array[2])
      cv.PutText(self.im_display,display_text,(25,440),self.font,self.font_color)
      # self.image_plate_origin_found = True
      self.draw_origin(self.im_display)

    except (tf.LookupException, tf.ConnectivityException):
      pass

    # rospy.logwarn("tvec_array = %s", str(self.tvec_array))
    # rotation_matrix = cv.CreateMat(3,3,cv.CV_32FC1)
    # cv.Rodrigues2(self.rvec, rotation_matrix)
    # R = cvNumpy.mat_to_array(rotation_matrix)
    # Rinv = numpy.linalg.inv(R)
    # plate_points = numpy.dot(Rinv,plate_points)
    # plate_points[-1,0] = 0
    # tvec_array_new = self.tvec_array + plate_points[:,0]
    # rospy.logwarn("tvec_array_new = %s", str(tvec_array_new))
    # rospy.logwarn("tvec_array_new_2 = %s", str(tvec_array_new_2))
    # tvec_array_new[0] += 5
    # rospy.logwarn("rvec_array = %s", str(self.rvec_array))
    # rospy.logwarn("tvec_array = %s", str(self.tvec_array))
    # rospy.logwarn("rvec_new = %s", str(rvec_new))
    # rospy.logwarn("tvec_new = %s", str(tvec_new))

    # rospy.logwarn("rvec_array = %s", str(rvec_array))
    # rospy.logwarn("tvec_array = %s", str(tvec_array))
    # self.rvec = cvNumpy.array_to_mat(rvec_new)
    # self.tvec = cvNumpy.array_to_mat(tvec_new)
    # Winv = la.inv(W)
    # rospy.logwarn("Winv = %s", str(Winv))
    # H = numpy.dot(M,Winv)
    # rospy.logwarn("H = %s", str(H))

  # def find_H_image_to_plate(self):
  #   Xplate = numpy.arange(0,self.pattern_size[0],1)
  #   Yplate = numpy.arange(0,self.pattern_size[1],1)
  #   XXplate,YYplate = numpy.meshgrid(Xplate,Yplate)
  #   XXplate = (XXplate - (self.pattern_size[0]-1)/2)*self.checker_size
  #   YYplate = (YYplate - (self.pattern_size[1]-1)/2)*self.checker_size
  #   XXplate_list = (XXplate.reshape(1,self.pattern_size[0]*self.pattern_size[1])).squeeze().tolist()
  #   YYplate_list = (YYplate.reshape(1,self.pattern_size[0]*self.pattern_size[1])).squeeze().tolist()
  #   # class PlatePoints:
  #   #     def __init__(self):
  #   #         Xsrc = ()
  #   #         Ysrc = ()
  #   # plate_points = PlatePoints()
  #   # plate_points.Xsrc = XXplate_list
  #   # plate_points.Ysrc = YYplate_list
  #   (XXcamera_list,YYcamera_list) = self.plate_to_camera(XXplate_list,YYplate_list)
  #   # XXcamera_list = list(image_points['Xdst'])
  #   # YYcamera_list = list(image_points['Ydst'])
  #   plate_points = numpy.array([XXplate_list,YYplate_list])
  #   plate_points = plate_points.transpose()
  #   plate_points = plate_points.astype('float32')
  #   # rospy.logwarn("plate_points = %s", str(plate_points))
  #   plate_points = cvNumpy.array_to_mat(plate_points)
  #   image_points = numpy.array([XXcamera_list,YYcamera_list])
  #   image_points = image_points.transpose()
  #   image_points = image_points.astype('float32')
  #   # rospy.logwarn("image_points = %s", str(image_points))
  #   image_points = cvNumpy.array_to_mat(image_points)
  #   Homography_camera_to_plate = cv.CreateMat(3,3,cv.CV_32FC1)
  #   # rospy.logwarn("image_points = %s", str(image_points))
  #   # rospy.logwarn("plate_points = %s", str(plate_points))
  #   # rospy.logwarn("self.Homography_camera_to_plate = %s", str(Homography_camera_to_plate))
  #   # cv.FindHomography(image_points,plate_points,Homography_camera_to_plate,0)
  #   self.Homography_camera_to_plate_array = cvNumpy.mat_to_array(Homography_camera_to_plate)

  #   # image_points = self.image_points
  #   # plate_points = cvNumpy.mat_to_array(self.plate_points)
  #   # plate_points = plate_points[:,:2]
  #   # # image_points = image_points.astype('float32')
  #   # plate_points = plate_points.astype('float32')
  #   # plate_points = cvNumpy.array_to_mat(plate_points)
  #   # rospy.loginfo("image_points = %s", str(image_points))
  #   # rospy.loginfo("plate_points = %s", str(plate_points))
  #   # self.Homography_image_to_plate = cv.CreateMat(3,3,cv.CV_32FC1)
  #   # rospy.loginfo("homography = %s", str(self.Homography_image_to_plate))
  #   # cv.FindHomography(image_points,plate_points,self.Homography_image_to_plate,0)
  #   # rospy.logwarn("Homo_mat = %s", str(self.Homography_image_to_plate))
  #   # rospy.logwarn("Homo_mat_type = %s", str(cv.GetElemType(self.Homography_image_to_plate)))
  #   # self.Homography_image_to_plate_array = cvNumpy.mat_to_array(self.Homography_image_to_plate)
  #   # rospy.logwarn("Homo_array = %s", str(self.Homography_image_to_plate_array))

  # def image_to_plate(self,Xsrc,Ysrc):
  #   Xsrc = list(Xsrc)
  #   Ysrc = list(Ysrc)
  #   point_count = min(len(Xsrc),len(Ysrc))
  #   Zsrc = [1]*point_count
  #   image_points = numpy.array([Xsrc,Ysrc,Zsrc])
  #   # rospy.loginfo("image_points = %s", str(image_points))
  #   # plate_points = numpy.dot(self.Homography_image_to_plate_array,image_points)
  #   plate_points = image_points
  #   Xdst = plate_points[0,:]
  #   Ydst = plate_points[1,:]
  #   return Xdst,Ydst

  # def plate_to_camera(self,Xsrc,Ysrc):
  #   point_count = min(len(Xsrc),len(Ysrc))
  #   # ProjectPoints2 does not like point counts less than three
  #   # Could use multiple channels
  #   # Or just stuff dummy points into point calculation
  #   if point_count < 3:
  #     point_count_calc = point_count + 2
  #     Xsrc = list(Xsrc)
  #     Xsrc.extend([0,0])
  #     Ysrc = list(Ysrc)
  #     Ysrc.extend([0,0])
  #   else:
  #     point_count_calc = point_count
  #     Xsrc = Xsrc
  #     Ysrc = Ysrc

  #   image_points = cv.CreateMat(point_count_calc,2,cv.CV_32FC1)
  #   # rospy.loginfo("image_points = %s", str(image_points))
  #   Zsrc = [0]*point_count_calc
  #   object_points = numpy.array([Xsrc,Ysrc,Zsrc])
  #   object_points = object_points.transpose()
  #   # rospy.loginfo("object_points = %s", str(object_points))
  #   object_points = cvNumpy.array_to_mat(object_points)
  #   cv.ProjectPoints2(object_points,self.rvec,self.tvec,self.intrinsic_matrix,self.distortion_coeffs,image_points)
  #   image_points = cvNumpy.mat_to_array(image_points)
  #   Xdst = image_points[:point_count,0].tolist()
  #   Ydst = image_points[:point_count,1].tolist()
  #   return Xdst,Ydst


def main(args):
  rospy.init_node('CameraInfoSave', anonymous=True)
  cal = Calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
