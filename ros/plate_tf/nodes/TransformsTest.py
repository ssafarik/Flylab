#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
# import cv
import numpy
import tf
from pythonmodules import CameraParameters
from pythonmodules import cvNumpy
from plate_tf.srv import *

class TransformsTest:
    def __init__(self):

        (intrinsic_matrix,distortion_coeffs) = CameraParameters.intrinsic("rect")
        (rvec,tvec) = CameraParameters.extrinsic("plate")
        intrinsic_matrix = cvNumpy.mat_to_array(intrinsic_matrix)
        rvec = cvNumpy.mat_to_array(rvec).squeeze()
        tvec = cvNumpy.mat_to_array(tvec).squeeze()

        # Listener/Subscribers
        self.tf_listener = tf.TransformListener()
        # self.Hinv = numpy.array([\
        #     [2.02, 7.57e-3, 3.18e2],
        #     [2.24e-3, -2, 2.40e2],
        #     [1.6e-5,2.56e-5,1]])

        # alpha, beta, gamma = 0.123, -1.234, 2.345
        # origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        # I = tf.transformations.identity_matrix()
        # Rx = tf.transformations.rotation_matrix(alpha, xaxis)
        # rospy.logwarn("Rx = %s", str(Rx))
        rvec_angle = numpy.linalg.norm(rvec)
        R = tf.transformations.rotation_matrix(rvec_angle,rvec)
        T = tf.transformations.translation_matrix(tvec)
        # rospy.logwarn("R = \n%s", str(R))
        # rospy.logwarn("T = \n%s", str(T))
        Wsub = numpy.zeros((3,3))
        Wsub[:,:-1] = R[:-1,:-2]
        Wsub[:,-1] = T[:-1,-1]
        # rospy.logwarn("Wsub = \n%s", str(Wsub))
        # M = intrinsic_matrix
        # rospy.logwarn("M = \n%s", str(M))
        Hinv = numpy.dot(M,Wsub)
        Hinv = Hinv/Hinv[-1,-1]
        # rospy.logwarn("Hinv = \n%s", str(Hinv))
        R = numpy.zeros((4,4))
        R[-1,-1] = 1
        R[-2,-2] = 1
        R[:2,:2] = Hinv[:2,:2]
        # rospy.logwarn("R = \n%s", str(R))
        T = Hinv[:-1,-1]
        # rospy.logwarn("T = \n%s", str(T))
        # q = tf.transformations.quaternion_from_matrix(R)
        # rospy.logwarn("q = \n%s", str(q))
        H = numpy.linalg.inv(Hinv)
        self.Hinv = Hinv
        self.H = H

    def plate_to_camera_tf(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        plate_points = numpy.array([Xsrc,Ysrc,Zsrc])
        # self.tf_listener.transformPoint
        image_points = numpy.dot(self.Hinv,plate_points)
        Xdst = image_points[0,:]
        Ydst = image_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

    def camera_to_plate_tf(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        image_points = numpy.array([Xsrc,Ysrc,Zsrc])
        # rospy.loginfo("image_points = %s", str(image_points))
        plate_points = numpy.dot(self.H,image_points)

        Xdst = plate_points[0,:]
        Ydst = plate_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}


def plate_camera_transforms_server_test():
    rospy.init_node('plate_camera_transforms_server_test')
    transforms_test = TransformsTest()
    s = rospy.Service('plate_to_camera_tf', PlateCameraConversion, transforms_test.plate_to_camera_tf)
    s = rospy.Service('camera_to_plate_tf', PlateCameraConversion, transforms_test.camera_to_plate_tf)


if __name__ == "__main__":
    plate_camera_transforms_server_test()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

