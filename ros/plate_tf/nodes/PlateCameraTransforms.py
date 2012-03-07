#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
import cv
import numpy as N
import tf
from sensor_msgs.msg import Image, CameraInfo
from pythonmodules import cvNumpy, CameraParameters
import plate_tf.srv


class Transforms:
    def __init__(self):
        self.camerainfo = None
        self.tfbx = tf.TransformBroadcaster()
        self.subCameraInfo = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)

        


    def CameraInfo_callback (self, msgCameraInfo):
        if self.camerainfo is None:
            #(intrinsic_matrix, distortion_coeffs) = CameraParameters.intrinsic("rect")
            self.camerainfo = msgCameraInfo
            self.M = N.ma.reshape(N.array(self.camerainfo.K),[3,3]) #cvNumpy.mat_to_array(N.array(self.camerainfo.K))
            self.M[:-1,-1] = 0
    
    
            (rvec, tvec) = CameraParameters.extrinsic("plate")
            rvec = cvNumpy.mat_to_array(rvec).squeeze()
            tvec = cvNumpy.mat_to_array(tvec).squeeze()
    
            rvec_angle = N.linalg.norm(rvec)
            R = tf.transformations.rotation_matrix(rvec_angle, rvec)
            T = tf.transformations.translation_matrix(tvec)
    
            self.Wsub = N.zeros((3,3))
            self.Wsub[:,:-1] = R[:-1,:-2]
            self.Wsub[:,-1] = T[:-1,-1]
    
            self.Hinv = N.dot(self.M, self.Wsub)
            self.Hinv = self.Hinv / self.Hinv[-1,-1]
            self.H = N.linalg.inv(self.Hinv)


    def SendTransforms(self):      
        if self.camerainfo is not None:
            self.tfbx.sendTransform((-self.camerainfo.K[2],-self.camerainfo.K[5],0), #(-608, -581, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "ImageRaw", "Camera")
            self.tfbx.sendTransform((-self.camerainfo.P[2],-self.camerainfo.P[6],0), #(-607, -551, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "ImageRect", "Camera")
            self.tfbx.sendTransform((48,-55,0),#(19, -10, 0.0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "PlateImage", "Camera")
            self.tfbx.sendTransform((0, 0, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "Plate", "PlateImage")
            self.tfbx.sendTransform((0, 0, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "ROIPlateImage", "ImageRect")
      
        
    def plate_to_camera(self, req):
        point_count = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        plate_points = N.array([Xsrc, Ysrc, Zsrc])
        camera_points = N.dot(self.Hinv, plate_points)
        Xdst = camera_points[0,:]
        Ydst = camera_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

    def camera_to_plate(self, req):
        point_count = min(len(req.Xsrc), len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        camera_points = N.array([Xsrc, Ysrc, Zsrc])
        plate_points = N.dot(self.H, camera_points)

        Xdst = plate_points[0,:]
        Ydst = plate_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}
        
    def Main(self):
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.SendTransforms()
                rate.sleep()
        except KeyboardInterrupt:
            print "Shutting down"



if __name__ == "__main__":
    rospy.init_node('camera_transforms')
    transforms = Transforms()
    rospy.Service('plate_to_camera', plate_tf.srv.PlateCameraConversion, transforms.plate_to_camera)
    rospy.Service('camera_to_plate', plate_tf.srv.PlateCameraConversion, transforms.camera_to_plate)
    transforms.Main()

