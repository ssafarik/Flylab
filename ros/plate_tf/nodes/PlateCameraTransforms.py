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
        
        self.originPlateX = rospy.get_param('camera/originPlateX', 0.0) 
        self.originPlateY = rospy.get_param('camera/originPlateY', 0.0) 
        self.originPlateZ = rospy.get_param('camera/originPlateZ', 0.0) 
        


    def CameraInfo_callback (self, msgCameraInfo):
        if self.camerainfo is None:
            self.camerainfo = msgCameraInfo
            M = N.reshape(N.array(self.camerainfo.K),[3,3]) #cvNumpy.mat_to_array(N.array(self.camerainfo.K))
            #M = N.reshape(N.array(self.camerainfo.P),[3,4])[0:3,0:3]
            #M = N.reshape(N.array([1690.6893719741, 0, 632.966049657568, 0, 1724.60733564515, 540.153219137248, 0, 0, 1]),[3,3])
            #M = N.reshape(N.array([1699.89142716178, 0, 637.374529813928, 0, 0, 1734.32501822431, 533.670067729561, 0, 0, 0, 1, 0]),[3,4])[0:3,0:3]
            M[:-1,-1] = 0  # Zero the translation entries (1,3) and (2,3).
    
            (rvec, tvec) = CameraParameters.extrinsic("plate")
            rvec = cvNumpy.mat_to_array(rvec).squeeze()
            tvec = cvNumpy.mat_to_array(tvec).squeeze()
    
            angleRvec = N.linalg.norm(rvec)
            R = tf.transformations.rotation_matrix(angleRvec, rvec)
            T = tf.transformations.translation_matrix(tvec)
    
            self.Wsub = N.zeros((3,3))
            self.Wsub[:,:-1] = R[:-1,:-2]
            self.Wsub[:,-1] = T[:-1,-1]
    
            self.Hinv = N.dot(M, self.Wsub)
            #rospy.logwarn ('Hinv scalar %f' % self.Hinv[-1,-1])
            self.Hinv = self.Hinv / self.Hinv[-1,-1]
            self.H = N.linalg.inv(self.Hinv)
            

    def SendTransforms(self):      
        if self.camerainfo is not None:
            self.tfbx.sendTransform((0,0,0),#(-self.camerainfo.K[2], -self.camerainfo.K[5],0), #(-608, -581, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "ImageRaw", "Camera")
            self.tfbx.sendTransform((0,0,0),#(self.camerainfo.K[2], self.camerainfo.K[5],0), #(-607, -551, 0), 
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "ImageRect", "ImageRaw")
            self.tfbx.sendTransform((self.originPlateX,
                                     -self.originPlateY,
                                     self.originPlateZ),#(19, -10, 0.0), # Comes from running CameraPlate.launch
                                    (0,0,0,1), 
                                    rospy.Time.now(), 
                                    "Plate", "ImageRect")
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

