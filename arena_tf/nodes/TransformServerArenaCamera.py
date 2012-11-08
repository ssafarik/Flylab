#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import cv
import numpy as N
import tf
from sensor_msgs.msg import Image, CameraInfo
from pythonmodules import cvNumpy, CameraParameters
import arena_tf.srv


class TransformServerArenaCamera:
    def __init__(self):
        self.initialized = False
        self.camerainfo = None
        rospy.init_node('TransformServerArenaCamera')
        
        self.tfbx = tf.TransformBroadcaster()

        
        self.xMask = rospy.get_param('camera/mask/x', 0.0) 
        self.yMask = rospy.get_param('camera/mask/y', 0.0) 
        self.zMask = rospy.get_param('camera/mask/z', 0.0) 

        self.Hinv = N.identity(3)
        self.subCameraInfo = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)

        rospy.Service('camera_from_arena', arena_tf.srv.ArenaCameraConversion, self.CameraFromArena_callback)
        rospy.Service('arena_from_camera', arena_tf.srv.ArenaCameraConversion, self.ArenaFromCamera_callback)
        
        self.initialized = True


    def CameraInfo_callback (self, camerainfo):
        if (self.initialized) and (self.camerainfo is None):
            M = N.reshape(N.array(camerainfo.K),[3,3]) #cvNumpy.mat_to_array(N.array(camerainfo.K))
            #M = N.reshape(N.array(camerainfo.P),[3,4])[0:3,0:3]
            M[:-1,-1] = 0  # Zero the translation entries (1,3) and (2,3).
    
            (rvec, tvec) = CameraParameters.extrinsic("arena")
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

        self.camerainfo = camerainfo
            

    def CameraFromArena_callback(self, req):
        if (self.camerainfo is not None):
            point_count = min(len(req.Xsrc), len(req.Ysrc))
            xSrc = list(req.Xsrc)
            ySrc = list(req.Ysrc)
            zSrc = [1]*point_count
            arena_points = N.array([xSrc, ySrc, zSrc])
            camera_points = N.dot(self.Hinv, arena_points)

            xDst = camera_points[0,:]
            yDst = camera_points[1,:]
        else:
            xDst = None
            yDst = None
            
        return {'Xdst': xDst,
                'Ydst': yDst}

    def ArenaFromCamera_callback(self, req):
        if (self.camerainfo is not None):
            point_count = min(len(req.Xsrc), len(req.Ysrc))
            xSrc = list(req.Xsrc)
            ySrc = list(req.Ysrc)
            zSrc = [1]*point_count
            camera_points = N.array([xSrc, ySrc, zSrc])
            arena_points = N.dot(self.H, camera_points)
    
            xDst = arena_points[0,:]
            yDst = arena_points[1,:]
        else:
            xDst = None
            yDst = None
            
        return {'Xdst': xDst,
                'Ydst': yDst}
        

    def SendTransforms(self):      
        if (self.camerainfo is not None):
            stamp = self.camerainfo.header.stamp 
            self.tfbx.sendTransform((0, 0, 0), 
                                    (0,0,0,1), 
                                    stamp, 
                                    "Camera", "Camera0")
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    stamp, 
                                    "ImageRaw", "Camera")
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    stamp, 
                                    "ImageRect", "ImageRaw")
            self.tfbx.sendTransform((self.xMask,
                                     self.yMask,
                                     self.zMask),
                                    (0,0,0,1), 
                                    stamp, 
                                    "Arena", "ImageRect")
      
        
    def Main(self):
        rate = rospy.Rate(100)
        try:
            while not rospy.is_shutdown():
                try:
                    self.SendTransforms()
                    rate.sleep()
                except tf.Exception:
                    rospy.logwarn ('Exception in TransformServerArenaCamera()')
        except:
            print "Shutting down"


if __name__ == "__main__":
    tspc = TransformServerArenaCamera()
    tspc.Main()

