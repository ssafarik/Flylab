#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import cv2
import numpy as N
import tf
from sensor_msgs.msg import Image, CameraInfo
import arena_tf.srv


class TransformServerArenaCamera:
    def __init__(self):
        self.initialized = False
        self.camerainfo = None
        rospy.init_node('TransformServerArenaCamera')
        
        self.tfbx = tf.TransformBroadcaster()

        self.rvec      = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.tvec      = N.zeros([1, 3], dtype=N.float32).squeeze()
        
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
            K = N.reshape(N.array(camerainfo.K),[3,3])
            P = N.reshape(N.array(camerainfo.P),[3,4])
            
            self.rvec[0] = rospy.get_param('/camera_arena_rvec_0')
            self.rvec[1] = rospy.get_param('/camera_arena_rvec_1')
            self.rvec[2] = rospy.get_param('/camera_arena_rvec_2')
            self.tvec[0] = rospy.get_param('/camera_arena_tvec_0')
            self.tvec[1] = rospy.get_param('/camera_arena_tvec_1')
            self.tvec[2] = rospy.get_param('/camera_arena_tvec_2')
    
            (R,jacobian) = cv2.Rodrigues(self.rvec)
            T = tf.transformations.translation_matrix(self.tvec)         # 4x4, for a 3D transform.
    
            # Make a 2D [R|T] transform.  Drop the Z coord.
            RT = N.zeros((3,3))
            RT[:-1,:-1] = R[:-1,:-1]
            RT[:,-1] = T[:-1,-1]
    
            self.Hinv = N.dot(K, RT)
            self.Hinv = self.Hinv / self.Hinv[-1,-1]
            try:
                self.H = N.linalg.inv(self.Hinv)
            except Exception, e:
                rospy.logwarn('Exception inverting Hinv: %s.  Is camera_info getting published?' % e)
                rospy.logwarn('camerainfo.K = %s' %repr(camerainfo.K))
                rospy.logwarn('Hinv = %s' %repr(self.Hinv))
            
#            rospy.logwarn(M)
#            rospy.logwarn(R)
#            rospy.logwarn(T)
#            rospy.logwarn(RT)
#            rospy.logwarn(self.Hinv)
#            rospy.logwarn(self.H)
        self.camerainfo = camerainfo
            

    def CameraFromArena_callback(self, req):
        if (self.camerainfo is not None):
            point_count = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*point_count
            arena_points = N.array([xSrc, ySrc, zSrc])

            imagerect_points = N.dot(self.Hinv, arena_points)
            #P = N.reshape(N.array(self.camerainfo.P),[3,4])
            #Pinv = N.linalg.inv(P[0:3,0:3])
            #imagerect_points = N.dot(Pinv, arena_points)

            xDst = imagerect_points[0,:]
            yDst = imagerect_points[1,:]
        else:
            xDst = None
            yDst = None
            
        return {'xDst': xDst,
                'yDst': yDst}

    def ArenaFromCamera_callback(self, req):
        if (self.camerainfo is not None):
            point_count = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*point_count
            imagerect_points = N.array([xSrc, ySrc, zSrc])
            
            arena_points = N.dot(self.H, imagerect_points)
            #P = N.reshape(N.array(self.camerainfo.P),[3,4])[0:3,0:3]
            #arena_points = N.dot(P, imagerect_points)
    
            xDst = arena_points[0,:]
            yDst = arena_points[1,:]
        else:
            xDst = None
            yDst = None
            
        return {'xDst': xDst,
                'yDst': yDst}
        

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

