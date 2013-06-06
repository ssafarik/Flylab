#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import cv2
import numpy as N
import tf
from sensor_msgs.msg import Image, CameraInfo
from arena_tf.msg import CalibrationCamera
import arena_tf.srv
from experiment_srvs.srv import Trigger, ExperimentParams



class TransformServerArenaCamera:
    def __init__(self):
        self.initialized = False
        self.initConstructor = False
        
        self.camerainfo = None
        self.calibration = None
        rospy.init_node('TransformServerArenaCamera')
        
        self.tfbx = tf.TransformBroadcaster()

        

        self.Hinv = N.identity(3)
        self.subCameraInfo           = rospy.Subscriber("camera/camera_info",           CameraInfo,        self.CameraInfo_callback)
        self.subCalibrationOriginate = rospy.Subscriber('camera/calibration_originate', CalibrationCamera, self.Calibration_callback)
        self.subCalibrationSet       = rospy.Subscriber('camera/calibration_set',       CalibrationCamera, self.Calibration_callback)
        self.pubCalibrationSet       = rospy.Publisher('camera/calibration_set',        CalibrationCamera, latch=True)


        self.services = {}
        self.services['camera_from_arena'] = rospy.Service('camera_from_arena', arena_tf.srv.ArenaCameraConversion, self.CameraFromArena_callback)
        self.services['arena_from_camera'] = rospy.Service('arena_from_camera', arena_tf.srv.ArenaCameraConversion, self.ArenaFromCamera_callback)
        
        self.services['transformserverarenacamera/init']            = rospy.Service('transformserverarenacamera/init',            ExperimentParams, self.Init_callback)
        self.services['transformserverarenacamera/trial_start']     = rospy.Service('transformserverarenacamera/trial_start',     ExperimentParams, self.TrialStart_callback)
        self.services['transformserverarenacamera/trial_end']       = rospy.Service('transformserverarenacamera/trial_end',       ExperimentParams, self.TrialEnd_callback)
        self.services['transformserverarenacamera/trigger']         = rospy.Service('transformserverarenacamera/trigger',         Trigger,          self.Trigger_callback)
        self.services['transformserverarenacamera/wait_until_done'] = rospy.Service('transformserverarenacamera/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)
        
        
        self.initConstructor = True


    # Receive calibration values (wherever they came from), and republish them (for the bag file).
    # Either:
    # -- file -> params -> OriginateCalibrationArena.py -> here.
    # or
    # -- bagfile -> here.
    def Calibration_callback (self, calibration):
        while (not self.initConstructor):
            rospy.loginfo('Waiting for initConstructor.')
            rospy.sleep(0.1)
        
            
        self.calibration = calibration


    def Init_callback(self, experimentparams):
        return True

    # TrialStart_callback()
    # Publishes the calibration.
    #
    def TrialStart_callback(self, experimentparams):
        while (self.calibration is None):
            rospy.sleep(0.5)

        self.pubCalibrationSet.publish(self.calibration)
            
        return True
                
                
    def Trigger_callback(self, reqTrigger):
        return True
    
    def TrialEnd_callback(self, experimentparams):
        return True

    def WaitUntilDone_callback(self, experimentparams):
        return True
        
        
        
        
    def CameraInfo_callback (self, camerainfo):
        if (self.calibration is not None) and (self.camerainfo is None):
            K = N.reshape(N.array(camerainfo.K),[3,3])
            P = N.reshape(N.array(camerainfo.P),[3,4])
            
    
            rvec = N.array([self.calibration.arena_rvec_0, self.calibration.arena_rvec_1, self.calibration.arena_rvec_2], dtype=N.float32)
            tvec = N.array([self.calibration.arena_tvec_0, self.calibration.arena_tvec_1, self.calibration.arena_tvec_2], dtype=N.float32)
            (R,jacobian) = cv2.Rodrigues(rvec)
            T = tf.transformations.translation_matrix(tvec)         # 4x4, for a 3D transform.
    
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
            self.initialized = True
            
        if (self.camerainfo is not None):
            self.camerainfo = camerainfo
            

    def CameraFromArena_callback(self, req):
        if (self.initialized):
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
        if (self.initialized):
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
        if (self.initialized):
            self.tfbx.sendTransform((0, 0, 0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    "Camera", "Camera0")
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    "ImageRaw", "Camera")
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    "ImageRect", "ImageRaw")
            self.tfbx.sendTransform((self.calibration.xMask,
                                     self.calibration.yMask,
                                     0.0),
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    "Arena", "ImageRect")
      
        
    def Main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                self.SendTransforms()
                rate.sleep()
            except tf.Exception:
                rospy.logwarn ('Exception in TransformServerArenaCamera()')
                
        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            


if __name__ == "__main__":
    tspc = TransformServerArenaCamera()
    tspc.Main()

