#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import cv2
import numpy as np
import tf
from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
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
        rospy.sleep(1)
        
        self.tfbx = tf.TransformBroadcaster()

        

        self.Hinv = np.identity(3)
        self.subCameraInfo           = rospy.Subscriber('camera/camera_info',           CameraInfo,        self.CameraInfo_callback)
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
        
        
        
    def GetTransformArenaFromImageRect(self, camerainfo):
        if (self.calibration is not None) and (self.camerainfo is None):
            P = np.reshape(np.array(camerainfo.P),[3,4])
            #K = np.reshape(np.array(camerainfo.K),[3,3])
            K = P[0:3,0:3] # P is the matrix that applies to image_rect.
            
    
            rvec = np.array([self.calibration.arena_rvec_0, self.calibration.arena_rvec_1, self.calibration.arena_rvec_2], dtype=np.float32)
            tvec = np.array([self.calibration.arena_tvec_0, self.calibration.arena_tvec_1, self.calibration.arena_tvec_2], dtype=np.float32)
            (R,jacobian) = cv2.Rodrigues(rvec)
            T = tf.transformations.translation_matrix(tvec)         # 4x4, for a 3D transform.
    
            # Make a 2D [R|T] transform.  Drop the Z coord.
            RT = np.zeros((3,3))
            RT[:-1,:-1] = R[:-1,:-1]
            RT[:,-1] = T[:-1,-1]
    
            Hinv = np.dot(K, RT)
            Hinv = Hinv / Hinv[-1,-1]    # Hinv transforms arena->camera.
            
#             rospy.logwarn(P)
#             rospy.logwarn(K)
#             rospy.logwarn(R)
#             rospy.logwarn(T)
#             rospy.logwarn(RT)
#             rospy.logwarn(np.dot(K, RT))
#             rospy.logwarn(Hinv)
#             rospy.logwarn('-------')
        else:
            Hinv = None
            
        return Hinv
        
        
    def CameraInfo_callback (self, camerainfo):
        if (self.calibration is not None) and (self.camerainfo is None):
            self.Hinv = self.GetTransformArenaFromImageRect(camerainfo)
            if (self.Hinv is not None):
                try:
                    self.H = np.linalg.inv(self.Hinv)       # H transforms camera->arena.
                except Exception, e:
                    rospy.logwarn('Exception inverting Hinv: %s.  Is camera_info getting published?' % e)
                
                self.initialized = True
            
        if (camerainfo is not None):
            self.camerainfo = camerainfo
            

    def CameraFromArena_callback(self, req):
        if (self.initialized):
            point_count = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*point_count
            arena_points = np.array([xSrc, ySrc, zSrc])

            imagerect_points = np.dot(self.Hinv, arena_points)
            #P = np.reshape(np.array(self.camerainfo.P),[3,4])
            #Pinv = np.linalg.inv(P[0:3,0:3])
            #imagerect_points = np.dot(Pinv, arena_points)

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
            imagerect_points = np.array([xSrc, ySrc, zSrc])
            
            arena_points = np.dot(self.H, imagerect_points)
            #P = np.reshape(np.array(self.camerainfo.P),[3,4])[0:3,0:3]
            #arena_points = np.dot(P, imagerect_points)
    
            xDst = arena_points[0,:]
            yDst = arena_points[1,:]
        else:
            xDst = None
            yDst = None
            
        return {'xDst': xDst,
                'yDst': yDst}
        

    def SendTransforms(self):      
        if (self.initialized) and (self.camerainfo is not None):
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'ImageRaw', 'Camera')
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'ImageRect', 'ImageRaw')
            params = rospy.get_param('camera')
            x = params['arena_tvec_0']
            y = params['arena_tvec_1']
            z = params['arena_tvec_2']
            rx = params['arena_rvec_0']
            ry = params['arena_rvec_1']
            rz = params['arena_rvec_2']
            q = tf.transformations.quaternion_from_euler(rx, ry, rz)

            self.tfbx.sendTransform((x,y,z), q, self.camerainfo.header.stamp, 'Arena', 'ImageRect')
#             tfs = TransformStamped()
#             tfs.header          = Header()
#             tfs.header.stamp    = self.camerainfo.header.stamp
#             tfs.header.frame_id = 'ImageRect'
#             tfs.child_frame_id  = 'Arena'
#             tfs.transform       = Transform(translation=(x,y,z), rotation=q)
#             self.tfbx.sendTransform(tfs)

            self.tfbx.sendTransform((self.calibration.xMask,
                                     self.calibration.yMask,
                                     0.0),
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'Mask', 'ImageRect')
#             tfs.header.frame_id = 'ImageRect'
#             tfs.child_frame_id  = 'Mask'
#             tfs.transform       = Transform(translation=(self.calibration.xMask, self.calibration.yMask, 0.0), rotation=(0,0,0,1))
#             self.tfbx.sendTransform(tfs)
      
        
    def Main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #try:
            self.SendTransforms()
            rate.sleep()
            #except (tf.Exception, rospy.exceptions.ROSInterruptException), e:
            #    pass # ROS shutting down.
                
        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            


if __name__ == '__main__':
    tspc = TransformServerArenaCamera()
    tspc.Main()

