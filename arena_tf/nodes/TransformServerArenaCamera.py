#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('arena_tf')
import rospy
import cv
import cv2
import copy
import numpy as np
import tf
from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from arena_tf.msg import CalibrationCamera
import arena_tf.srv
from experiment_srvs.srv import Trigger, ExperimentParams
from pythonmodules import SetDict




class TransformServerArenaCamera:
    def __init__(self):
        self.initialized = False
        self.initConstructor = False
        
        self.camerainfo = None
        self.calibration = None
        self.K_raw = None
        self.K_rect = None
        self.scale = 1.0
        
        self.cache = [None, None]
        self.iWorking = 0
        
        
        rospy.init_node('TransformServerArenaCamera')
        rospy.sleep(1)
        
        self.tfbx = tf.TransformBroadcaster()

        self.Hinv = np.identity(3)
        self.subImageRect            = rospy.Subscriber('camera/image_rect',            Image,             self.Image_callback, queue_size=2, buff_size=262144, tcp_nodelay=True)
        self.subCameraInfo           = rospy.Subscriber('camera/camera_info',           CameraInfo,        self.CameraInfo_callback)
        self.subCalibrationOriginate = rospy.Subscriber('camera/calibration_originate', CalibrationCamera, self.Calibration_callback)
        self.subCalibrationSet       = rospy.Subscriber('camera/calibration_set',       CalibrationCamera, self.Calibration_callback)
        self.pubCalibrationSet       = rospy.Publisher('camera/calibration_set',        CalibrationCamera, latch=True)

        self.pubImageRviz           = rospy.Publisher('camera_rviz/image_rect', Image)
        self.pubCamerainfoRviz      = rospy.Publisher('camera_rviz/camera_info', CameraInfo)


        self.services = {}
        self.services['image_from_arena'] = rospy.Service('image_from_arena', arena_tf.srv.ArenaCameraConversion, self.ImageFromArena_callback)
        self.services['arena_from_image'] = rospy.Service('arena_from_image', arena_tf.srv.ArenaCameraConversion, self.ArenaFromImage_callback)
        
        self.services['transformserverarenacamera/init']            = rospy.Service('transformserverarenacamera/init',            ExperimentParams, self.Init_callback)
        self.services['transformserverarenacamera/trial_start']     = rospy.Service('transformserverarenacamera/trial_start',     ExperimentParams, self.TrialStart_callback)
        self.services['transformserverarenacamera/trial_end']       = rospy.Service('transformserverarenacamera/trial_end',       ExperimentParams, self.TrialEnd_callback)
        self.services['transformserverarenacamera/trigger']         = rospy.Service('transformserverarenacamera/trigger',         Trigger,          self.Trigger_callback)
        self.services['transformserverarenacamera/wait_until_done'] = rospy.Service('transformserverarenacamera/wait_until_done', ExperimentParams, self.WaitUntilDone_callback)
        
        
        self.params = rospy.get_param('camera', {})

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
        
        
        
    def GetTransformImagerectFromArena(self, camerainfo):
        if (self.Hinv is not None):
            Hinv = self.Hinv
        else:
            if (self.calibration is not None) and (self.K_rect is not None):
                rvec = np.array([self.calibration.arena_rvec_0, self.calibration.arena_rvec_1, self.calibration.arena_rvec_2], dtype=np.float32)
                tvec = np.array([self.calibration.arena_tvec_0, self.calibration.arena_tvec_1, self.calibration.arena_tvec_2], dtype=np.float32)
                (R,jacobian) = cv2.Rodrigues(rvec)
                T = tf.transformations.translation_matrix(tvec)         # 4x4, for a 3D transform.
        
                # Make a 2D [R|T] transform.  Drop the Z coord.
                RT = np.zeros((3,3))
                RT[:-1,:-1] = R[:-1,:-1]
                RT[0:2,-1] = T[0:2,-1]
                RT[-1,-1] = T[-1,-1]
                
                Hinv = np.dot(self.K_rect, RT)    # Hinv transforms arena->imagerect.
                Hinv = Hinv / self.scale
                
            else:
                Hinv = None
                
            
        return Hinv
        
        
    def Image_callback(self, rosimage):
        # Point to the cache non-working entry.
        iLoading = (self.iWorking+1) % 2
        self.cache[iLoading] = rosimage 
    
    
    def ProcessImage(self):
        if (self.cache[self.iWorking] is not None):
            # Publish a special image for use in rviz.
            camerainfo2 = copy.copy(self.camerainfo)
            camerainfo2.header.frame_id = 'ImageRectCenter'
            camerainfo2.D = (0.0, 0.0, 0.0, 0.0, 0.0)
            camerainfo2.K = tuple(np.array(camerainfo2.K) / self.scale)
            camerainfo2.P = tuple(np.array(camerainfo2.P) / self.scale)

            self.pubCamerainfoRviz.publish(camerainfo2)
            self.pubImageRviz.publish(self.cache[self.iWorking])
                        
            # Mark this entry as done.
            self.cache[self.iWorking] = None
            
        # Go to the other image.
        self.iWorking = (self.iWorking+1) % 2
        
        
        
        
    def CameraInfo_callback (self, camerainfo):
        if (self.calibration is not None):
            self.scale = camerainfo.P[0] / self.calibration.arena_tvec_2
        
        self.K_raw = np.reshape(np.array(camerainfo.K),[3,3])
        self.D_raw = np.array(camerainfo.D)
        
        if (self.calibration is not None):
            P = np.reshape(np.array(camerainfo.P),[3,4])
            self.K_rect      = P[0:3,0:3] # P is the matrix that applies to image_rect.
            self.D_rect      = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.K_rect = None
            self.D_rect = None
            

        self.Hinv = self.GetTransformImagerectFromArena(camerainfo)
        if (self.Hinv is not None):
            try:
                self.H = np.linalg.inv(self.Hinv)       # H transforms imagerect->;arena.
            except Exception, e:
                rospy.logwarn('Exception inverting Hinv: %s.  Is camera_info getting published?' % e)
            
        else:
            self.H = None
            

        if (camerainfo is not None):
            self.camerainfo = camerainfo
            

    # ImageFromArena_callback()
    # Get image coordinates from arena coordinates.
    #
    def ImageFromArena_callback(self, req):
        if (self.Hinv is not None) and (self.initConstructor):
            nPoints = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*nPoints
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

    # ArenaFromImage_callback()
    # Get arena coordinates from image coordinates.
    #
    def ArenaFromImage_callback(self, req):
        if (self.H is not None) and (self.initConstructor):
            nPoints = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*nPoints
            imagerect_points = np.array([xSrc, ySrc, zSrc])
            arena_points = np.dot(self.H, imagerect_points)
            #rospy.logwarn((imagerect_points,arena_points))
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
        if (self.initConstructor) and (self.camerainfo is not None):
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'ImageRaw', 'Camera')

            # Center of the image_rect relative to the image_raw.
            if (self.K_raw is not None):
                x = (self.K_raw[0,2] - self.K_rect[0,2]) / self.scale
                y = (self.K_raw[1,2] - self.K_rect[1,2]) / self.scale
                z = 0.0
                self.tfbx.sendTransform((x,y,z), (0,0,0,1), self.camerainfo.header.stamp, 
                                        'ImageRect', 'ImageRaw')
            
            
            if (self.K_rect is not None):
                x = self.K_rect[0,2] / self.scale
                y = self.K_rect[1,2] / self.scale
                z = 0.0
                self.tfbx.sendTransform((x,y,z), (0,0,0,1), self.camerainfo.header.stamp, 
                                        'ImageRectCenter', 'ImageRect')


            if (self.calibration is not None):
                self.tfbx.sendTransform((self.calibration.xMask / self.scale,
                                         self.calibration.yMask / self.scale,
                                         0.0),
                                        (0,0,0,1), 
                                        self.camerainfo.header.stamp, 
                                        'Mask', 'ImageRect')
      
                x = self.calibration.arena_tvec_0
                y = self.calibration.arena_tvec_1
                z = self.calibration.arena_tvec_2
                rx = self.calibration.arena_rvec_0
                ry = self.calibration.arena_rvec_1
                rz = self.calibration.arena_rvec_2
                q = tf.transformations.quaternion_from_euler(rx, ry, rz)
                self.tfbx.sendTransform((x,y,z), q, self.camerainfo.header.stamp, 
                                        'ViewCenter', 'ImageRectCenter')
    
                x = 0#  self.calibration.xMask - self.K_rect[0,2]
                y = 0#-(self.calibration.yMask - self.K_rect[1,2])
                z = self.calibration.arena_tvec_2
                self.tfbx.sendTransform((x,y,z), q, self.camerainfo.header.stamp, 
                                        'Arena', 'Mask')

        
    def Main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #try:
            self.SendTransforms()
            self.ProcessImage()
            rate.sleep()
            #except (tf.Exception, rospy.exceptions.ROSInterruptException), e:
            #    pass # ROS shutting down.
                
        # Shutdown all the services we offered.
        for key in self.services:
            self.services[key].shutdown()
            


if __name__ == '__main__':
    tspc = TransformServerArenaCamera()
    tspc.Main()

