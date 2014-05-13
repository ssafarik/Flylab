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
from experiment_srvs.srv import Trigger, ExperimentParams, ExperimentParamsChoices



class TransformServerArenaCamera:
    def __init__(self):
        self.bInitialized = False
        self.bInitializedConstructor = False
        
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

        self.a_H_i = None
        self.a_H3_i = None
        self.i_Hinv_a = np.identity(3)
        self.i_Hinv3_a = np.identity(4)
        
        self.subImageRect            = rospy.Subscriber('camera/image_rect',            Image,             self.Image_callback, queue_size=2, buff_size=262144, tcp_nodelay=True)
        self.subCameraInfo           = rospy.Subscriber('camera/camera_info',           CameraInfo,        self.CameraInfo_callback)
        self.subCalibrationOriginate = rospy.Subscriber('camera/calibration_originate', CalibrationCamera, self.Calibration_callback)
        self.subCalibrationSet       = rospy.Subscriber('camera/calibration_set',       CalibrationCamera, self.Calibration_callback)
        self.pubCalibrationSet       = rospy.Publisher('camera/calibration_set',        CalibrationCamera, latch=True)

        self.pubImageArena                = rospy.Publisher('camera_arena/image_rect', Image)
        self.pubCamerainfoArena           = rospy.Publisher('camera_arena/camera_info', CameraInfo)
        self.pubImageImageRectCenter      = rospy.Publisher('camera_imagerectcenter/image_rect', Image)
        self.pubCamerainfoImageRectCenter = rospy.Publisher('camera_imagerectcenter/camera_info', CameraInfo)


        self.services = {}
        self.services['image_from_arena'] = rospy.Service('image_from_arena', arena_tf.srv.ArenaCameraConversion, self.ImageFromArena_callback)
        self.services['arena_from_image'] = rospy.Service('arena_from_image', arena_tf.srv.ArenaCameraConversion, self.ArenaFromImage_callback)
        
        self.services['transformserverarenacamera/init']            = rospy.Service('transformserverarenacamera/init',            ExperimentParamsChoices, self.Init_callback)
        self.services['transformserverarenacamera/trial_start']     = rospy.Service('transformserverarenacamera/trial_start',     ExperimentParams,        self.TrialStart_callback)
        self.services['transformserverarenacamera/trial_end']       = rospy.Service('transformserverarenacamera/trial_end',       ExperimentParams,        self.TrialEnd_callback)
        self.services['transformserverarenacamera/trigger']         = rospy.Service('transformserverarenacamera/trigger',         Trigger,                 self.Trigger_callback)
        self.services['transformserverarenacamera/wait_until_done'] = rospy.Service('transformserverarenacamera/wait_until_done', ExperimentParamsChoices, self.WaitUntilDone_callback)
        
        
        self.bInitializedConstructor = True


    # Receive calibration values (wherever they came from), and republish them (for the bag file).
    # Either:
    # -- file -> params -> OriginateCalibrationArena.py -> here.
    # or
    # -- bagfile -> here.
    def Calibration_callback (self, calibration):
        while (not self.bInitializedConstructor):
            rospy.loginfo('Waiting for bInitializedConstructor.')
            rospy.sleep(0.1)
        
            
        self.calibration = calibration
        self.calibration.arena_rvec_2 = 0.0 # We don't care about rotation around z-axis.


    def Init_callback(self, experimentparamsChoices):
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

    def WaitUntilDone_callback(self, experimentparamsChoices):
        return True
        
        
        
    def ComputeTransforms(self):
#        if (self.i_Hinv_a is None):
            if (self.calibration is not None) and (self.K_rect is not None):
                rvec = np.array([self.calibration.arena_rvec_0, self.calibration.arena_rvec_1, self.calibration.arena_rvec_2], dtype=np.float32)
                tvec = np.array([self.calibration.arena_tvec_0, self.calibration.arena_tvec_1, self.calibration.arena_tvec_2], dtype=np.float32)
                (R,jacobian) = cv2.Rodrigues(rvec)              # 3x3 
                T = tf.transformations.translation_matrix(tvec) # 4x4, for a 3D transform.
        
                # Make a 2D extrinsic transform.  Drop the Z coord.
                RT2 = np.zeros((3,3))
                RT2[:-1,:-1] = R[:-1,:-1]
                RT2[0:2,-1] = -np.dot(R,T[0:3,-1])[0:2] #T[0:2,-1]
                RT2[-1,-1] = 1.0
                
                self.i_Hinv_a = np.dot(self.K_rect, RT2)    # i_Hinv_a transforms arena->imagerect.
                try:
                    self.a_H_i = np.linalg.inv(self.i_Hinv_a)
                except Exception, e:
                    rospy.logwarn('Exception inverting i_Hinv_a: %s.  Is camera_info getting published?' % e)

                
                # The whole 3D extrinsic transform.
                RT3 = copy.copy(T[0:3,0:4])
                RT3[0:3,0:3] = R
                
                # The whole 3D transform.
                self.i_Hinv3_a = np.append(np.dot(self.K_rect, RT3),[[0,0,0,1]],0)
                #rospy.logwarn(self.i_Hinv3_a)
                try:
                    self.a_H3_i = np.linalg.inv(self.i_Hinv3_a)
                except Exception, e:
                    rospy.logwarn('Exception inverting i_Hinv3_a: %s.  Is camera_info getting published?' % e)
                
                
        
    def Image_callback(self, rosimage):
        # Point to the cache non-working entry.
        iLoading = 1-self.iWorking
        self.cache[iLoading] = rosimage 
    
    
    def ProcessImage(self):
        if (self.cache[self.iWorking] is not None):
            self.PublishImages()
            
            # Mark this entry as done.
            self.cache[self.iWorking] = None
            
        # Go to the other image.
        self.iWorking = 1-self.iWorking
        
        
        
        
    def CameraInfo_callback (self, camerainfo):
        self.camerainfo = camerainfo

        if (self.calibration is not None):
            self.scale = self.camerainfo.P[0] / self.calibration.arena_tvec_2
        
        self.K_raw = np.reshape(np.array(self.camerainfo.K),[3,3])
        self.D_raw = np.array(self.camerainfo.D)
        
        if (self.calibration is not None):
            P = np.reshape(np.array(self.camerainfo.P),[3,4])
            self.K_rect      = P[0:3,0:3] # P is the matrix that applies to image_rect.
            self.D_rect      = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.K_rect = None
            self.D_rect = None
            

        self.ComputeTransforms()

            

    # ImageFromArena_callback()
    # Get image coordinates from arena coordinates.
    #
    def ImageFromArena_callback(self, req):
        if (self.i_Hinv_a is not None) and (self.bInitializedConstructor):
            nPoints = min(len(req.xSrc), len(req.ySrc))
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [self.calibration.arena_tvec_2]*nPoints
            oSrc = [1]*nPoints
            #ptaArena = np.array([xSrc, ySrc, zSrc, oSrc])
            #ptaImage = np.dot(self.i_Hinv3_a, ptaArena)
            #T = np.array([self.K_rect[0,2], self.K_rect[1,2], 0])


            ptaArena = np.array([xSrc, ySrc, oSrc])
            ptaImage = np.dot(self.i_Hinv_a, ptaArena)

            xDst = ptaImage[0,:]
            yDst = ptaImage[1,:]
            rospy.logwarn(ptaImage[2,:])
        else:
            xDst = []
            yDst = []
            
        return {'xDst': xDst,
                'yDst': yDst}

    # ArenaFromImage_callback()
    # Get arena coordinates from image coordinates.
    #
    def ArenaFromImage_callback(self, req):
        if (self.a_H_i is not None) and (self.bInitializedConstructor):
            nPoints = min(len(req.xSrc), len(req.ySrc))
            
            # Image points.
            xSrc = list(req.xSrc)
            ySrc = list(req.ySrc)
            zSrc = [1]*nPoints
            ptaImage = np.array([xSrc, ySrc, zSrc])
            
            # Principal point.
            C = np.array([[self.K_rect[0,2]], [self.K_rect[1,2]], [0]]) 
            
            
            ptaArena = np.dot(self.a_H_i, ptaImage)
    
            xDst = ptaArena[0,:]
            yDst = ptaArena[1,:]
        else:
            xDst = []
            yDst = []
            
        return {'xDst': xDst,
                'yDst': yDst}
        

#     def SendTransforms(self):      
#         if (self.bInitializedConstructor) and (self.camerainfo is not None):
#             self.tfbx.sendTransform((0,0,0), 
#                                     (0,0,0,1), 
#                                     self.camerainfo.header.stamp, 
#                                     'ImageRaw', 'Camera')
# 
#             # Center of the image_rect relative to the image_raw.
#             if (self.K_raw is not None) and (self.K_rect is not None):
#                 x = (self.K_raw[0,2] - self.K_rect[0,2]) #/ self.scale
#                 y = (self.K_raw[1,2] - self.K_rect[1,2]) #/ self.scale
#                 z = 0.0
#                 self.tfbx.sendTransform((x,y,z), (0,0,0,1), self.camerainfo.header.stamp, 
#                                         'ImageRect', 'ImageRaw')
#             
#             
#             if (self.K_rect is not None):
#                 x = self.K_rect[0,2] #/ self.scale
#                 y = self.K_rect[1,2] #/ self.scale
#                 z = 0.0
#                 self.tfbx.sendTransform((x,y,z), (0,0,0,1), self.camerainfo.header.stamp, 
#                                         'ImageRectCenter', 'ImageRect')
# 
# 
#             if (self.calibration is not None):
#                 x = self.calibration.arena_tvec_0
#                 y = self.calibration.arena_tvec_1
#                 z = self.calibration.arena_tvec_2
#                 rx = self.calibration.arena_rvec_0
#                 ry = self.calibration.arena_rvec_1
#                 rz = self.calibration.arena_rvec_2
#                 q = tf.transformations.quaternion_from_euler(rx, ry, rz)
# 
#                 self.tfbx.sendTransform((self.calibration.xMask,# * self.scale,
#                                          self.calibration.yMask,# * self.scale,
#                                          0.0),
#                                         (0,0,0,1), 
#                                         self.camerainfo.header.stamp, 
#                                         'Mask', 'ImageRect')
#       
#                 self.tfbx.sendTransform((0,0,z), q, self.camerainfo.header.stamp, 
#                                         'ViewCenter', 'ImageRectCenter')
#     
#                 x =   self.calibration.xMask - self.K_rect[0,2]
#                 y = -(self.calibration.yMask - self.K_rect[1,2])
#                 z = self.calibration.arena_tvec_2
#                 self.tfbx.sendTransform((0,0,z), q, self.camerainfo.header.stamp, 
#                                         'Arena', 'Mask')

    def SendTransforms(self):      
        if (self.bInitializedConstructor) and (self.camerainfo is not None):
            #q = tf.transformations.quaternion_from_euler(self.rvec[0], self.rvec[1], self.rvec[2])
            tvec = (self.calibration.arena_tvec_0,
                    self.calibration.arena_tvec_1,
                    self.calibration.arena_tvec_2)
            rvec = (self.calibration.arena_rvec_0,
                    self.calibration.arena_rvec_1,
                    self.calibration.arena_rvec_2)
            q = tf.transformations.quaternion_about_axis(np.linalg.norm(rvec), rvec)


            # ImageRaw is the same as Camera.
            self.tfbx.sendTransform((0,0,0), 
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'ImageRaw', 'Camera')

            # Center of the image_rect relative to the image_raw.
            if (self.K_raw is not None) and (self.K_rect is not None):
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


            self.tfbx.sendTransform((self.calibration.xMask / self.scale,
                                     self.calibration.yMask / self.scale,
                                     0.0),
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'Mask', 'ImageRect')
  
            self.tfbx.sendTransform((0,0,tvec[2]), (0,0,0,1), self.camerainfo.header.stamp, 
                                    'ViewCenter', 'ImageRectCenter')

#             self.tfbx.sendTransform(tvec, q, self.camerainfo.header.stamp,
#                                     FRAME_CHECKERBOARD,
#                                     'ImageRectCenter')
                
            x = (self.calibration.xMask - self.K_rect[0,2]) / self.scale
            y = (self.calibration.yMask - self.K_rect[1,2]) / self.scale
            self.tfbx.sendTransform((0,0,tvec[2]), q, self.camerainfo.header.stamp, 
                                    'Arena', 'Mask')

            
    def PublishImages(self):
        if (self.camerainfo is not None) and (self.cache[self.iWorking] is not None):
            # Publish a special image for use in rviz.
            K = copy.deepcopy(np.reshape(np.array(self.camerainfo.K),[3,3]))
            P = copy.deepcopy(np.reshape(np.array(self.camerainfo.P),[3,4]))
#             K[0:2,0:2] /= rospy.get_param('scale',1.0)
#             P[0:2,0:2] /= rospy.get_param('scale',1.0)
#             K[0,2] += rospy.get_param('x',0.0)
#             K[1,2] += rospy.get_param('y',0.0)
#             P[0,2] += rospy.get_param('x',0.0)
#             P[1,2] += rospy.get_param('y',0.0)
            
            #rospy.logwarn('****************')
            #rospy.logwarn(P)
            
            frame_id = 'Arena'
            camerainfo2 = copy.deepcopy(self.camerainfo)
            camerainfo2.header.frame_id = frame_id
#             camerainfo2.D = (0.0, 0.0, 0.0, 0.0, 0.0)
#             camerainfo2.K = tuple(np.reshape(K,[1,9]).squeeze())#/ self.scale)
#             camerainfo2.P = tuple(np.reshape(P,[1,12]).squeeze())#/ self.scale)
            image2 = copy.deepcopy(self.cache[self.iWorking])
            image2.header.frame_id = frame_id

            self.pubCamerainfoArena.publish(camerainfo2)
            self.pubImageArena.publish(image2)
        
            frame_id = 'ImageRectCenter'
            camerainfo2 = copy.deepcopy(self.camerainfo)
            camerainfo2.header.frame_id = frame_id
#             camerainfo2.D = (0.0, 0.0, 0.0, 0.0, 0.0)
#             camerainfo2.K = tuple(np.reshape(K,[1,9]).squeeze())#/ self.scale)
#             camerainfo2.P = tuple(np.reshape(P,[1,12]).squeeze())#/ self.scale)
            image2 = copy.deepcopy(self.cache[self.iWorking])
            image2.header.frame_id = frame_id

            self.pubCamerainfoImageRectCenter.publish(camerainfo2)
            self.pubImageImageRectCenter.publish(image2)
        
    

        
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

