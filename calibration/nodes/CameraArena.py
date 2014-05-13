#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('calibration')
import rospy
import copy
import cv
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo

FRAME_CHECKERBOARD='Checkerboard'
FRAME_IMAGE='ImageRect'
FRAME_ARENA='Arena'


class CalibrateCameraArena:
    def __init__(self):
        self.bInitialized = False
        self.bInitialized_images = False
        rospy.init_node('CalibrateCameraArena')

        self.cvbridge = CvBridge()
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        self.subCameraInfo  = rospy.Subscriber('camera/camera_info', CameraInfo, self.CameraInfo_callback)
        self.subImage       = rospy.Subscriber('camera/image_rect', Image, self.Image_callback)
        self.subPoint       = rospy.Subscriber('Joystick/Commands', Point, self.Point_callback)

        self.pubImageArena                = rospy.Publisher('camera_arena/image_rect', Image)
        self.pubCamerainfoArena           = rospy.Publisher('camera_arena/camera_info', CameraInfo)
        
        self.pubImageImageRectCenter      = rospy.Publisher('camera_imagerectcenter/image_rect', Image)
        self.pubCamerainfoImageRectCenter = rospy.Publisher('camera_imagerectcenter/camera_info', CameraInfo)
        
        self.cache = [None, None]
        self.iWorking = 0
        
        self.camerainfo = None
        self.colorMax = 255
        self.colorFont = cv.CV_RGB(self.colorMax,0,0)
        
        self.ptOriginArena = PointStamped()
        self.ptOriginArena.header.frame_id = FRAME_IMAGE

        params = rospy.get_param('camera', {})
        if ('mask' in params):
            self.xMask = params['mask']['x']
            self.yMask = params['mask']['y']
            self.radiusMask = params['mask']['radius']
        else:
            self.xMask = 0
            self.yMask = 0
            self.radiusMask = 10

        self.scale = 1.0
        
        # Checkerboard info
        self.shapeCheckerboard = (8,6)
        self.nCols = self.shapeCheckerboard[0]
        self.nRows = self.shapeCheckerboard[1]
        self.nCorners = self.nCols * self.nRows
        self.checker_size = rospy.get_param('calibration/checker_size', 15)
        
        self.shapeWindow = (4,4)
        self.shapeDeadzone = (2,2)
        self.terminationCriteria = (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 100, 0.01)
        
        self.pointsAxes          = np.zeros([4, 3], dtype=np.float32)
        self.pointsAxesProjected = np.zeros([4, 2], dtype=np.float32)

        self.rvec      = np.ones([1, 3], dtype=np.float32).squeeze()
        self.rvec0     = np.ones([1, 3], dtype=np.float32).squeeze()
        self.rvec2     = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec      = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec2     = np.zeros([1, 3], dtype=np.float32).squeeze()

        self.rvec_avg  = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.rvec2_avg = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec_avg  = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec2_avg = np.zeros([1, 3], dtype=np.float32).squeeze()
        
        self.sumWrap   = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.sumWrap2  = np.zeros([1, 3], dtype=np.float32).squeeze()

        self.nMeasurements = 0
        self.alpha = 0.001 # For moving averages.
        
        self.a_H_i = None
        self.i_Hinv_a = None
        
        self.bInitialized = True
    
    
    def CameraInfo_callback (self, msgCameraInfo):
        self.camerainfo = msgCameraInfo
        self.K_rect = np.reshape(self.camerainfo.P,[3,4])[0:3,0:3] # camerainfo.K and camerainfo.D apply to image_raw; P w/ D=0 applies to image_rect.
        self.D_rect = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
      

    def Point_callback(self, point):
        if self.bInitialized and self.bInitialized_images:
            self.ptOriginArena.point.x += point.x
            self.ptOriginArena.point.y += -point.y
            self.radiusMask += point.z
            
            rospy.set_param('camera/mask/x', self.ptOriginArena.point.x)
            rospy.set_param('camera/mask/y', self.ptOriginArena.point.y)

        
    def InitializeImages(self, (height,width)):
        if self.camerainfo is not None:
            self.imgProcessed = np.zeros([height, width],    dtype=np.uint8)
            self.imgMask      = np.zeros([height, width],    dtype=np.uint8)
            self.imgDisplay   = np.zeros([height, width, 3], dtype=np.uint8)
            
            self.ptOriginArena.point.x = self.xMask
            self.ptOriginArena.point.y = self.yMask
            self.bInitialized_images = True


    # Make two lists of the corners.  One for the arena, and one for the image, in prep for solvePnP().
    def PrepareImageCorners(self, corners):
        pointsArena         = np.zeros([self.nCorners, 3], dtype=np.float32)
        pointsImage         = np.zeros([self.nCorners, 2], dtype=np.float32)

        for iCorner in range(self.nCorners):
            #iRow = (self.nRows-1)-(iCorner // self.nCols) # reverse
            iRow = iCorner // self.nCols
            iCol = iCorner % self.nCols
            xA = iCol*self.checker_size - ((self.nCols-1)*self.checker_size)/2
            yA = iRow*self.checker_size - ((self.nRows-1)*self.checker_size)/2
            
            # Idealized position of the checkerboard corners in mm, centered at the center of all the squares.
            pointsArena[iCorner][0] = xA
            pointsArena[iCorner][1] = yA
            pointsArena[iCorner][2] = 0.0

            # Actual position of the corner in pixels.
            (xI,yI) = corners[iCorner]
            pointsImage[iCorner][0] = xI
            pointsImage[iCorner][1] = yI

        return (pointsArena, pointsImage)
            


    def DrawOriginAxes(self, image, rvec, tvec):
        if (self.camerainfo is not None):
            self.pointsAxes       = np.zeros([4, 3], dtype=np.float32) #cv.CreateMat(4, 3, cv.CV_32FC1)
            self.pointsAxes[0][:] = 0.0               # (0,0,0)T origin point,    pt[0] 
            self.pointsAxes[1][0] = self.checker_size # (1,0,0)T point on x-axis, pt[1]
            self.pointsAxes[2][1] = self.checker_size # (0,1,0)T point on y-axis, pt[2]
            self.pointsAxes[3][2] = self.checker_size # (0,0,1)T point on z-axis, pt[3]
            widthAxisLine = 3
            
            (self.pointsAxesProjected,jacobian) = cv2.projectPoints(self.pointsAxes,
                                                                    rvec,
                                                                    tvec,
                                                                    self.K_rect,
                                                                    self.D_rect
                                                                    )
            try:
                # origin point
                pt1 = tuple(self.pointsAxesProjected[0][0])
    
                # draw x-axis
                pt2 = tuple(self.pointsAxesProjected[1][0])
                cv2.line(image, pt1, pt2, cv.CV_RGB(self.colorMax,0,0), widthAxisLine)
    
                # draw y-axis
                pt2 = tuple(self.pointsAxesProjected[2][0])
                cv2.line(image, pt1, pt2, cv.CV_RGB(0,self.colorMax,0), widthAxisLine)
                
                # draw z-axis
                pt2 = tuple(self.pointsAxesProjected[3][0])
                cv2.line(image, pt1, pt2, cv.CV_RGB(0,0,self.colorMax), widthAxisLine)
            except OverflowError:
                pass
                
            

    # FindExtrinsics()
    # Compute the extrinsic parameters: rvec & tvec.
    #
    def FindExtrinsics(self):
        if (self.camerainfo is not None):
            (bFoundChessboard, cornersImage) = cv2.findChessboardCorners(self.imgProcessed, self.shapeCheckerboard)
            if (bFoundChessboard) and (len(cornersImage)==self.nCorners):
                cv2.cornerSubPix(self.imgProcessed, cornersImage, self.shapeWindow, self.shapeDeadzone, self.terminationCriteria)
                cv2.drawChessboardCorners(self.imgDisplay, self.shapeCheckerboard, cornersImage, bFoundChessboard)
                (pointsArena, pointsImage) = self.PrepareImageCorners(cornersImage.squeeze())
                pointsArena3 = np.append(pointsArena, np.ones([pointsArena.shape[0],1]), 1) # Augmented points.
                #rospy.logwarn((pointsArena, pointsImage))
                self.rvecPrev = self.rvec
                (rv, self.rvec, self.tvec) = cv2.solvePnP(pointsArena, 
                                                          pointsImage, 
                                                          self.K_rect, 
                                                          self.D_rect,
                                                          self.rvec,
                                                          self.tvec,
                                                          useExtrinsicGuess=False)
                self.nMeasurements += 1

                self.rvec = self.rvec.squeeze()
                self.tvec = self.tvec.squeeze()

                self.rvec0 = copy.deepcopy(self.rvec)
                self.rvec0[2] = 0.0 # We don't care about checkerboard rotation around z-axis.
                
                # Mean values of rvec & tvec.
                if (np.any(self.rvec != 0.0)):
                    diff = self.rvecPrev - self.rvec
                    signWrap = ((np.abs(diff)>np.pi) * np.sign(diff)).astype('float32')    # An array of -1/0/+1 indicating wrap and direction.
                    self.sumWrap += signWrap * 2*np.pi                                     # The cumulative correction for wrapping.
        
                    if (self.nMeasurements<100):
                        self.rvec_avg = ((self.rvec0+self.sumWrap)+2*np.pi)%(4*np.pi) - 2*np.pi
                        self.tvec_avg = self.tvec 
                        
                    part1 = self.rvec_avg
                    part2 = ((self.rvec0+self.sumWrap)+2*np.pi)%(4*np.pi) - 2*np.pi # On range [-2pi,+2pi]
                    self.rvec_avg = (1.0-self.alpha)*part1 + self.alpha*part2
                    self.tvec_avg = (1.0-self.alpha)*self.tvec_avg + self.alpha*self.tvec
                
                
                self.scale = self.camerainfo.P[0] / self.tvec[2]
                
                if (np.isnan(self.rvec[0])): # This happens if pointsArena[i][2]==0.0, above in PrepareImageCorners().
                    self.rvec      = np.ones([1, 3], dtype=np.float32).squeeze()
                    self.tvec      = np.zeros([1, 3], dtype=np.float32).squeeze()
                    
                (R,jacobian) = cv2.Rodrigues(self.rvec)
                T = tf.transformations.translation_matrix(self.tvec)
                #T0 = -np.dot(R,T[0:3,-1])
                T0 = T[0:3,-1]
                
                # Make a 2D extrinsic transform out of 3D transforms.
                RT2 = np.zeros((3,3))
                RT2[:-1,:-1] = R[:-1,:-1]
                RT2[0:2,-1]  = T0[0:2] # T[0:2,-1]
                RT2[-1,-1]   = 1.0
                
                rospy.logwarn((R,T,RT2))
                
                # RT2 and RT3 will take a point from arena space to image space. 
#                 rospy.logwarn('***************')
#                 rospy.logwarn(np.dot(RT2, pointsArena.T).T)
#                 rospy.logwarn(np.dot(RT3, pointsArena3.T).T)
                
                # pixels = intrinsic * extrinsic * millimeters
                # pixels = K * RT * x
                # i_Hinv_a: image <- arena

                # a_H_i: arena <- image
                self.i_Hinv_a = np.dot(self.K_rect, RT2)# /= self.scale
                self.a_H_i = np.linalg.inv(self.i_Hinv_a)

                
                # The whole 3D extrinsic transform.
                RT3 = copy.copy(T[0:3,0:4])
                RT3[0:3,0:3] = R
                
                self.i_Hinv3_a = np.append(np.dot(self.K_rect, RT3),[[0,0,0,1]],0)
                self.a_H3_i = np.linalg.inv(self.i_Hinv3_a)

#                 rospy.logwarn('K_rect: %s' % self.K_rect)
#                 rospy.logwarn('RT2: %s' % RT2)
#                 rospy.logwarn('RT3: %s' % RT3)
#                 rospy.logwarn('a_H_i: %s' % self.a_H_i)
#                 rospy.logwarn('i_Hinv_a: %s' % self.i_Hinv_a)
#                 rospy.logwarn('a_H3_i: %s' % self.a_H3_i)
#                 rospy.logwarn('i_Hinv3_a: %s' % self.i_Hinv3_a)
                
                

    def SendTransforms(self):      
        if (self.bInitialized) and (self.camerainfo is not None):
            rvec = self.rvec0
            tvec = self.tvec
            
            #q = tf.transformations.quaternion_from_euler(self.rvec[0], self.rvec[1], self.rvec[2])
            q = tf.transformations.quaternion_about_axis(np.linalg.norm(rvec), rvec)

#             rospy.logwarn('******************')
#             rospy.logwarn(self.scale)
#             rospy.logwarn(self.K_rect)
#             rospy.logwarn(self.a_H_i)
#             rospy.logwarn(self.i_Hinv_a)
            
            x = self.K_rect[0,2] / self.scale
            y = self.K_rect[1,2] / self.scale
            z = 0.0
            self.tfbx.sendTransform((x,y,z), (0,0,0,1), self.camerainfo.header.stamp, 
                                    'ImageRectCenter', 'ImageRect')


            self.tfbx.sendTransform((self.xMask / self.scale,
                                     self.yMask / self.scale,
                                     0.0),
                                    (0,0,0,1), 
                                    self.camerainfo.header.stamp, 
                                    'Mask', 'ImageRect')
  
            self.tfbx.sendTransform((0,0,tvec[2]), (0,0,0,1), self.camerainfo.header.stamp, 
                                    'ViewCenter', 'ImageRectCenter')

            self.tfbx.sendTransform((tvec[0], tvec[1], tvec[2]), q, self.camerainfo.header.stamp,
                                    FRAME_CHECKERBOARD, 'ImageRectCenter')
                
            x = (self.xMask - self.K_rect[0,2]) / self.scale
            y = (self.yMask - self.K_rect[1,2]) / self.scale
            self.tfbx.sendTransform((0,0,tvec[2]), q, self.camerainfo.header.stamp, 
                                    'Arena', 'Mask')
            
            
                                        

    def Image_callback(self, image):
        # Point to the cache non-working entry.
        iLoading = (self.iWorking+1) % 2
        self.cache[iLoading] = image 
    
    
    def ProcessImage(self):
        if (self.cache[self.iWorking] is not None):
            image = self.cache[self.iWorking]

            if self.bInitialized:
                self.stampImage = image.header.stamp
                try:
                    imgInput = np.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, 'passthrough')))
                except CvBridgeError, e:
                    rospy.logwarn('Exception CvBridgeError in image callback: %s' % e)
    
                
                if (not self.bInitialized_images):
                    self.InitializeImages(imgInput.shape)
                
                if self.bInitialized_images:
                    xText = 25
                    yText = 25
                    dyText = 20
                    
                    self.imgProcessed = copy.copy(imgInput)
                    self.imgDisplay = cv2.cvtColor(imgInput, cv.CV_GRAY2RGB)
                    

                    # *************************
                    display_text = 'checker_size=%0.3fmm' % self.checker_size
                    cv2.putText(self.imgDisplay, display_text,
                               (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
            

                    # *************************
                    display_text = 'originArena = [%0.0f, %0.0f]' % (self.ptOriginArena.point.x, self.ptOriginArena.point.y)
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    
                    # Update image mask
                    #cv2.circle(self.imgMask, 
                    #           (int(self.ptOriginArena.point.x), int(self.ptOriginArena.point.y)), 
                    #           int(self.radiusMask), self.colorMax, cv.CV_FILLED)
                    #self.imgProcessed = cv2.bitwise_and(self.imgProcessed, self.imgMask)

                    
                    # *************************
                    # Compute tvec & rvec.
                    self.FindExtrinsics()


                    # *************************
                    self.DrawOriginAxes(self.imgDisplay, self.rvec, self.tvec)


                    # *************************
                    # Draw the arena origin.
                    cv2.circle(self.imgDisplay, 
                               (int(self.ptOriginArena.point.x),int(self.ptOriginArena.point.y)), 
                               3, cv.CV_RGB(0,self.colorMax,0), cv.CV_FILLED)
                    
                    # *************************
                    # Draw the mask circle
                    cv2.circle(self.imgDisplay, 
                               (int(self.ptOriginArena.point.x),int(self.ptOriginArena.point.y)), 
                               int(self.radiusMask), cv.CV_RGB(0,self.colorMax,0))
                    
                    
                    # *************************
                    display_text = 'radiusMask = ' + str(int(self.radiusMask))
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText

    
                    # *************************
                    display_text = 'rvec cur=[%+3.3f, %+3.3f, %+3.3f] avg=[%3.3f, %3.3f, %3.3f]' % ((self.rvec[0]+np.pi)%(2*np.pi)-np.pi,     
                                                                                                    (self.rvec[1]+np.pi)%(2*np.pi)-np.pi,     
                                                                                                    (self.rvec[2]+np.pi)%(2*np.pi)-np.pi,
                                                                                                    (self.rvec_avg[0]+np.pi)%(2*np.pi)-np.pi, 
                                                                                                    (self.rvec_avg[1]+np.pi)%(2*np.pi)-np.pi, 
                                                                                                    (self.rvec_avg[2]+np.pi)%(2*np.pi)-np.pi)
                    cv2.putText(self.imgDisplay, display_text,
                               (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText

                    
                    # *************************
                    display_text = 'tvec cur=[%+3.3f, %+3.3f, %+3.3f] avg=[%3.3f, %3.3f, %3.3f]' % (self.tvec[0],     self.tvec[1],     self.tvec[2],
                                                                                                    self.tvec_avg[0], self.tvec_avg[1], self.tvec_avg[2])
                    cv2.putText(self.imgDisplay, display_text,
                                (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText

    
                    # *************************
                    if (self.tvec2_avg[2] != 0.0):
                        display_text = 'pixels_per_mm=%3.3f' % (self.K_rect[0,0]/self.tvec2_avg[2])
                        cv2.putText(self.imgDisplay, display_text,
                                    (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
        
                    
                    cv2.imshow('Camera Arena Calibration', self.imgDisplay)
                    cv2.waitKey(3)
    
    
            self.PublishImages()

            # Mark this entry as done.
            self.cache[self.iWorking] = None
            
        # Go to the other image.
        self.iWorking = (self.iWorking+1) % 2


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
        
    

if __name__ == '__main__':
    cal = CalibrateCameraArena()
    while (not rospy.is_shutdown()):
        cal.SendTransforms()
        cal.ProcessImage()
        
    cv2.destroyAllWindows()

