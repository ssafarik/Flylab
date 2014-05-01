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

        
        # Checkerboard info
        self.sizeCheckerboard = (8,6)
        self.nCols = self.sizeCheckerboard[0]
        self.nRows = self.sizeCheckerboard[1]
        self.nCorners = self.nCols * self.nRows
        self.checker_size = rospy.get_param('calibration/checker_size', 15)
        
        self.sizeWindow = (4,4)
        self.sizeDeadzone = (2,2)
        self.terminationCriteria = (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 100, 0.01)
        
        self.pointsArena         = np.zeros([self.nCorners, 3], dtype=np.float32)
        self.pointsImage         = np.zeros([self.nCorners, 2], dtype=np.float32)
        self.pointsAxes          = np.zeros([4, 3], dtype=np.float32)
        self.pointsAxesProjected = np.zeros([4, 2], dtype=np.float32)

        self.rvec      = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.rvec2     = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.rvec2_avg = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.sumWrap   = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec      = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec2     = np.zeros([1, 3], dtype=np.float32).squeeze()
        self.tvec2_avg = np.zeros([1, 3], dtype=np.float32).squeeze()

        self.nMeasurements = 0
        self.alpha = 0.001 # For moving averages.
        
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

        
    def InitializeImages(self, (height,width)):
        if self.camerainfo is not None:
            self.imgProcessed = np.zeros([height, width],    dtype=np.uint8)
            self.imgMask      = np.zeros([height, width],    dtype=np.uint8)
            self.imgDisplay   = np.zeros([height, width, 3], dtype=np.uint8)
            
            self.ptOriginArena.point.x = self.xMask
            self.ptOriginArena.point.y = self.yMask
            self.bInitialized_images = True


    def PrepareImageCorners(self, corners):
        (x0,y0) = (None,None)
        m = None
        a = 0.01
        for iCorner in range(self.nCorners):
            # Idealized position of the corner.
            self.pointsArena[iCorner][0] = (iCorner // self.nCols)*self.checker_size
            self.pointsArena[iCorner][1] = (iCorner % self.nCols)*self.checker_size
            self.pointsArena[iCorner][2] = 0.0

            # Actual position of the corner.
            (x,y) = corners[iCorner]
            self.pointsImage[iCorner][0] = x
            self.pointsImage[iCorner][1] = y

#             # Find the mean distance between neighbor points.            
#             if (x0 is not None):
#                 d = np.linalg.norm([x-x0,y-y0])
#                 if (m is not None):
#                     if (d < m*2):
#                         m = (1-a)*m + a*d
#                 else:
#                     m = d
#                     
#             (x0,y0) = (x,y)
#             
#         self.pixels_per_mm = m / self.checker_size
            


    def DrawOriginAxes(self, cvimage, rvec, tvec):
        if self.camerainfo is not None:
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
                
            # origin point
            pt1 = tuple(self.pointsAxesProjected[0][0])

            # draw x-axis
            pt2 = tuple(self.pointsAxesProjected[1][0])
            cv2.line(cvimage, pt1, pt2, cv.CV_RGB(self.colorMax,0,0), widthAxisLine)

            # draw y-axis
            pt2 = tuple(self.pointsAxesProjected[2][0])
            cv2.line(cvimage, pt1, pt2, cv.CV_RGB(0,self.colorMax,0), widthAxisLine)
            
            # draw z-axis
            pt2 = tuple(self.pointsAxesProjected[3][0])
            cv2.line(cvimage, pt1, pt2, cv.CV_RGB(0,0,self.colorMax), widthAxisLine)
                
            

    def FindExtrinsics(self):
        if self.camerainfo is not None:
            (bFoundChessboard, cornersImage) = cv2.findChessboardCorners(self.imgProcessed, self.sizeCheckerboard)
            if (bFoundChessboard) and (len(cornersImage)==self.nCorners):
                cv2.cornerSubPix(self.imgProcessed, cornersImage, self.sizeWindow, self.sizeDeadzone, self.terminationCriteria)
                cv2.drawChessboardCorners(self.imgDisplay, self.sizeCheckerboard, cornersImage, bFoundChessboard)
                self.PrepareImageCorners(cornersImage.squeeze())
                (rv, self.rvec, self.tvec) = cv2.solvePnP(self.pointsArena, 
                                                          self.pointsImage, 
                                                          self.K_rect, 
                                                          self.D_rect)
                self.rvec = self.rvec.squeeze()
                self.tvec = self.tvec.squeeze()
                #rospy.logwarn(np.linalg.norm(self.rvec))
                
                (R,jacobian) = cv2.Rodrigues(self.rvec)
                T = tf.transformations.translation_matrix(self.tvec)
                
                RT = np.zeros((3,3))
                RT[:-1,:-1] = R[:-1,:-1]
                RT[0:2,-1] = T[0:2,-1]
                RT[-1,-1] = T[-1,-1]
                
                
                # H is/mightbe the transform from camera point to arena point.
                Hinv = np.dot(self.K_rect, RT)
                Hinv = Hinv / Hinv[-1,-1]
                H = np.linalg.inv(Hinv)
                self.Hinv = Hinv
                self.H = H
                
                
                self.tfbx.sendTransform(self.tvec,
                                          tf.transformations.quaternion_about_axis(np.linalg.norm(self.rvec), self.rvec),
                                          self.stampImage,
                                          FRAME_CHECKERBOARD,
                                          FRAME_IMAGE)
                
                self.DrawOriginAxes(self.imgDisplay, self.rvec, self.tvec)
                self.RemapExtrinsics()


    def RemapExtrinsics(self):
        X = [self.ptOriginArena.point.x, self.ptOriginArena.point.x + self.checker_size]
        Y = [self.ptOriginArena.point.y, self.ptOriginArena.point.y]
        Z = [1,                          1]
        
        pointsCamera = np.array([X,Y,Z])
        pointsArena = np.dot(self.H, pointsCamera).transpose()
        x0 = pointsArena[0,0]
        x1 = pointsArena[1,0]
        y0 = pointsArena[0,1]
        y1 = pointsArena[1,1]
        angleRot = np.arctan2((y1-y0),(x1-x0))
        
        transCheckerboardArena = pointsArena[0,:]
        transCheckerboardArena[2] = 0
        
        try:
            self.tfbx.sendTransform(transCheckerboardArena,
                                    tf.transformations.quaternion_from_euler(0, 0, angleRot),
                                    self.stampImage,
                                    FRAME_ARENA,
                                    FRAME_CHECKERBOARD)
        
            self.tfrx.waitForTransform(FRAME_IMAGE, FRAME_ARENA, self.stampImage, rospy.Duration(1.0))
            (trans, quat) = self.tfrx.lookupTransform(FRAME_IMAGE, FRAME_ARENA, self.stampImage)
            quatMat = tf.transformations.quaternion_matrix(quat)  # Returns a 4x4 numpy.array of float64.
            
            
        except tf.Exception, e:
            rospy.logwarn ('tf.Exception in RemapExtrinsics():  %s' % e)
        else:
            rotMat = quatMat[0:3, 0:3]
            rotMat = rotMat.astype('float32')
            self.rvec2Prev = self.rvec2
            (rvec2, jacobian) = cv2.Rodrigues(rotMat)
            rvec2 = rvec2.squeeze()
            if (np.any(rvec2 != 0.0)):
                self.rvec2 = rvec2
                signWrap = ((np.abs(self.rvec2Prev-self.rvec2)>np.pi) * np.sign(self.rvec2Prev-self.rvec2)).astype('float32')    # An array of -1/0/+1 indicating wrap and direction.
                self.sumWrap += signWrap * 2*np.pi                                           # The cumulative correction for wrapping.
    
                self.tvec2 = np.array(trans)

                if (self.nMeasurements<10):
                    self.rvec2_avg = self.rvec2+self.sumWrap 
                    self.tvec2_avg = self.tvec2 
                    self.rvec2_avg = (self.rvec2_avg+2*np.pi)%(4*np.pi) - 2*np.pi # On range [-2pi,+2pi]
                self.nMeasurements += 1
                    
                part1 = self.rvec2_avg
                part2 = ((self.rvec2+self.sumWrap)+2*np.pi)%(4*np.pi) - 2*np.pi # On range [-2pi,+2pi]
                self.rvec2_avg = (1.0-self.alpha)*part1 + self.alpha*part2
                self.tvec2_avg = (1.0-self.alpha)*self.tvec2_avg + self.alpha*self.tvec2
    
                params = {'arena_tvec_0':float(self.tvec2[0]),
                          'arena_tvec_1':float(self.tvec2[1]),
                          'arena_tvec_2':float(self.tvec2[2]),
                          'arena_rvec_0':float(self.rvec2[0]),
                          'arena_rvec_1':float(self.rvec2[1]),
                          'arena_rvec_2':float(self.rvec2[2])}
                #rospy.set_param('camera', params)

                
                # self.image_arena_origin_found = True
                #self.DrawOriginAxes(self.imgDisplay, self.rvec2, self.tvec2)


    def Image_callback(self, image):
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
                
                display_text = 'checker_size=%0.3f' % self.checker_size
                cv2.putText(self.imgDisplay, display_text,
                           (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
        
                display_text = 'originArena = [%0.0f, %0.0f]' % (self.ptOriginArena.point.x, self.ptOriginArena.point.y)
                cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                # Update image mask
                cv2.circle(self.imgMask, 
                           (int(self.ptOriginArena.point.x), int(self.ptOriginArena.point.y)), 
                           int(self.radiusMask), self.colorMax, cv.CV_FILLED)
                self.imgProcessed = cv2.bitwise_and(self.imgProcessed, self.imgMask)
                self.FindExtrinsics()

                # Draw the arena origin.
                cv2.circle(self.imgDisplay, 
                           (int(self.ptOriginArena.point.x),int(self.ptOriginArena.point.y)), 
                           3, cv.CV_RGB(0,self.colorMax,0), cv.CV_FILLED)
                
                # Draw the mask circle
                cv2.circle(self.imgDisplay, 
                           (int(self.ptOriginArena.point.x),int(self.ptOriginArena.point.y)), 
                           int(self.radiusMask), cv.CV_RGB(0,self.colorMax,0))
                
                
                display_text = 'radiusMask = ' + str(int(self.radiusMask))
                cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText

                display_text = 'rvec cur=[%+3.3f, %+3.3f, %+3.3f] avg=[%3.3f, %3.3f, %3.3f]' % ((self.rvec2[0]+np.pi)%(2*np.pi)-np.pi,     
                                                                                                (self.rvec2[1]+np.pi)%(2*np.pi)-np.pi,     
                                                                                                (self.rvec2[2]+np.pi)%(2*np.pi)-np.pi,
                                                                                                (self.rvec2_avg[0]+np.pi)%(2*np.pi)-np.pi, 
                                                                                                (self.rvec2_avg[1]+np.pi)%(2*np.pi)-np.pi, 
                                                                                                (self.rvec2_avg[2]+np.pi)%(2*np.pi)-np.pi)
                cv2.putText(self.imgDisplay, display_text,
                           (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                display_text = 'tvec cur=[%+3.3f, %+3.3f, %+3.3f] avg=[%3.3f, %3.3f, %3.3f]' % (self.tvec2[0],     self.tvec2[1],     self.tvec2[2],
                                                                                                self.tvec2_avg[0], self.tvec2_avg[1], self.tvec2_avg[2])
                cv2.putText(self.imgDisplay, display_text,
                            (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText

                display_text = 'pixels_per_mm=%3.3f' % (self.K_rect[0,0]/self.tvec2_avg[2])
                cv2.putText(self.imgDisplay, display_text,
                            (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
    
                
                cv2.imshow('Camera Arena Calibration', self.imgDisplay)
                cv2.waitKey(3)

    
    

if __name__ == '__main__':
    cal = CalibrateCameraArena()
    try:
        rospy.spin()
    except:
        print 'Shutting down'
    cv2.destroyAllWindows()

