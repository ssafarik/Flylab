#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('calibration')
import rospy
import copy
import cv
import cv2
import numpy as N
import tf
from cv_bridge import CvBridge, CvBridgeError
from pythonmodules import cvNumpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo

FRAME_CHECKERBOARD="Checkerboard"
FRAME_IMAGERECT="ImageRect"
FRAME_ARENA="Arena"
FRAME_CAMERA="Camera"


class CalibrateCameraArena:
    def __init__(self):
        self.initialized = False
        self.initialized_images = False
        rospy.init_node('CalibrateCameraArena')

        self.cvbridge = CvBridge()
        
        self.tfrx = tf.TransformListener()
        self.tfbx = tf.TransformBroadcaster()
        self.subCameraInfo  = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        self.subImage       = rospy.Subscriber("camera/image_rect", Image, self.Image_callback)

        self.camerainfo = None
        self.colorMax = 255
        self.colorFont = cv.CV_RGB(self.colorMax,0,0)
        
        self.ptOriginArena = PointStamped()
        self.ptOriginArena.header.frame_id = FRAME_CAMERA

        self.ptOriginCamera = PointStamped()
        self.ptOriginCamera.header.frame_id = FRAME_CAMERA
        self.ptOriginCamera.point.x = 0
        self.ptOriginCamera.point.y = 0
        
        
        # Checkerboard info
        self.sizeCheckerboard = (8,6)
        self.nCols = self.sizeCheckerboard[0]
        self.nRows = self.sizeCheckerboard[1]
        self.nCorners = self.nCols * self.nRows
        self.checker_size = rospy.get_param('calibration/checker_size', 15)
        self.xMask = rospy.get_param ('camera/mask/x', 0)
        self.yMask = rospy.get_param ('camera/mask/y', 0)
        self.radiusMask = rospy.get_param ('camera/mask/radius', 10)
        
        self.sizeWindow = (4,4)
        self.sizeDeadzone = (2,2)
        self.criteriaTerm = (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 100, 0.01)
        
        self.pointsArena         = N.zeros([self.nCorners, 3], dtype=N.float32)
        self.pointsImage         = N.zeros([self.nCorners, 2], dtype=N.float32)
        self.pointsAxes          = N.zeros([4, 3], dtype=N.float32)
        self.pointsAxesProjected = N.zeros([4, 2], dtype=N.float32)

        self.rvec      = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.rvec2     = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.rvec2_avg = 0.0
        self.sumWrap   = N.zeros([1, 3], dtype=N.float32).squeeze()
        self.tvec      = N.zeros([1, 3], dtype=N.float32).squeeze()

        self.nMeasurements = 0
        self.alpha = 0.001 # For moving averages.
        
        self.initialized = True
    
    
    def CameraInfo_callback (self, msgCameraInfo):
        self.camerainfo = msgCameraInfo
      
        
    def InitializeImages(self, (height,width)):
        if self.camerainfo is not None:
            self.imgProcessed = N.zeros([height, width],    dtype=N.uint8)
            self.imgMask      = N.zeros([height, width],    dtype=N.uint8)
            self.imgDisplay   = N.zeros([height, width, 3], dtype=N.uint8)
            
            self.ptOriginArena.point.x = self.xMask #width/2 - self.camerainfo.P[2] #self.KK_cx
            self.ptOriginArena.point.y = self.yMask #height/2 - self.camerainfo.P[6] #self.KK_cy
            self.initialized_images = True


    def PrepareImageCorners(self, corners):
        for iCorner in range(self.nCorners):
            (x,y) = corners[iCorner]
            self.pointsArena[iCorner][0] = (iCorner // self.nCols)*self.checker_size
            self.pointsArena[iCorner][1] = (iCorner % self.nCols)*self.checker_size
            self.pointsArena[iCorner][2] = 0.0

            self.pointsImage[iCorner][0] = x
            self.pointsImage[iCorner][1] = y
            

    def DrawOriginAxes(self, cvimage, rvec, tvec):
        if self.camerainfo is not None:
            self.pointsAxes       = N.zeros([4, 3], dtype=N.float32) #cv.CreateMat(4, 3, cv.CV_32FC1)
            self.pointsAxes[0][:] = 0.0               # (0,0,0)T origin point,    pt[0] 
            self.pointsAxes[1][0] = self.checker_size # (1,0,0)T point on x-axis, pt[1]
            self.pointsAxes[2][1] = self.checker_size # (0,1,0)T point on y-axis, pt[2]
            self.pointsAxes[3][2] = self.checker_size # (0,0,1)T point on z-axis, pt[3]
            widthAxisLine = 3
            
            (self.pointsAxesProjected,jacobian) = cv2.projectPoints(self.pointsAxes,
                                                                    rvec,
                                                                    tvec,
                                                                    N.reshape(self.camerainfo.K, [3,3]),
                                                                    N.reshape(self.camerainfo.D, [5,1])
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
                cv2.cornerSubPix(self.imgProcessed, cornersImage, self.sizeWindow, self.sizeDeadzone, self.criteriaTerm)
                cv2.drawChessboardCorners(self.imgDisplay, self.sizeCheckerboard, cornersImage, bFoundChessboard)
                self.PrepareImageCorners(cornersImage.squeeze())
                (rv, self.rvec, self.tvec) = cv2.solvePnP(self.pointsArena, 
                                                          self.pointsImage, 
                                                          N.reshape(self.camerainfo.K,[3,3]), 
                                                          N.reshape(self.camerainfo.D,[5,1]))
                self.rvec = self.rvec.squeeze()
                self.tvec = self.tvec.squeeze()
                
                angleRvec = N.linalg.norm(self.rvec)
                R = tf.transformations.rotation_matrix(angleRvec, self.rvec)
                T = tf.transformations.translation_matrix(self.tvec)
                
                Wsub = N.zeros((3,3))
                Wsub[:,:-1] = R[:-1,:-2]
                Wsub[:,-1] = T[:-1,-1]
                
                M = N.reshape(self.camerainfo.K,[3,3])
                M[:-1,-1] = 0
                
                # H is/mightbe the transform from camera point to arena point.
                Hinv = N.dot(M, Wsub)
                Hinv = Hinv / Hinv[-1,-1]
                H = N.linalg.inv(Hinv)
                self.Hinv = Hinv
                self.H = H
                
                
                self.tfbx.sendTransform(self.tvec,
                                          tf.transformations.quaternion_about_axis(angleRvec, self.rvec),
                                          self.stampImage,
                                          FRAME_CHECKERBOARD,
                                          FRAME_IMAGERECT)
                
                self.DrawOriginAxes(self.imgDisplay, self.rvec, self.tvec)
                self.RemapExtrinsics()


    def RemapExtrinsics(self):
        X = [self.ptOriginArena.point.x, self.ptOriginArena.point.x + self.checker_size]
        Y = [self.ptOriginArena.point.y, self.ptOriginArena.point.y]
        Z = [1,                          1]
        
        pointsCamera = N.array([X,Y,Z])
        pointsArena = N.dot(self.H, pointsCamera).transpose()
        x0 = pointsArena[0,0]
        x1 = pointsArena[1,0]
        y0 = pointsArena[0,1]
        y1 = pointsArena[1,1]
        angleRot = N.arctan2((y1-y0),(x1-x0))
        
        transCheckerboardArena = pointsArena[0,:]
        transCheckerboardArena[2] = 0
        
        try:
            self.tfbx.sendTransform(transCheckerboardArena,
                                    tf.transformations.quaternion_from_euler(0, 0, angleRot),
                                    self.stampImage,
                                    FRAME_ARENA,
                                    FRAME_CHECKERBOARD)
        
            self.tfrx.waitForTransform(FRAME_IMAGERECT, FRAME_ARENA, self.stampImage, rospy.Duration(1.0))
            (trans, quat) = self.tfrx.lookupTransform(FRAME_IMAGERECT, FRAME_ARENA, self.stampImage)
            quatMat = tf.transformations.quaternion_matrix(quat)  # Returns a 4x4 numpy.array of float64.
            
            
        except tf.Exception, e:
            rospy.logwarn ('tf.Exception in RemapExtrinsics():  %s' % e)
        else:
            rotMat = quatMat[0:3, 0:3]
            rotMat = rotMat.astype('float32')
            self.rvec2Prev = self.rvec2
            (rvec2, jacobian) = cv2.Rodrigues(rotMat)
            rvec2 = rvec2.squeeze()
            if (N.any(rvec2 != 0.0)):
                self.rvec2 = rvec2
                signWrap = ((N.abs(self.rvec2Prev-self.rvec2)>N.pi) * N.sign(self.rvec2Prev-self.rvec2)).astype('float32')    # An array of -1/0/+1 indicating wrap and direction.
                self.sumWrap += signWrap * 2*N.pi                                           # The cumulative correction for wrapping.
    
                self.tvec2 = N.array(trans)

                if (self.nMeasurements<10):
                    self.rvec2_avg = self.rvec2+self.sumWrap 
                    self.tvec2_avg = self.tvec2 #N.array(self.tvec_array)
                    self.rvec2_avg = (self.rvec2_avg+2*N.pi)%(4*N.pi) - 2*N.pi # On range [-2pi,+2pi]
                self.nMeasurements += 1
                    
                part1 = self.rvec2_avg
                part2 = ((self.rvec2+self.sumWrap)+2*N.pi)%(4*N.pi) - 2*N.pi # On range [-2pi,+2pi]
                self.rvec2_avg = (1.0-self.alpha)*part1 + self.alpha*part2
                self.tvec2_avg = (1.0-self.alpha)*self.tvec2_avg + self.alpha*self.tvec2
    
                
                # self.image_arena_origin_found = True
                #self.DrawOriginAxes(self.imgDisplay, self.rvec2, self.tvec2)


    def Image_callback(self, image):
        if self.initialized:
            self.stampImage = image.header.stamp
            try:
                imgInput = N.uint8(cv.GetMat(self.cvbridge.imgmsg_to_cv(image, "passthrough")))
            except CvBridgeError, e:
                rospy.logwarn('Exception CvBridgeError in image callback: %s' % e)
            
            if not self.initialized_images:
                self.InitializeImages(imgInput.shape)
            
            if self.initialized_images:
                xText = 25
                yText = 25
                dyText = 20
                
                self.imgProcessed = copy.copy(imgInput)
                self.imgDisplay = cv2.cvtColor(imgInput, cv.CV_GRAY2RGB)
                
                display_text = "checker_size=%0.3f" % self.checker_size
                cv2.putText(self.imgDisplay, display_text,
                           (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
        
                display_text = "originArena = [%0.0f, %0.0f]" % (self.ptOriginArena.point.x, self.ptOriginArena.point.y)
                cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                yText += dyText
                
                try:
                    self.ptOriginCameraUndistorted = self.tfrx.transformPoint(FRAME_IMAGERECT, self.ptOriginCamera) # This only works for the origin point.  Merely put into the image_rect frame.
                    self.ptOriginArenaUndistorted = self.tfrx.transformPoint(FRAME_IMAGERECT, self.ptOriginArena)   # Other points should go through something like image_proc node.
                except tf.Exception, e:
                    rospy.logwarn('tf.Exception in Image_callback(): %s' % e)
                else:
                    # Update image mask
                    cv2.circle(self.imgMask, 
                               (int(self.ptOriginArenaUndistorted.point.x), int(self.ptOriginArenaUndistorted.point.y)), 
                               int(self.radiusMask), self.colorMax, cv.CV_FILLED)
                    self.imgProcessed = cv2.bitwise_and(self.imgProcessed, self.imgMask)
                    self.FindExtrinsics()

                    # Draw the camera origin.
                    cv2.circle(self.imgDisplay, 
                               (int(self.ptOriginCameraUndistorted.point.x),int(self.ptOriginCameraUndistorted.point.y)), 
                               3, cv.CV_RGB(self.colorMax,0,self.colorMax), cv.CV_FILLED)
                    
                    # Draw the arena origin.
                    cv2.circle(self.imgDisplay, 
                               (int(self.ptOriginArenaUndistorted.point.x),int(self.ptOriginArenaUndistorted.point.y)), 
                               3, cv.CV_RGB(0,self.colorMax,0), cv.CV_FILLED)
                    
                    # Draw the mask circle
                    cv2.circle(self.imgDisplay, 
                               (int(self.ptOriginArenaUndistorted.point.x),int(self.ptOriginArenaUndistorted.point.y)), 
                               int(self.radiusMask), cv.CV_RGB(0,self.colorMax,0))
                    
                    
                    display_text = "radiusMask = " + str(int(self.radiusMask))
                    cv2.putText(self.imgDisplay, display_text, (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    
                    display_text = "rvec cur=[%+0.3f, %+0.3f, %+0.3f] avg=[%0.3f, %0.3f, %0.3f]" % ((self.rvec2[0]+N.pi)%(2*N.pi)-N.pi,     
                                                                                                    (self.rvec2[1]+N.pi)%(2*N.pi)-N.pi,     
                                                                                                    (self.rvec2[2]+N.pi)%(2*N.pi)-N.pi,
                                                                                                    (self.rvec2_avg[0]+N.pi)%(2*N.pi)-N.pi, 
                                                                                                    (self.rvec2_avg[1]+N.pi)%(2*N.pi)-N.pi, 
                                                                                                    (self.rvec2_avg[2]+N.pi)%(2*N.pi)-N.pi)
                    cv2.putText(self.imgDisplay, display_text,
                               (xText,yText), cv.CV_FONT_HERSHEY_TRIPLEX, 0.5, self.colorFont)
                    yText += dyText
                    
                    display_text = "tvec=[%+0.3f, %+0.3f, %+0.3f] avg=[%0.3f, %0.3f, %0.3f]" % (self.tvec2[0],     self.tvec2[1],     self.tvec2[2],
                                                                                                self.tvec2_avg[0], self.tvec2_avg[1], self.tvec2_avg[2])
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
        print "Shutting down"
    cv2.destroyAllWindows()

