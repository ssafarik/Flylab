#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('tracking')
import sys
import rospy
import cv
import tf
import numpy as N
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tracking.msg import ContourInfo
from plate_tf.srv import PlateCameraConversion

FILENAME_BACKGROUND="/cameras/background.png"



###############################################################################
###############################################################################
###############################################################################
# The class ContourGenerator subscribes to image_rect, and finds the contours of the objects in the image.
# Publishes a ContourInfo message, and several intermediary images.
#
class ContourGenerator:

    def __init__(self):
        
        # Image Initialization
        self.initialized = False
        self.initialized_images = False
        self.initialized_coords = False
        
        # Messages
        self.camerainfo = None
        self.subCameraInfo       = rospy.Subscriber("camera/camera_info", CameraInfo, self.CameraInfo_callback)
        self.subImage            = rospy.Subscriber("camera/image_rect", Image, self.Image_callback)
        
        self.pubImageProcessed   = rospy.Publisher("camera/image_processed", Image)
        self.pubImageDiff        = rospy.Publisher("camera/image_diff", Image)
        self.pubImageBackground  = rospy.Publisher("camera/image_background", Image)
        self.pubImageForeground  = rospy.Publisher("camera/image_foreground", Image)
        self.pubContourInfo      = rospy.Publisher("ContourInfo", ContourInfo)
        
        self.tfrx = tf.TransformListener()
        
        # Contour Info
        self.contourinfo = ContourInfo()
        self.x0_list = []
        self.y0_list = []
        self.theta_list = []
        self.area_list = []
        self.ecc_list = []
        self.nContours = 0
        self.nContours_max = 20 #int(rospy.get_param("nContours_max", 2))
        self.min_ecc = 1.75
        self.minSumImage = 100
        
        # Robot Info
        #self.robot_visible = bool(rospy.get_param("robot_visible", "true"))
        #if (not self.robot_visible) and (1 < self.nContours_max):
        if (1 < self.nContours_max):
            self.nContours_max -= 1
        
        # Image Windows
        self.display_processedimage = bool(rospy.get_param("display_processedimage","false"))
        self.display_contours       = bool(rospy.get_param("display_contours","false"))
        self.display_diff           = bool(rospy.get_param("display_diff","false"))
        self.display_foreground     = bool(rospy.get_param("display_foreground","false"))
        self.display_background     = bool(rospy.get_param("display_background","false"))
        
        if self.display_processedimage:
            cv.NamedWindow("Processed Image", 1)
        if self.display_diff:
            cv.NamedWindow("Diff Image", 1)
        if self.display_foreground:
            cv.NamedWindow("Foreground Image", 1)
        if self.display_background:
            cv.NamedWindow("Background Image", 1)
        
        # OpenCV
        self.max_8U = 255
        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.storage = cv.CreateMemStorage()
        self.cvbridge = CvBridge()
        
        # Coordinate Systems
        self.frameidOutput = rospy.get_param("frameid_contours","ImageRect")
        
        self.ptsOriginROI = PointStamped()
        self.ptsOriginROI.header.frame_id = "ROI"
        self.ptsOriginROI.point.x = 0
        self.ptsOriginROI.point.y = 0
        
        self.ptsOriginPlate = PointStamped()
        self.ptsOriginPlate.header.frame_id = "Plate"
        self.ptsOriginPlate.point.x = 0
        self.ptsOriginPlate.point.y = 0
        
        self.tfrx.waitForTransform("ImageRect", self.ptsOriginROI.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        self.tfrx.waitForTransform("ROI", self.ptsOriginPlate.header.frame_id, rospy.Time(), rospy.Duration(5.0))

#        rospy.wait_for_service('camera_from_plate', timeout=10.0)
#        try:
#            self.camera_from_plate = rospy.ServiceProxy('camera_from_plate', PlateCameraConversion)
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e
#        rospy.wait_for_service('plate_from_camera', timeout=10.0)
#        try:
#            self.plate_from_camera = rospy.ServiceProxy('plate_from_camera', PlateCameraConversion)
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

        
        while (not self.initialized_coords):
            try:
                self.ptsOriginROI_ImageRect = self.tfrx.transformPoint("ImageRect", self.ptsOriginROI)
                self.ptsOriginPlate_ROI = self.tfrx.transformPoint("ROI", self.ptsOriginPlate)
                self.initialized_coords = True
        
            except (tf.Exception):
                rospy.logwarn ('tf.Exception')
                rospy.sleep(1.0)
        
        
        # Mask Info
        self.radiusMask = int(rospy.get_param("camera/mask/radius", 25))

#        rospy.logwarn('100 ImageRect = %s Plate' % self.MmFromPixels(100))
#        rospy.logwarn('100 Plate = %s ImageRect' % self.PixelsFromMm(100))
        
        self.initialized = True
        

    def CameraInfo_callback (self, msgCameraInfo):
            self.camerainfo = msgCameraInfo

    
    def MmFromPixels (self, xIn):
        response = self.camera_from_plate(xIn, xIn)
        return (response.Xdst, response.Ydst)
        
        
    def PixelsFromMm (self, xIn):
        ptsIn = PointStamped()
        ptsIn.header.frame_id = "Plate"
        ptsIn.point.x = 0.0
        ptsIn.point.y = 0.0
        ptsOrigin = self.tfrx.transformPoint("ImageRect", ptsIn)        
        
        ptsIn.point.x = xIn
        ptsIn.point.y = xIn
        ptsOut = self.tfrx.transformPoint("ImageRect", ptsIn)
                
        return (ptsOut.point.x - ptsOrigin.point.x)
        
        
    def InitializeImages(self, imagecvRect):
        if self.camerainfo is not None:
            self.sizeImageRect = cv.GetSize(imagecvRect)
            
            # ROI Setup
            self.widthROI = self.camerainfo.width
            self.heightROI = self.camerainfo.height
            self.sizeROI = (self.widthROI, self.heightROI)
            self.rectROI = (int(self.ptsOriginROI_ImageRect.point.x),
                            int(self.ptsOriginROI_ImageRect.point.y),
                            int(self.widthROI),
                            int(self.heightROI))
            
            
            self.image                  = cv.CreateImage(self.sizeImageRect,cv.IPL_DEPTH_8U,1)
            self.imageProcessed         = cv.CreateImage(self.sizeImageRect,cv.IPL_DEPTH_8U,3)
            cv.SetImageROI(self.imageProcessed, self.rectROI)
            self.imageProcessed2        = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,3)
            self.imageMask              = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageBackground        = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageForeground        = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageForeground_binary = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            #self.imageDilate            = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            #self.imageErode             = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageZeros             = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageContour           = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,1)
            self.imageContourDisplay    = cv.CreateImage(self.sizeROI,cv.IPL_DEPTH_8U,3)
            cv.Zero(self.imageZeros)
            cv.Zero(self.imageMask)
            cv.Circle(self.imageMask,
                      (int(self.ptsOriginPlate_ROI.point.x),-int(self.ptsOriginPlate_ROI.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
            #rospy.logwarn('ROI=%s' % [int(self.ptsOriginPlate_ROI.point.x),int(self.ptsOriginPlate_ROI.point.y), int(self.radiusMask)])
            
            # Create Background image
            # First image is background unless one can be loaded
            #q = cv.GetSize(imagecvRect)
            #rospy.logwarn('size1=%s' % [q[0],q[1]])
            cv.SetImageROI(imagecvRect, self.rectROI)
            
            #q = cv.GetSize(imagecvRect)
            #rospy.logwarn('size2=%s' % [q[0],q[1]])
            try:
                self.imageBackground = cv.LoadImage(FILENAME_BACKGROUND, cv.CV_LOAD_IMAGE_GRAYSCALE)
            except:
                rospy.logwarn ('Saving new background image %s' % FILENAME_BACKGROUND)
                cv.And(imagecvRect, self.imageMask, self.imageBackground)
                cv.SaveImage(FILENAME_BACKGROUND, self.imageBackground)
                #q = cv.GetSize(self.imageBackground)
                #rospy.logwarn('size3=%s' % [q[0],q[1]])
              
            self.initialized_images = True
            

    def FindAngleEcc(self, A, B, C, D):
        angle = float('NaN')
        ecc = 1.0
        
        if C != 0: # Div by zero.
            inside = A*A + 4*B*C - 2*A*D + D*D
            if inside >= 0: # Complex answer.
                inside = N.sqrt(inside)
                evalA = 0.5*(A+D-inside)
                evalB = 0.5*(A+D+inside)
                evecA1 = (-A+D+inside)/(-2*C)
                evecB1 = (-A+D-inside)/(-2*C)
                rise = 1
                try:
                    if evalB < evalA:
                        run = evecA1
                        ecc = evalA/evalB
                      
                    else:
                        run = evecB1
                        ecc = evalB/evalA
                      
                    angle = -N.arctan2(rise, run)
                    #rospy.loginfo ('IP angle=%s, rise/run=%s/%s' % (angle, rise,run))

                except:
                    rospy.logwarn ('Exception in FindAngleEcc()')
                    pass
        
        if N.isnan(angle):
            angle = 0.0
              
        return angle, ecc
        

    def ContourFromMoments(self, moments):
        Mu00 = cv.GetSpatialMoment(moments,0,0)
        Mu10 = cv.GetSpatialMoment(moments,1,0)
        Mu01 = cv.GetSpatialMoment(moments,0,1)
        
        if Mu00 != 0:
            x = Mu10/Mu00
            y = Mu01/Mu00
        
        else:
            x = None
            y = None
          
        Uu11 = cv.GetCentralMoment(moments,1,1)
        Uu20 = cv.GetCentralMoment(moments,2,0)
        Uu02 = cv.GetCentralMoment(moments,0,2)
        area = Mu00
        angle, ecc = self.FindAngleEcc(Uu20, Uu11, Uu11, Uu02)
        
        return x, y, area, angle, ecc
    
    
    def DrawAngleLine(self, image, x0, y0, angle, ecc, length):
        if (not N.isnan(angle)) and (self.min_ecc < ecc):
            height, width = cv.GetSize(image)
            y0 = height-y0
            
            # line segment for orientation
            xmin = 0
            ymin = 0
            xmax = width-1
            ymax = height-1
    
            r = length/2
            x1 = x0 - r * N.cos(angle)
            y1 = y0 - r * N.sin(angle)
            x2 = x0 + r * N.cos(angle)
            y2 = y0 + r * N.sin(angle)
            # BUG: What happens if x,y are offscreen?
            
            
            #rospy.logwarn ('IP %s, %s, %s, %s, %s, %s, %s, %s' % (x1, width, xg, xo, x0, y0, angle, ecc))
            cv.Line(image, 
                    (int(x1),int(y1)),
                    (int(x2),int(y2)), 
                    cv.CV_RGB(0,self.color_max,0))

    
    # DrawImContour()
    #   Draws the given contour_seq onto self.imageContour.
    #
    def DrawImContour(self, contour_seq):
        imageContourMask = cv.CloneImage(self.imageZeros)
        cv.DrawContours(imageContourMask, 
                        contour_seq, 
                        cv.CV_RGB(0,0,self.color_max), 
                        cv.CV_RGB(0,self.color_max,0), 
                        -1, 
                        cv.CV_FILLED)
        cv.And(self.imageForeground, imageContourMask, self.imageContour)
      
      
      
    def AppendContour(self, contour_seq):
        #seq = cv.FindContours(self.imageContour, cv.CreateMemStorage()) # Finds contours from imageContour, not imageForeground_binary.
        moments = cv.Moments(contour_seq)
        (xROI, yROI, area, theta, ecc) = self.ContourFromMoments(moments)
        
        
        # Save contour info
        #rospy.loginfo ('IP area=%5.5f' % area)
        if True: #area>0.0:
            # Convert from ROI coordinates to Camera coordinates
            ptContourROI = PointStamped()
            ptContourROI.header.frame_id = "ROI"
            ptContourROI.point.x = xROI
            ptContourROI.point.y = yROI
            try:
                self.ptsOutput = self.tfrx.transformPoint(self.frameidOutput, ptContourROI)
                self.x0_list.append(self.ptsOutput.point.x)
                self.y0_list.append(self.ptsOutput.point.y)
                self.theta_list.append(theta)
                self.area_list.append(area)
                self.ecc_list.append(ecc)
                self.nContours += 1

            except tf.Exception:
                rospy.logwarn ('Cannot transform point to frame=%s from frame=%s' % (self.frameidOutput,ptContourROI.header.frame_id))
                self.ptsOutput = PointStamped()
            except TypeError:
                pass

        


    def ShowContourWindow(self, iContour):            
        if self.display_contours:
            cv.NamedWindow("Contour %s"%str(iContours), 1)
            #cv.NamedWindow("Contour", 1)
            cv.CvtColor(self.imageContour, self.imageContourDisplay, cv.CV_GRAY2RGB)
            
            display_text = "Area = " + str(int(area))
            cv.PutText(self.imageContourDisplay, display_text,(25,25),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "ecc = " + str(round(ecc,2))
            cv.PutText(self.imageContourDisplay, display_text,(int(self.widthROI/2),25),self.font,cv.CV_RGB(self.color_max,0,0))
            
            cv.Circle(self.imageContourDisplay, (int(xROI),int(yROI)), 4, cv.CV_RGB(0,self.color_max,0))
            length = 100
            self.DrawAngleLine(self.imageContourDisplay, xROI, yROI, angle, ecc, length)
            
            display_text = "x0 = " + str(int(self.ptsOutput.point.x))
            cv.PutText(self.imageContourDisplay, display_text,(25,45),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "y0 = " + str(int(self.ptsOutput.point.y))
            cv.PutText(self.imageContourDisplay, display_text,(int(self.widthROI/2),45),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "theta = " + str(round(theta,2))
            cv.PutText(self.imageContourDisplay, display_text,(25,65),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "output coordinates = " + self.frameidOutput
            cv.PutText(self.imageContourDisplay, display_text,(25,85),self.font,cv.CV_RGB(self.color_max,0,0))
            
            cv.ShowImage("Contour %s" % str(iContours), self.imageContourDisplay)
            #cv.ShowImage("Contour", self.imageContourDisplay)

    
      
    def ContourinfoFromImage(self, image):
        self.x0_list = []
        self.y0_list = []
        self.theta_list = []
        self.area_list = []
        self.ecc_list = []
        
        # Find contours
        sumImage = cv.Sum(image)
        if self.minSumImage < sumImage[0]:
            contour_seq = cv.FindContours(image, self.storage, mode=cv.CV_RETR_CCOMP)
        else:
            contour_seq = None
        
        # Process contours
        self.nContours = 0
        if contour_seq is not None:
            while True:
                self.DrawImContour(contour_seq)
                self.AppendContour(contour_seq) # self.nContours++ gets incremented inside function.
                self.ShowContourWindow(self.nContours)
                if (contour_seq.h_next()) and (self.nContours < self.nContours_max):
                    contour_seq = contour_seq.h_next()
                else:
                    break
        
            # Reset contour_seq to the start.
            while contour_seq.h_prev():
                contour_seq = contour_seq.h_prev()
            
        
        # display_text = str(self.nContours)
        # cv.PutText(self.im_display, display_text,(25,25), self.font, cv.CV_RGB(self.color_max,0,0))
        
        # Put contours into contourinfo.
        contourinfo = ContourInfo()
        contourinfo.header.stamp = rospy.Time.now()
        contourinfo.header.frame_id = self.frameidOutput # i.e. Camera
        
        if self.nContours != 0:
            contourinfo.x = self.x0_list
            contourinfo.y = self.y0_list
            contourinfo.theta = self.theta_list
            contourinfo.area = self.area_list
            contourinfo.ecc = self.ecc_list
        
        # Remove duplicates
        if self.nContours != 0:
            # Repackage the data.
            contours = []
            for iContour in range(self.nContours):
                contours.append([contourinfo.x[iContour], 
                                 contourinfo.y[iContour], 
                                 contourinfo.theta[iContour], 
                                 contourinfo.area[iContour], 
                                 contourinfo.ecc[iContour]])
            
            # Remove the dups.
            contours = sorted(tuple(contours))
            contours = [x for i, x in enumerate(contours) if (not i) or (N.linalg.norm(N.array(x[0:2])-N.array(contours[i-1][0:2]))>0.1)]
        
            
            # Repackage the cleaned data.
            self.nContours = len(contours)
            contourinfo.x = []
            contourinfo.y = []
            contourinfo.theta = []
            contourinfo.area = []
            contourinfo.ecc = []
            for iContour in range(self.nContours):
                contourinfo.x.append(contours[iContour][0])
                contourinfo.y.append(contours[iContour][1])
                contourinfo.theta.append(contours[iContour][2])
                contourinfo.area.append(contours[iContour][3])
                contourinfo.ecc.append(contours[iContour][4])
            
            
        
            # TESTING...
        #       contourinfo.x = [0.0, 10.0]
        #       contourinfo.y = [0.0, 20.0]
        #       contourinfo.theta = [0.0, 0.0]
        #       contourinfo.area = [100.0, 100.0]
        #       contourinfo.ecc = [1.0, 1.0]
        
            #rospy.loginfo ('IP contourinfo.x,y = %s, %s' %  (contourinfo.x, contourinfo.y))      
            self.pubContourInfo.publish(contourinfo)
        
        return contourinfo, contour_seq    
        

    def Image_callback(self, image):
        if not self.initialized:
            return
        #rospy.logwarn ('image callback, stamp=%s' % image.header.stamp)

        # Convert ROS image to OpenCV image
        try:
            imagecvRect = cv.GetImage(self.cvbridge.imgmsg_to_cv(image, "passthrough"))
        except CvBridgeError, e:
          rospy.logwarn ('Exception %s' % e)
        
        if not self.initialized_images:
            if self.initialized_coords:
                self.InitializeImages(imagecvRect)
            else:
                return
        
        radiusMask = int(rospy.get_param("camera/mask/radius", 25)) 
        if radiusMask != self.radiusMask:
            self.radiusMask = radiusMask 
            cv.Zero(self.imageMask)
            cv.Circle(self.imageMask,
                      (int(self.ptsOriginPlate_ROI.point.x),-int(self.ptsOriginPlate_ROI.point.y)),
                      int(self.radiusMask), 
                      self.color_max, 
                      cv.CV_FILLED)
        
        self.image = imagecvRect
        cv.SetImageROI(self.image, self.rectROI)

        
        # Look for new diff_threshold value
        self.diff_threshold = int(rospy.get_param("diff_threshold",50))
        
        # Apply mask and Subtract background
        #q1 = cv.GetSize(self.image)
        #q2 = cv.GetSize(self.imageMask)
        #rospy.logwarn ('%s' % [[q1[0],q1[1]],[q2[0],q2[1]]])
        cv.And(self.image, self.imageMask, self.image)
        cv.AbsDiff(self.image, self.imageBackground, self.imageForeground)
        if self.display_diff:
            cv.ShowImage("Diff Image", self.imageForeground)
        
        # Threshold
        cv.Threshold(self.imageForeground, 
                     self.imageForeground_binary, 
                     self.diff_threshold, 
                     self.max_8U, 
                     cv.CV_THRESH_BINARY)
        cv.Threshold(self.imageForeground, 
                     self.imageForeground, 
                     self.diff_threshold, 
                     self.max_8U, 
                     cv.CV_THRESH_TOZERO)
        # cv.Dilate(self.imageForeground,self.imageDilate)
        # cv.Erode(self.imageDilate,self.imageErode)
        # cv.ShowImage("Dilate Plus Erode Image", self.imageErode)

        
        # Get the ContourInfo.
        (self.contourinfo, self.contour_seq) = self.ContourinfoFromImage(self.imageForeground_binary)
        #rospy.logwarn ('IP self.contourinfo=\n%s\n, self.contour_seq\n=%s' % (self.contourinfo,self.contour_seq))
        
        
        # Convert to color for display image
        cv.CvtColor(self.image, self.imageProcessed, cv.CV_GRAY2RGB)
        
        # Draw contours on Processed image.
        if self.contour_seq:
            cv.DrawContours(self.imageProcessed, self.contour_seq, cv.CV_RGB(0,0,self.color_max), cv.CV_RGB(0,self.color_max,0), 1, 1)
        
        
        if self.display_processedimage:
            cv.ShowImage("Processed Image", self.imageProcessed)
        
        if self.display_foreground:
            cv.ShowImage("Foreground Image", self.imageForeground)
        
        if self.display_background:
            cv.ShowImage("Background Image", self.imageBackground)
        
        cv.WaitKey(3)
        
        
        # Publish processed image
        try:
            cv.Copy(self.imageProcessed, self.imageProcessed2)
            image2 = self.cvbridge.cv_to_imgmsg(self.imageProcessed2, "passthrough")
            image2.header = image.header
            self.pubImageProcessed.publish(image2)
        except (CvBridgeError, ROSException), e:
            rospy.logwarn ('Exception %s' % e)
        
        # Publish background image
        try:
            image2 = self.cvbridge.cv_to_imgmsg(self.imageBackground, "passthrough")
            image2.header.stamp = image.header.stamp
            self.pubImageBackground.publish(image2)
        except (CvBridgeError, ROSException), e:
            rospy.logwarn ('Exception %s' % e)
          
        # Publish foreground image
        try:
            image2 = self.cvbridge.cv_to_imgmsg(self.imageForeground, "passthrough")
            image2.header.stamp = image.header.stamp
            self.pubImageForeground.publish(image2)
        except (CvBridgeError, ROSException), e:
            rospy.logwarn ('Exception %s' % e)
          
        # Publish diff image
        try:
            image2 = self.cvbridge.cv_to_imgmsg(self.imageForeground, "passthrough")
            image2.header.stamp = image.header.stamp
            self.pubImageDiff.publish(image2)
        except (CvBridgeError, ROSException), e:
            rospy.logwarn ('Exception %s' % e)


    def Main(self):
        rospy.spin()
      

def main(args):
    rospy.init_node('ContourGenerator') #, anonymous=True)
    ip = ContourGenerator()
    try:
        ip.Main()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
