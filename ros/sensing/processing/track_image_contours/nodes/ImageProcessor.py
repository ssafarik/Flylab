#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('track_image_contours')
import sys
import rospy
import cv
import tf
import numpy as N
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from track_image_contours.msg import ContourInfo
from plate_tf.srv import PlateCameraConversion


class ImageProcessor:

    def __init__(self):
        
        # Image Initialization
        self.initialized = False
        self.initialized_images = False
        self.initialized_coords = False
        
        # Messages
        self.sub_image            = rospy.Subscriber("camera/image_rect", Image, self.Image_callback)
        self.pub_image_processed  = rospy.Publisher("camera/image_processed", Image)
        self.pub_image_diff       = rospy.Publisher("camera/image_diff", Image)
        self.pub_image_foreground = rospy.Publisher("camera/image_foreground", Image)
        self.pub_contourinfo      = rospy.Publisher("ContourInfo", ContourInfo)
        self.tfrx = tf.TransformListener()
        
        # Contour Info
        self.contourinfo = ContourInfo()
        self.x0_list = []
        self.y0_list = []
        self.theta_list = []
        self.area_list = []
        self.ecc_list = []
        self.nContours = 0
        self.nContours_max = 20 #int(rospy.get_param("nContours_max", "2"))
        self.min_ecc = 1.75
        self.minSumImage = 100
        
        # Robot Info
        self.robot_visible = bool(rospy.get_param("robot_visible", "true"))
        if (not self.robot_visible) and (1 < self.nContours_max):
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
        self.bridge = CvBridge()
        
        # Coordinate Systems
        self.frameidOutput = rospy.get_param("frameid_contours","Camera")
        self.ROIPlateImage_origin = PointStamped()
        self.ROIPlateImage_origin.header.frame_id = "ROIPlateImage"
        self.ROIPlateImage_origin.point.x = 0
        self.ROIPlateImage_origin.point.y = 0
        self.PlateImage_origin = PointStamped()
        self.PlateImage_origin.header.frame_id = "PlateImage"
        self.PlateImage_origin.point.x = 0
        self.PlateImage_origin.point.y = 0
        
        self.tfrx.waitForTransform("ImageRect", self.ROIPlateImage_origin.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        self.tfrx.waitForTransform("ROIPlateImage", self.PlateImage_origin.header.frame_id, rospy.Time(), rospy.Duration(5.0))

#        rospy.wait_for_service('plate_to_camera', timeout=10.0)
#        try:
#            self.plate_to_camera = rospy.ServiceProxy('plate_to_camera', PlateCameraConversion)
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e
#        rospy.wait_for_service('camera_to_plate', timeout=10.0)
#        try:
#            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

        
        while (not self.initialized_coords):
            try:
                self.undistorted_ROIPlateImage_origin = self.tfrx.transformPoint("ImageRect", self.ROIPlateImage_origin)
                self.ROIPlateImage_PlateImage_origin = self.tfrx.transformPoint("ROIPlateImage", self.PlateImage_origin)
                self.initialized_coords = True
        
            except (tf.LookupException, tf.ConnectivityException):
                rospy.sleep(1.0)
        
        
        # Mask Info
        self.radiusMask = int(rospy.get_param("arena/radius_camera","25.4")) #int(rospy.get_param("radiusMask","225"))
#        rospy.logwarn('100 ImageRect = %s Plate' % self.MmFromPixels(100))
#        rospy.logwarn('100 Plate = %s ImageRect' % self.PixelsFromMm(100))
        
        self.initialized = True
        

    def MmFromPixels (self, xIn):
        response = self.plate_to_camera(xIn, xIn)
        return (response.Xdst, response.Ydst)
        
        
    def PixelsFromMm (self, xIn):
        ptIn = PointStamped()
        ptIn.header.frame_id = "Plate"
        ptIn.point.x = 0.0
        ptIn.point.y = 0.0
        ptOrigin = self.tfrx.transformPoint("ImageRect", ptIn)        
        ptIn.point.x = xIn
        ptIn.point.y = xIn
        ptOut = self.tfrx.transformPoint("ImageRect", ptIn)        
        return (ptOut.point.x - ptOrigin.point.x)
        
        
    def InitializeImages(self,cv_image):
        self.undistorted_size = cv.GetSize(cv_image)
        (self.undistorted_width,self.undistorted_height) = cv.GetSize(cv_image)
        
        # ROI Setup
        self.ROIPlateImage_width = int(rospy.get_param("ROIPlateImage_width", "640"))
        self.ROIPlateImage_height = int(rospy.get_param("ROIPlateImage_height", "480"))
        self.ROIPlateImage_size = (self.ROIPlateImage_width, self.ROIPlateImage_height)
        self.ROIPlateImage_cvrect = (int(self.undistorted_ROIPlateImage_origin.point.x),
                                     int(self.undistorted_ROIPlateImage_origin.point.y),
                                     int(self.ROIPlateImage_width),
                                     int(self.ROIPlateImage_height))
        
        
        self.im                   = cv.CreateImage(self.undistorted_size,cv.IPL_DEPTH_8U,1)
        self.im_processed         = cv.CreateImage(self.undistorted_size,cv.IPL_DEPTH_8U,3)
        cv.SetImageROI(self.im_processed,self.ROIPlateImage_cvrect)
        self.im_processed2        = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,3)
        self.im_mask              = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_background        = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_foreground        = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_foreground_binary = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_dilate            = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_erode             = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_zeros             = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_contour           = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
        self.im_contour_display   = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,3)
        cv.Zero(self.im_zeros)
        # self.center_ROIx = self.ROIwidth//2
        # self.center_ROIy = self.ROIheight//2
        cv.Zero(self.im_mask)
        cv.Circle(self.im_mask,
                  (int(self.ROIPlateImage_PlateImage_origin.point.x),int(self.ROIPlateImage_PlateImage_origin.point.y)),
                  int(self.radiusMask), 
                  self.color_max, 
                  cv.CV_FILLED)
        # Create Background image
        # First image is background unless one can be loaded
        cv.SetImageROI(cv_image,self.ROIPlateImage_cvrect)
        try:
            self.im_background = cv.LoadImage("/cameras/background.png",cv.CV_LOAD_IMAGE_GRAYSCALE)
        except:
            cv.And(cv_image,self.im_mask,self.im_background)
            cv.SaveImage("/cameras/background.png",self.im_background)
          
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
            x = 0
            y = 0
          
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
    #   Draws the given contour_seq onto self.im_contour.
    #
    def DrawImContour(self, contour_seq):
        im_contour_mask = cv.CloneImage(self.im_zeros)
        cv.DrawContours(im_contour_mask, 
                        contour_seq, 
                        cv.CV_RGB(0,0,self.color_max), 
                        cv.CV_RGB(0,self.color_max,0), 
                        -1, 
                        cv.CV_FILLED)
        cv.And(self.im_foreground, im_contour_mask, self.im_contour)
      
      
      
    def AppendContour(self, contour_seq):
        #seq = cv.FindContours(self.im_contour, cv.CreateMemStorage()) # Finds contours from im_contour, not im_foreground_binary.
        moments = cv.Moments(contour_seq)
        (xROI, yROI, area, theta, ecc) = self.ContourFromMoments(moments)
        
        
        # Save contour info
        #rospy.loginfo ('IP area=%5.5f' % area)
        if True: #area>0.0:
            self.nContours += 1
            
            # Convert from ROIPlateImage coordinates to Camera coordinates
            ptContourROIPlateImage = PointStamped()
            ptContourROIPlateImage.header.frame_id = "ROIPlateImage"
            ptContourROIPlateImage.point.x = xROI
            ptContourROIPlateImage.point.y = yROI
            self.ptOutput = self.tfrx.transformPoint(self.frameidOutput, ptContourROIPlateImage)
        
            self.x0_list.append(self.ptOutput.point.x)
            self.y0_list.append(self.ptOutput.point.y)
            self.theta_list.append(theta)
            self.area_list.append(area)
            self.ecc_list.append(ecc)


    def ShowContourWindow(self, iContour):            
        if self.display_contours:
            cv.NamedWindow("Contour %s"%str(iContours), 1)
            #cv.NamedWindow("Contour", 1)
            cv.CvtColor(self.im_contour, self.im_contour_display, cv.CV_GRAY2RGB)
            
            display_text = "Area = " + str(int(area))
            cv.PutText(self.im_contour_display, display_text,(25,25),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "ecc = " + str(round(ecc,2))
            cv.PutText(self.im_contour_display, display_text,(int(self.ROIPlateImage_width/2),25),self.font,cv.CV_RGB(self.color_max,0,0))
            
            cv.Circle(self.im_contour_display, (int(xROI),int(yROI)), 4, cv.CV_RGB(0,self.color_max,0))
            length = 100
            self.DrawAngleLine(self.im_contour_display, xROI, yROI, angle, ecc, length)
            
            display_text = "x0 = " + str(int(self.ptOutput.point.x))
            cv.PutText(self.im_contour_display, display_text,(25,45),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "y0 = " + str(int(self.ptOutput.point.y))
            cv.PutText(self.im_contour_display, display_text,(int(self.ROIPlateImage_width/2),45),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "theta = " + str(round(theta,2))
            cv.PutText(self.im_contour_display, display_text,(25,65),self.font,cv.CV_RGB(self.color_max,0,0))
            
            display_text = "output coordinates = " + self.frameidOutput
            cv.PutText(self.im_contour_display, display_text,(25,85),self.font,cv.CV_RGB(self.color_max,0,0))
            
            cv.ShowImage("Contour %s" % str(iContours), self.im_contour_display)
            #cv.ShowImage("Contour", self.im_contour_display)

    
      
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
            self.pub_contourinfo.publish(contourinfo)
        
        return contourinfo, contour_seq    
        

    def Image_callback(self, image):
        if not self.initialized:
            return
        
        # Convert ROS image to OpenCV image
        try:
            cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(image, "passthrough"))
        except CvBridgeError, e:
          print e
        
        if not self.initialized_images:
            if self.initialized_coords:
                self.InitializeImages(cv_image)
            else:
                return
        
        self.im = cv_image
        cv.SetImageROI(self.im,self.ROIPlateImage_cvrect)
        
        # Look for new diff_threshold value
        self.diff_threshold = int(rospy.get_param("diff_threshold","30"))
        
        # Apply mask and Subtract background
        cv.And(self.im, self.im_mask, self.im)
        cv.AbsDiff(self.im, self.im_background, self.im_foreground)
        if self.display_diff:
            cv.ShowImage("Diff Image", self.im_foreground)
        
        # Threshold
        cv.Threshold(self.im_foreground, 
                     self.im_foreground_binary, 
                     self.diff_threshold, 
                     self.max_8U, 
                     cv.CV_THRESH_BINARY)
        cv.Threshold(self.im_foreground, 
                     self.im_foreground, 
                     self.diff_threshold, 
                     self.max_8U, 
                     cv.CV_THRESH_TOZERO)
        # cv.Dilate(self.im_foreground,self.im_dilate)
        # cv.Erode(self.im_dilate,self.im_erode)
        # cv.ShowImage("Dilate Plus Erode Image", self.im_erode)

        
        # Get the ContourInfo.
        (self.contourinfo, self.contour_seq) = self.ContourinfoFromImage(self.im_foreground_binary)
        #rospy.logwarn ('IP self.contourinfo=\n%s\n, self.contour_seq\n=%s' % (self.contourinfo,self.contour_seq))
        
        
        # Convert to color for display image
        cv.CvtColor(self.im, self.im_processed, cv.CV_GRAY2RGB)
        
        # Draw contours on Processed image.
        if self.contour_seq:
            cv.DrawContours(self.im_processed, self.contour_seq, cv.CV_RGB(0,0,self.color_max), cv.CV_RGB(0,self.color_max,0), 1, 1)
        
        
        if self.display_processedimage:
            cv.ShowImage("Processed Image", self.im_processed)
        
        if self.display_foreground:
            cv.ShowImage("Foreground Image", self.im_foreground)
        
        if self.display_background:
            cv.ShowImage("Background Image", self.im_background)
        
        cv.WaitKey(3)
        
        
        # Publish processed image
        try:
            cv.Copy(self.im_processed, self.im_processed2)
            self.pub_image_processed.publish(self.bridge.cv_to_imgmsg(self.im_processed2, "passthrough"))
        except CvBridgeError, e:
            print e
        
        # Publish foreground image
        try:
            self.pub_image_foreground.publish(self.bridge.cv_to_imgmsg(self.im_foreground,"passthrough"))
        except CvBridgeError, e:
            print e
          
        # Publish diff image
        try:
            self.pub_image_diff.publish(self.bridge.cv_to_imgmsg(self.im_foreground, "passthrough"))
        except CvBridgeError, e:
            print e

      

def main(args):
    rospy.init_node('ImageProcessor') #, anonymous=True)
    ip = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
  
